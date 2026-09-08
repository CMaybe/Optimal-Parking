import { useEffect, useRef, useState, useCallback } from "react";

const BASE_SCALE = 14; // pixels per meter at zoom = 1
const HANDLE_RADIUS_M = 0.35;
const MIN_ZOOM = 0.15;
const MAX_ZOOM = 8;
const MIN_OBSTACLE_SIZE_M = 0.5;
const DEFAULT_VIEW = { centerX: 0, centerY: 2, zoom: 1 };

const DEFAULT_OBSTACLES = [
  { x: -6, y: 0, length: 4, width: 2, yaw: 0 },
  { x: 6, y: 0, length: 4, width: 2, yaw: 0 },
  { x: 0, y: -4, length: 4, width: 3, yaw: 0 },
  { x: 0, y: 8, length: 10, width: 3, yaw: 0 }
];

const DEFAULT_PARAMS = {
  safetyMargin: 0,
  goalPenalty: 200,
  obstaclePenalty: 20,
  sqpIterations: 30,
  qpIterations: 500,
  trajectoryTime: 20,
  sampleTime: 0.2,
  stateWeight: [0.1, 0.1, 0.1, 0.01, 0.01],
  inputWeight: [1.0, 10.0],
  inputLower: [-1.0, -1.0],
  inputUpper: [2.0, 2.0],
  velocitySteerLower: [-10.0, -0.63792],
  velocitySteerUpper: [10.0, 0.63792]
};

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

function viewScale(view) {
  return BASE_SCALE * view.zoom;
}

function worldToCanvas(x, y, width, height, view) {
  const scale = viewScale(view);
  return [width / 2 + (x - view.centerX) * scale, height / 2 - (y - view.centerY) * scale];
}

function canvasToWorld(px, py, width, height, view) {
  const scale = viewScale(view);
  return [(px - width / 2) / scale + view.centerX, -(py - height / 2) / scale + view.centerY];
}

// A point offset by (dx, dy) in the pose's own rotated (heading, left) frame.
function localAxisPoint(pose, dx, dy) {
  const cos = Math.cos(pose.yaw);
  const sin = Math.sin(pose.yaw);
  return { x: pose.x + dx * cos - dy * sin, y: pose.y + dx * sin + dy * cos };
}

function headingHandlePosition(pose, reach) {
  return localAxisPoint(pose, reach, 0);
}

function obstacleRotateHandlePosition(obstacle) {
  return localAxisPoint(obstacle, obstacle.length / 2 + 1, obstacle.width / 2 + 1);
}

function obstacleLengthHandlePosition(obstacle) {
  return localAxisPoint(obstacle, obstacle.length / 2, 0);
}

function obstacleWidthHandlePosition(obstacle) {
  return localAxisPoint(obstacle, 0, obstacle.width / 2);
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

// Interpolates an angle along the shorter direction so it doesn't spin the long way around.
function lerpAngle(a, b, t) {
  const delta = Math.atan2(Math.sin(b - a), Math.cos(b - a));
  return a + delta * t;
}

// ---------------------------------------------------------------------------
// Canvas rendering
// ---------------------------------------------------------------------------

function drawGrid(ctx, width, height, view) {
  ctx.strokeStyle = "#20242c";
  ctx.lineWidth = 1;
  const scale = viewScale(view);
  const step = scale * 2; // every 2 meters
  const [originX, originY] = worldToCanvas(0, 0, width, height, view);
  for (let x = originX % step; x < width; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = originY % step; y < height; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
}

function drawHandleDot(ctx, worldPoint, width, height, view, color = "#fff") {
  const [hx, hy] = worldToCanvas(worldPoint.x, worldPoint.y, width, height, view);
  ctx.beginPath();
  ctx.arc(hx, hy, 6, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

function drawHandleSquare(ctx, worldPoint, width, height, view, color = "#ffb454") {
  const [hx, hy] = worldToCanvas(worldPoint.x, worldPoint.y, width, height, view);
  ctx.fillStyle = color;
  ctx.fillRect(hx - 5, hy - 5, 10, 10);
}

function drawCar(ctx, pose, color, width, height, view, options = {}) {
  const scale = viewScale(view);
  const [cx, cy] = worldToCanvas(pose.x, pose.y, width, height, view);
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = color;
  ctx.globalAlpha = options.ghost ? 0.55 : 1;
  ctx.fillRect(-1.4 * scale, -0.8 * scale, 2.8 * scale, 1.6 * scale);
  ctx.globalAlpha = 1;
  // Heading stripe marks the front of the car.
  ctx.fillStyle = "rgba(0,0,0,0.35)";
  ctx.fillRect(0.6 * scale, -0.8 * scale, 0.25 * scale, 1.6 * scale);
  ctx.restore();

  if (options.showHandle) {
    drawHandleDot(ctx, headingHandlePosition(pose, HANDLE_RADIUS_M + 1.4), width, height, view);
  }
}

function drawScene(ctx, canvas, state) {
  const { obstacles, initialPose, goalPose, path, animatedPose, dragTarget, selection, view } = state;
  const { width, height } = canvas;
  const scale = viewScale(view);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0c0e12";
  ctx.fillRect(0, 0, width, height);
  drawGrid(ctx, width, height, view);

  obstacles.forEach((obstacle, index) => {
    const isSelected = selection?.type === "obstacle" && selection.index === index;
    const isDragging = dragTarget?.type === "obstacle" && dragTarget.index === index;
    const [cx, cy] = worldToCanvas(obstacle.x, obstacle.y, width, height, view);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-obstacle.yaw);
    ctx.fillStyle = isDragging ? "#e07b39" : isSelected ? "#6b7280" : "#4a4f58";
    ctx.fillRect((-obstacle.length / 2) * scale, (-obstacle.width / 2) * scale, obstacle.length * scale, obstacle.width * scale);
    if (isSelected) {
      ctx.strokeStyle = "#ffb454";
      ctx.lineWidth = 2;
      ctx.strokeRect((-obstacle.length / 2) * scale, (-obstacle.width / 2) * scale, obstacle.length * scale, obstacle.width * scale);
    }
    ctx.restore();

    if (isSelected) {
      drawHandleDot(ctx, obstacleRotateHandlePosition(obstacle), width, height, view);
      drawHandleSquare(ctx, obstacleLengthHandlePosition(obstacle), width, height, view);
      drawHandleSquare(ctx, obstacleWidthHandlePosition(obstacle), width, height, view);
    }
  });

  if (path) {
    ctx.strokeStyle = "#4ea1ff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    path.x.forEach((x, i) => {
      const [px, py] = worldToCanvas(x, path.y[i], width, height, view);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  drawCar(ctx, initialPose, "#63d471", width, height, view, {
    showHandle: selection?.type === "pose" && selection.which === "initial"
  });
  drawCar(ctx, goalPose, "#ff5c8a", width, height, view, {
    showHandle: selection?.type === "pose" && selection.which === "goal"
  });

  if (animatedPose) {
    drawCar(ctx, animatedPose, "#ffd166", width, height, view, {});
  }
}

// ---------------------------------------------------------------------------
// UI subcomponents
// ---------------------------------------------------------------------------

function Panel({ title, children, style }) {
  return (
    <div
      style={{
        background: "rgba(18, 20, 26, 0.88)",
        border: "1px solid #2a2f3a",
        borderRadius: 8,
        padding: "10px 14px",
        marginBottom: 12,
        backdropFilter: "blur(4px)",
        ...style
      }}
    >
      {title && <div style={{ fontWeight: 600, fontSize: 13, marginBottom: 8, color: "#9fb4d1" }}>{title}</div>}
      {children}
    </div>
  );
}

function SliderRow({ label, value, min, max, step, onChange, format }) {
  return (
    <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 6 }}>
      <span style={{ width: 108, flexShrink: 0, whiteSpace: "nowrap", fontSize: 11, color: "#c9d3e0" }}>{label}</span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        style={{ flex: 1, minWidth: 0 }}
      />
      <span style={{ width: 60, flexShrink: 0, textAlign: "right", fontSize: 11, fontFamily: "monospace" }}>
        {format ? format(value) : value}
      </span>
    </div>
  );
}

// A labelled group of number inputs, one per vector component (e.g. state weight [x,y,yaw,v,steer]).
// Each component gets its own visible mini-label so the numbers aren't just anonymous boxes.
function VectorRow({ label, labels, values, step = 0.01, onChange }) {
  return (
    <div style={{ marginBottom: 10 }}>
      <div style={{ fontSize: 11, color: "#9fb4d1", marginBottom: 4 }}>{label}</div>
      <div style={{ display: "flex", gap: 6, flexWrap: "wrap" }}>
        {values.map((value, i) => (
          <label key={i} style={{ display: "flex", flexDirection: "column", flex: "1 1 64px", minWidth: 60 }}>
            <span style={{ fontSize: 10, color: "#8b93a3", marginBottom: 2, whiteSpace: "nowrap" }}>{labels?.[i] ?? i}</span>
            <input
              type="number"
              step={step}
              value={value}
              onChange={(e) => {
                const next = values.slice();
                next[i] = parseFloat(e.target.value) || 0;
                onChange(next);
              }}
              style={{ width: "100%", fontSize: 11, boxSizing: "border-box" }}
            />
          </label>
        ))}
      </div>
    </div>
  );
}

function PoseControls({ label, color, pose, onChange }) {
  return (
    <Panel title={<span style={{ color }}>{label}</span>}>
      <SliderRow label="x (m)" value={pose.x} min={-15} max={15} step={0.1} onChange={(v) => onChange({ ...pose, x: v })} />
      <SliderRow label="y (m)" value={pose.y} min={-15} max={15} step={0.1} onChange={(v) => onChange({ ...pose, y: v })} />
      <SliderRow
        label="yaw"
        value={pose.yaw}
        min={-Math.PI}
        max={Math.PI}
        step={0.01}
        format={(v) => v.toFixed(2)}
        onChange={(v) => onChange({ ...pose, yaw: v })}
      />
    </Panel>
  );
}

function ObstaclesPanel({ obstacles, onChange, onAdd, onRemove }) {
  return (
    <Panel title="Obstacles">
      {obstacles.map((obstacle, index) => (
        <div key={index} style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 6 }}>
          <span style={{ width: 16, fontSize: 12, color: "#c9d3e0" }}>{index + 1}</span>
          {["x", "y", "length", "width", "yaw"].map((field) => (
            <input
              key={field}
              type="number"
              step="0.1"
              value={field === "yaw" ? obstacle[field].toFixed(2) : obstacle[field]}
              onChange={(e) => {
                const value = parseFloat(e.target.value) || 0;
                onChange(obstacles.map((o, i) => (i === index ? { ...o, [field]: value } : o)));
              }}
              title={field}
              style={{ width: 0, flex: 1, minWidth: 34, fontSize: 11 }}
            />
          ))}
          <button onClick={() => onRemove(index)} title="remove" style={{ padding: "2px 6px" }}>
            ×
          </button>
        </div>
      ))}
      <button onClick={onAdd} style={{ width: "100%", marginTop: 4 }}>
        + Add obstacle
      </button>
      <p style={{ fontSize: 11, color: "#8b93a3", margin: "6px 0 0" }}>
        Select an obstacle on the canvas to drag its rotate (dot) and resize (square) handles.
      </p>
    </Panel>
  );
}

function ParameterControls({ params, onChange }) {
  const set = (patch) => onChange({ ...params, ...patch });
  return (
    <Panel title="Planner parameters">
      <SliderRow
        label="Safety margin"
        value={params.safetyMargin}
        min={0}
        max={2}
        step={0.05}
        format={(v) => v.toFixed(2)}
        onChange={(v) => set({ safetyMargin: v })}
      />
      <SliderRow
        label="Goal penalty"
        value={params.goalPenalty}
        min={1}
        max={2000}
        step={1}
        onChange={(v) => set({ goalPenalty: v })}
      />
      <SliderRow
        label="Obstacle penalty"
        value={params.obstaclePenalty}
        min={1}
        max={200}
        step={1}
        onChange={(v) => set({ obstaclePenalty: v })}
      />
      <SliderRow
        label="SQP iterations"
        value={params.sqpIterations}
        min={1}
        max={100}
        step={1}
        onChange={(v) => set({ sqpIterations: v })}
      />
      <SliderRow
        label="QP iterations"
        value={params.qpIterations}
        min={10}
        max={2000}
        step={10}
        onChange={(v) => set({ qpIterations: v })}
      />
      <SliderRow
        label="Horizon"
        value={params.trajectoryTime}
        min={2}
        max={60}
        step={1}
        format={(v) => `${v}s`}
        onChange={(v) => set({ trajectoryTime: v })}
      />
      <SliderRow
        label="Sample time"
        value={params.sampleTime}
        min={0.05}
        max={1}
        step={0.05}
        format={(v) => v.toFixed(2)}
        onChange={(v) => set({ sampleTime: v })}
      />
      <VectorRow
        label="State weight"
        labels={["x", "y", "yaw", "v", "steer"]}
        values={params.stateWeight}
        onChange={(v) => set({ stateWeight: v })}
      />
      <VectorRow
        label="Input weight"
        labels={["accel", "steer rate"]}
        values={params.inputWeight}
        onChange={(v) => set({ inputWeight: v })}
      />
      <VectorRow
        label="Input lower bound"
        labels={["accel", "steer rate"]}
        values={params.inputLower}
        onChange={(v) => set({ inputLower: v })}
      />
      <VectorRow
        label="Input upper bound"
        labels={["accel", "steer rate"]}
        values={params.inputUpper}
        onChange={(v) => set({ inputUpper: v })}
      />
      <VectorRow
        label="Velocity / steer lower bound"
        labels={["velocity", "steer"]}
        values={params.velocitySteerLower}
        onChange={(v) => set({ velocitySteerLower: v })}
      />
      <VectorRow
        label="Velocity / steer upper bound"
        labels={["velocity", "steer"]}
        values={params.velocitySteerUpper}
        onChange={(v) => set({ velocitySteerUpper: v })}
      />
    </Panel>
  );
}

function ReadoutPanel({ status, log, path, sampleTime, playback, onPlaybackChange }) {
  const logRef = useRef(null);
  useEffect(() => {
    if (logRef.current) logRef.current.scrollTop = logRef.current.scrollHeight;
  }, [log]);

  const duration = path ? (path.x.length - 1) * sampleTime : 0;

  return (
    <Panel title="Readout" style={{ width: 320 }}>
      <div style={{ fontSize: 12, marginBottom: 6 }}>
        status: <span style={{ color: "#7fd88f" }}>{status}</span>
      </div>
      {path && (
        <div style={{ fontSize: 12, marginBottom: 6 }}>
          points: {path.x.length} · duration: {duration.toFixed(1)}s
        </div>
      )}
      {path && (
        <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 8 }}>
          <button onClick={() => onPlaybackChange({ ...playback, playing: !playback.playing })}>
            {playback.playing ? "Pause" : "Play"}
          </button>
          <input
            type="range"
            min={0}
            max={duration}
            step={duration / 500}
            value={playback.time}
            onChange={(e) => onPlaybackChange({ ...playback, playing: false, time: parseFloat(e.target.value) })}
            style={{ flex: 1 }}
          />
        </div>
      )}
      <div
        ref={logRef}
        style={{
          fontFamily: "monospace",
          fontSize: 11,
          height: 140,
          overflowY: "auto",
          background: "#05060a",
          padding: 6,
          borderRadius: 4,
          whiteSpace: "pre-wrap"
        }}
      >
        {log.join("\n")}
      </div>
    </Panel>
  );
}

// ---------------------------------------------------------------------------
// Main app
// ---------------------------------------------------------------------------

export default function App() {
  const canvasRef = useRef(null);
  const plannerRef = useRef(null);
  const [status, setStatus] = useState("loading WASM module...");
  const [log, setLog] = useState([]);
  const [obstacles, setObstacles] = useState(DEFAULT_OBSTACLES);
  const [initialPose, setInitialPose] = useState({ x: -6, y: 4, yaw: 0 });
  const [goalPose, setGoalPose] = useState({ x: 0, y: 0, yaw: 0 });
  const [params, setParams] = useState(DEFAULT_PARAMS);
  const [path, setPath] = useState(null);
  const [pathSampleTime, setPathSampleTime] = useState(DEFAULT_PARAMS.sampleTime);
  const [dragTarget, setDragTarget] = useState(null);
  const [selection, setSelection] = useState(null);
  // playback.time is a continuous seconds value driven by requestAnimationFrame, giving smooth interpolation.
  const [playback, setPlayback] = useState({ playing: false, time: 0 });
  const [view, setView] = useState(DEFAULT_VIEW);
  const [canvasSize, setCanvasSize] = useState({ width: window.innerWidth, height: window.innerHeight });
  const [sidebarWidth, setSidebarWidth] = useState(320);

  useEffect(() => {
    const handleResize = () => setCanvasSize({ width: window.innerWidth, height: window.innerHeight });
    window.addEventListener("resize", handleResize);
    return () => window.removeEventListener("resize", handleResize);
  }, []);

  useEffect(() => {
    let cancelled = false;
    // planner.js is a runtime asset (not part of the webpack graph); load it as a plain URL.
    // Relative (not "/wasm/...") so this also works under a GitHub Pages project subpath.
    import(/* webpackIgnore: true */ "./wasm/planner.js")
      .then(({ default: createPlannerModule }) =>
        createPlannerModule({
          locateFile: (path) => `./wasm/${path}`,
          print: (line) => setLog((current) => [...current.slice(-199), line]),
          printErr: (line) => setLog((current) => [...current.slice(-199), line])
        })
      )
      .then((module) => {
        if (cancelled) return;
        plannerRef.current = new module.Planner("/config.yaml");
        setStatus("ready");
      })
      .catch((error) => setStatus(`failed to load WASM module: ${error}`));
    return () => {
      cancelled = true;
    };
  }, []);

  // Continuous playback driver: advances by real elapsed time via requestAnimationFrame
  // instead of stepping once per sample, so the car glides smoothly between path points.
  useEffect(() => {
    if (!playback.playing || !path) return undefined;
    let frameId;
    let lastTimestamp;
    const duration = (path.x.length - 1) * pathSampleTime;

    const tick = (timestamp) => {
      if (lastTimestamp === undefined) lastTimestamp = timestamp;
      const dt = (timestamp - lastTimestamp) / 1000;
      lastTimestamp = timestamp;
      setPlayback((current) => {
        const next = current.time + dt;
        if (next >= duration) return { playing: false, time: duration };
        return { ...current, time: next };
      });
      frameId = requestAnimationFrame(tick);
    };
    frameId = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frameId);
  }, [playback.playing, path, pathSampleTime]);

  // Render loop.
  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    let animatedPose = null;
    if (path) {
      const floatIndex = Math.min(playback.time / pathSampleTime, path.x.length - 1);
      const i0 = Math.floor(floatIndex);
      const i1 = Math.min(i0 + 1, path.x.length - 1);
      const t = floatIndex - i0;
      animatedPose = {
        x: lerp(path.x[i0], path.x[i1], t),
        y: lerp(path.y[i0], path.y[i1], t),
        yaw: lerpAngle(path.yaw[i0], path.yaw[i1], t)
      };
    }
    drawScene(ctx, canvas, { obstacles, initialPose, goalPose, path, animatedPose, dragTarget, selection, view });
  }, [obstacles, initialPose, goalPose, path, playback.time, pathSampleTime, dragTarget, selection, view, canvasSize]);

  const handlePlan = () => {
    const planner = plannerRef.current;
    if (!planner) return;
    setStatus("planning...");
    setLog([]);
    planner.setInitialPose(initialPose.x, initialPose.y, initialPose.yaw, 0, 0);
    planner.setGoalPose(goalPose.x, goalPose.y, goalPose.yaw, 0, 0);
    planner.setObstacles(obstacles);
    planner.setSafetyMargin(params.safetyMargin);
    planner.setGoalPenalty(params.goalPenalty);
    planner.setObstaclePenalty(params.obstaclePenalty);
    planner.setSqpIterations(params.sqpIterations);
    planner.setQpIterations(params.qpIterations);
    planner.setHorizon(params.trajectoryTime, params.sampleTime);
    planner.setStateWeight(...params.stateWeight);
    planner.setInputWeight(...params.inputWeight);
    planner.setInputBounds(params.inputLower[0], params.inputLower[1], params.inputUpper[0], params.inputUpper[1]);
    planner.setVelocitySteerBounds(
      params.velocitySteerLower[0],
      params.velocitySteerLower[1],
      params.velocitySteerUpper[0],
      params.velocitySteerUpper[1]
    );
    const result = planner.plan();
    const nextPath = { x: Array.from(result.x), y: Array.from(result.y), yaw: Array.from(result.yaw) };
    setPath(nextPath);
    setPathSampleTime(params.sampleTime);
    setPlayback({ playing: true, time: 0 });
    setStatus("ready");
  };

  const hitTestObstacle = useCallback(
    (worldX, worldY) =>
      obstacles.findIndex((o) => Math.abs(worldX - o.x) <= o.length / 2 && Math.abs(worldY - o.y) <= o.width / 2),
    [obstacles]
  );

  const hitTestPoint = (worldX, worldY, point, radius) => Math.hypot(worldX - point.x, worldY - point.y) <= radius;

  const hitTestPoseHandle = (worldX, worldY, pose) => hitTestPoint(worldX, worldY, headingHandlePosition(pose, HANDLE_RADIUS_M + 1.4), 0.5);

  const hitTestPoseBody = (worldX, worldY, pose) => hitTestPoint(worldX, worldY, pose, 1.6);

  const handleMouseDown = (event) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const [worldX, worldY] = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top, canvas.width, canvas.height, view);

    // Handles of the currently selected object take priority so they stay clickable
    // even though they're drawn slightly away from the object body.
    if (selection?.type === "pose") {
      const pose = selection.which === "initial" ? initialPose : goalPose;
      if (hitTestPoseHandle(worldX, worldY, pose)) return setDragTarget({ type: "pose", which: selection.which, mode: "rotate" });
    }
    if (selection?.type === "obstacle") {
      const obstacle = obstacles[selection.index];
      if (hitTestPoint(worldX, worldY, obstacleRotateHandlePosition(obstacle), 0.5)) {
        return setDragTarget({ type: "obstacle", index: selection.index, mode: "rotate" });
      }
      if (hitTestPoint(worldX, worldY, obstacleLengthHandlePosition(obstacle), 0.45)) {
        return setDragTarget({ type: "obstacle", index: selection.index, mode: "resize-length" });
      }
      if (hitTestPoint(worldX, worldY, obstacleWidthHandlePosition(obstacle), 0.45)) {
        return setDragTarget({ type: "obstacle", index: selection.index, mode: "resize-width" });
      }
    }

    if (hitTestPoseBody(worldX, worldY, initialPose)) {
      setSelection({ type: "pose", which: "initial" });
      return setDragTarget({ type: "pose", which: "initial", mode: "move" });
    }
    if (hitTestPoseBody(worldX, worldY, goalPose)) {
      setSelection({ type: "pose", which: "goal" });
      return setDragTarget({ type: "pose", which: "goal", mode: "move" });
    }

    const obstacleIndex = hitTestObstacle(worldX, worldY);
    if (obstacleIndex >= 0) {
      setSelection({ type: "obstacle", index: obstacleIndex });
      return setDragTarget({ type: "obstacle", index: obstacleIndex, mode: "move" });
    }

    setSelection(null);
    setDragTarget({
      type: "pan",
      startClientX: event.clientX,
      startClientY: event.clientY,
      startCenterX: view.centerX,
      startCenterY: view.centerY
    });
  };

  const handleMouseMove = (event) => {
    if (!dragTarget) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();

    if (dragTarget.type === "pan") {
      const scale = viewScale(view);
      const dx = event.clientX - dragTarget.startClientX;
      const dy = event.clientY - dragTarget.startClientY;
      setView((current) => ({
        ...current,
        centerX: dragTarget.startCenterX - dx / scale,
        centerY: dragTarget.startCenterY + dy / scale
      }));
      return;
    }

    const [worldX, worldY] = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top, canvas.width, canvas.height, view);

    if (dragTarget.type === "obstacle") {
      setObstacles((current) =>
        current.map((o, i) => {
          if (i !== dragTarget.index) return o;
          if (dragTarget.mode === "rotate") return { ...o, yaw: Math.atan2(worldY - o.y, worldX - o.x) };
          if (dragTarget.mode === "resize-length" || dragTarget.mode === "resize-width") {
            const alongLength = dragTarget.mode === "resize-length";
            const dirX = alongLength ? Math.cos(o.yaw) : -Math.sin(o.yaw);
            const dirY = alongLength ? Math.sin(o.yaw) : Math.cos(o.yaw);
            const projection = (worldX - o.x) * dirX + (worldY - o.y) * dirY;
            const size = Math.max(MIN_OBSTACLE_SIZE_M, 2 * Math.abs(projection));
            return alongLength ? { ...o, length: size } : { ...o, width: size };
          }
          return { ...o, x: worldX, y: worldY };
        })
      );
      return;
    }

    const setPose = dragTarget.which === "initial" ? setInitialPose : setGoalPose;
    if (dragTarget.mode === "move") {
      setPose((current) => ({ ...current, x: worldX, y: worldY }));
    } else {
      setPose((current) => ({ ...current, yaw: Math.atan2(worldY - current.y, worldX - current.x) }));
    }
  };

  const handleMouseUp = () => setDragTarget(null);

  // React attaches onWheel as a passive listener, which can't call preventDefault;
  // attach natively instead so scrolling the page doesn't fight with zooming.
  useEffect(() => {
    const canvas = canvasRef.current;
    const handleWheel = (event) => {
      event.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const cursorPx = event.clientX - rect.left;
      const cursorPy = event.clientY - rect.top;

      setView((current) => {
        const [worldXBefore, worldYBefore] = canvasToWorld(cursorPx, cursorPy, canvas.width, canvas.height, current);
        const zoomFactor = event.deltaY < 0 ? 1.15 : 1 / 1.15;
        const newZoom = Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, current.zoom * zoomFactor));
        const newScale = BASE_SCALE * newZoom;
        return {
          zoom: newZoom,
          centerX: worldXBefore - (cursorPx - canvas.width / 2) / newScale,
          centerY: worldYBefore + (cursorPy - canvas.height / 2) / newScale
        };
      });
    };
    canvas.addEventListener("wheel", handleWheel, { passive: false });
    return () => canvas.removeEventListener("wheel", handleWheel);
  }, []);

  const handleZoomButton = (factor) => {
    setView((current) => ({ ...current, zoom: Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, current.zoom * factor)) }));
  };

  const handleResetView = () => setView(DEFAULT_VIEW);

  const handleAddObstacle = () => {
    setObstacles((current) => [...current, { x: 0, y: 0, length: 3, width: 2, yaw: 0 }]);
  };

  const handleRemoveObstacle = (index) => {
    setObstacles((current) => current.filter((_, i) => i !== index));
    setSelection((current) => (current?.type === "obstacle" && current.index === index ? null : current));
  };

  const handleDoubleClick = (event) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const [worldX, worldY] = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top, canvas.width, canvas.height, view);
    const index = hitTestObstacle(worldX, worldY);
    if (index >= 0) handleRemoveObstacle(index);
  };

  // Drag the sidebar's right edge to resize it; tracked with document-level
  // listeners since the drag can move outside the handle/sidebar itself.
  const handleSidebarResizeStart = (event) => {
    event.preventDefault();
    const startX = event.clientX;
    const startWidth = sidebarWidth;
    const handleMove = (moveEvent) => {
      const next = startWidth + (moveEvent.clientX - startX);
      setSidebarWidth(Math.min(520, Math.max(220, next)));
    };
    const handleUp = () => {
      document.removeEventListener("mousemove", handleMove);
      document.removeEventListener("mouseup", handleUp);
    };
    document.addEventListener("mousemove", handleMove);
    document.addEventListener("mouseup", handleUp);
  };

  return (
    <div style={{ position: "relative", width: "100vw", height: "100vh", overflow: "hidden" }}>
      <canvas
        ref={canvasRef}
        width={canvasSize.width}
        height={canvasSize.height}
        style={{ display: "block", cursor: dragTarget ? "grabbing" : "grab" }}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onDoubleClick={handleDoubleClick}
      />

      <div style={{ position: "absolute", bottom: 16, right: 16, display: "flex", gap: 6 }}>
        <button onClick={() => handleZoomButton(1 / 1.3)}>−</button>
        <button onClick={handleResetView}>{Math.round(view.zoom * 100)}%</button>
        <button onClick={() => handleZoomButton(1.3)}>+</button>
      </div>

      <div
        style={{
          position: "absolute",
          top: 16,
          left: 16,
          width: sidebarWidth,
          maxHeight: "calc(100vh - 32px)",
          overflowY: "auto",
          overflowX: "hidden"
        }}
      >
        <Panel title="Optimal Parking">
          <p style={{ fontSize: 12, color: "#c9d3e0", margin: "4px 0 8px" }}>
            Click a car or obstacle to select it, then drag its dot (rotate) or squares (resize). Drag empty space to
            pan, scroll to zoom, double-click an obstacle to delete it.
          </p>
          <button onClick={handlePlan} disabled={status !== "ready"} style={{ width: "100%", padding: 8 }}>
            Plan trajectory
          </button>
        </Panel>
        <PoseControls label="Initial pose" color="#63d471" pose={initialPose} onChange={setInitialPose} />
        <PoseControls label="Goal pose" color="#ff5c8a" pose={goalPose} onChange={setGoalPose} />
        <ObstaclesPanel obstacles={obstacles} onChange={setObstacles} onAdd={handleAddObstacle} onRemove={handleRemoveObstacle} />
        <ParameterControls params={params} onChange={setParams} />
      </div>

      <div
        onMouseDown={handleSidebarResizeStart}
        title="Drag to resize"
        style={{
          position: "absolute",
          top: 16,
          left: sidebarWidth + 16,
          width: 6,
          height: "calc(100vh - 32px)",
          cursor: "ew-resize"
        }}
      />

      <div style={{ position: "absolute", top: 16, right: 16 }}>
        <ReadoutPanel
          status={status}
          log={log}
          path={path}
          sampleTime={pathSampleTime}
          playback={playback}
          onPlaybackChange={setPlayback}
        />
      </div>
    </div>
  );
}

