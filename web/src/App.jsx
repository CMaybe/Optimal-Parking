import { useEffect, useRef, useState, useCallback } from "react";

const SCALE = 14; // pixels per meter
const SAMPLE_TIME = 0.2; // seconds between path samples (matches example/config.yaml Ts)
const HANDLE_RADIUS_M = 0.35;

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
  qpIterations: 500
};

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

function worldToCanvas(x, y, width, height) {
  return [width / 2 + x * SCALE, height / 2 - y * SCALE];
}

function canvasToWorld(px, py, width, height) {
  return [(px - width / 2) / SCALE, -(py - height / 2) / SCALE];
}

function headingHandlePosition(pose, reach) {
  return { x: pose.x + Math.cos(pose.yaw) * reach, y: pose.y + Math.sin(pose.yaw) * reach };
}

// ---------------------------------------------------------------------------
// Canvas rendering
// ---------------------------------------------------------------------------

function drawGrid(ctx, width, height) {
  ctx.strokeStyle = "#20242c";
  ctx.lineWidth = 1;
  const step = SCALE * 2; // every 2 meters
  for (let x = (width / 2) % step; x < width; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = (height / 2) % step; y < height; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
}

function drawCar(ctx, pose, color, width, height, options = {}) {
  const [cx, cy] = worldToCanvas(pose.x, pose.y, width, height);
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = color;
  ctx.globalAlpha = options.ghost ? 0.55 : 1;
  ctx.fillRect(-1.4 * SCALE, -0.8 * SCALE, 2.8 * SCALE, 1.6 * SCALE);
  ctx.globalAlpha = 1;
  // Heading stripe marks the front of the car.
  ctx.fillStyle = "rgba(0,0,0,0.35)";
  ctx.fillRect(0.6 * SCALE, -0.8 * SCALE, 0.25 * SCALE, 1.6 * SCALE);
  ctx.restore();

  if (options.showHandle) {
    const handle = headingHandlePosition(pose, HANDLE_RADIUS_M + 1.4);
    const [hx, hy] = worldToCanvas(handle.x, handle.y, width, height);
    ctx.beginPath();
    ctx.arc(hx, hy, 6, 0, Math.PI * 2);
    ctx.fillStyle = "#fff";
    ctx.fill();
  }
}

function drawScene(ctx, canvas, state) {
  const { obstacles, initialPose, goalPose, path, animatedPose, dragTarget } = state;
  const { width, height } = canvas;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0c0e12";
  ctx.fillRect(0, 0, width, height);
  drawGrid(ctx, width, height);

  obstacles.forEach((obstacle, index) => {
    const [cx, cy] = worldToCanvas(obstacle.x, obstacle.y, width, height);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-obstacle.yaw);
    const active = dragTarget && dragTarget.type === "obstacle" && dragTarget.index === index;
    ctx.fillStyle = active ? "#e07b39" : "#4a4f58";
    ctx.fillRect((-obstacle.length / 2) * SCALE, (-obstacle.width / 2) * SCALE, obstacle.length * SCALE, obstacle.width * SCALE);
    ctx.restore();
  });

  if (path) {
    ctx.strokeStyle = "#4ea1ff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    path.x.forEach((x, i) => {
      const [px, py] = worldToCanvas(x, path.y[i], width, height);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  drawCar(ctx, initialPose, "#63d471", width, height, { showHandle: dragTarget?.type === "pose" && dragTarget.which === "initial" });
  drawCar(ctx, goalPose, "#ff5c8a", width, height, { showHandle: dragTarget?.type === "pose" && dragTarget.which === "goal" });

  if (animatedPose) {
    drawCar(ctx, animatedPose, "#ffd166", width, height, {});
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
      <span style={{ width: 44, fontSize: 12, color: "#c9d3e0" }}>{label}</span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        style={{ flex: 1 }}
      />
      <span style={{ width: 56, textAlign: "right", fontSize: 12, fontFamily: "monospace" }}>
        {format ? format(value) : value}
      </span>
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
          {["x", "y", "length", "width"].map((field) => (
            <input
              key={field}
              type="number"
              step="0.1"
              value={obstacle[field]}
              onChange={(e) => {
                const value = parseFloat(e.target.value) || 0;
                onChange(obstacles.map((o, i) => (i === index ? { ...o, [field]: value } : o)));
              }}
              title={field}
              style={{ width: 44 }}
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
    </Panel>
  );
}

function ParameterControls({ params, onChange }) {
  return (
    <Panel title="Planner parameters">
      <SliderRow
        label="margin"
        value={params.safetyMargin}
        min={0}
        max={2}
        step={0.05}
        format={(v) => v.toFixed(2)}
        onChange={(v) => onChange({ ...params, safetyMargin: v })}
      />
      <SliderRow
        label="goal k"
        value={params.goalPenalty}
        min={1}
        max={2000}
        step={1}
        onChange={(v) => onChange({ ...params, goalPenalty: v })}
      />
      <SliderRow
        label="obs k"
        value={params.obstaclePenalty}
        min={1}
        max={200}
        step={1}
        onChange={(v) => onChange({ ...params, obstaclePenalty: v })}
      />
      <SliderRow
        label="sqp #"
        value={params.sqpIterations}
        min={1}
        max={100}
        step={1}
        onChange={(v) => onChange({ ...params, sqpIterations: v })}
      />
      <SliderRow
        label="qp #"
        value={params.qpIterations}
        min={10}
        max={2000}
        step={10}
        onChange={(v) => onChange({ ...params, qpIterations: v })}
      />
    </Panel>
  );
}

function ReadoutPanel({ status, log, path, playback, onPlaybackChange }) {
  const logRef = useRef(null);
  useEffect(() => {
    if (logRef.current) logRef.current.scrollTop = logRef.current.scrollHeight;
  }, [log]);

  return (
    <Panel title="Readout" style={{ width: 320 }}>
      <div style={{ fontSize: 12, marginBottom: 6 }}>
        status: <span style={{ color: "#7fd88f" }}>{status}</span>
      </div>
      {path && (
        <div style={{ fontSize: 12, marginBottom: 6 }}>
          points: {path.x.length} · duration: {(path.x.length * SAMPLE_TIME).toFixed(1)}s
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
            max={path.x.length - 1}
            value={playback.index}
            onChange={(e) => onPlaybackChange({ ...playback, playing: false, index: parseInt(e.target.value, 10) })}
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
  const [dragTarget, setDragTarget] = useState(null);
  const [playback, setPlayback] = useState({ playing: false, index: 0 });

  useEffect(() => {
    let cancelled = false;
    // planner.js is a runtime asset (not part of the webpack graph); load it as a plain URL.
    import(/* webpackIgnore: true */ "/wasm/planner.js")
      .then(({ default: createPlannerModule }) =>
        createPlannerModule({
          locateFile: (path) => `/wasm/${path}`,
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

  // Render loop.
  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    const animatedPose =
      path && path.x[playback.index] !== undefined
        ? { x: path.x[playback.index], y: path.y[playback.index], yaw: path.yaw[playback.index] }
        : null;
    drawScene(ctx, canvas, { obstacles, initialPose, goalPose, path, animatedPose, dragTarget });
  }, [obstacles, initialPose, goalPose, path, playback.index, dragTarget]);

  // Playback driver: steps through the path at the same rate the solver sampled it.
  useEffect(() => {
    if (!playback.playing || !path) return undefined;
    const id = setInterval(() => {
      setPlayback((current) => {
        const next = current.index + 1;
        if (next >= path.x.length) return { ...current, playing: false };
        return { ...current, index: next };
      });
    }, SAMPLE_TIME * 1000);
    return () => clearInterval(id);
  }, [playback.playing, path]);

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
    const result = planner.plan();
    const nextPath = { x: Array.from(result.x), y: Array.from(result.y), yaw: Array.from(result.yaw) };
    setPath(nextPath);
    setPlayback({ playing: true, index: 0 });
    setStatus("ready");
  };

  const hitTestObstacle = useCallback(
    (worldX, worldY) =>
      obstacles.findIndex((o) => Math.abs(worldX - o.x) <= o.length / 2 && Math.abs(worldY - o.y) <= o.width / 2),
    [obstacles]
  );

  const hitTestPoseHandle = (worldX, worldY, pose) => {
    const handle = headingHandlePosition(pose, HANDLE_RADIUS_M + 1.4);
    return Math.hypot(worldX - handle.x, worldY - handle.y) <= 0.5;
  };

  const hitTestPoseBody = (worldX, worldY, pose) => Math.hypot(worldX - pose.x, worldY - pose.y) <= 1.6;

  const handleMouseDown = (event) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const [worldX, worldY] = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top, canvas.width, canvas.height);

    if (hitTestPoseHandle(worldX, worldY, initialPose)) return setDragTarget({ type: "pose", which: "initial", mode: "rotate" });
    if (hitTestPoseHandle(worldX, worldY, goalPose)) return setDragTarget({ type: "pose", which: "goal", mode: "rotate" });
    if (hitTestPoseBody(worldX, worldY, initialPose)) return setDragTarget({ type: "pose", which: "initial", mode: "move" });
    if (hitTestPoseBody(worldX, worldY, goalPose)) return setDragTarget({ type: "pose", which: "goal", mode: "move" });

    const obstacleIndex = hitTestObstacle(worldX, worldY);
    if (obstacleIndex >= 0) setDragTarget({ type: "obstacle", index: obstacleIndex });
  };

  const handleMouseMove = (event) => {
    if (!dragTarget) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const [worldX, worldY] = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top, canvas.width, canvas.height);

    if (dragTarget.type === "obstacle") {
      setObstacles((current) => current.map((o, i) => (i === dragTarget.index ? { ...o, x: worldX, y: worldY } : o)));
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

  const handleAddObstacle = () => {
    setObstacles((current) => [...current, { x: 0, y: 0, length: 3, width: 2, yaw: 0 }]);
  };

  const handleRemoveObstacle = (index) => {
    setObstacles((current) => current.filter((_, i) => i !== index));
  };

  const handleDoubleClick = (event) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const [worldX, worldY] = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top, canvas.width, canvas.height);
    const index = hitTestObstacle(worldX, worldY);
    if (index >= 0) handleRemoveObstacle(index);
  };

  return (
    <div style={{ position: "relative", width: "100vw", height: "100vh", overflow: "hidden" }}>
      <canvas
        ref={canvasRef}
        width={window.innerWidth}
        height={window.innerHeight}
        style={{ display: "block", cursor: dragTarget ? "grabbing" : "grab" }}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onDoubleClick={handleDoubleClick}
      />

      <div style={{ position: "absolute", top: 16, left: 16, width: 280 }}>
        <Panel title="Optimal Parking">
          <p style={{ fontSize: 12, color: "#c9d3e0", margin: "4px 0 8px" }}>
            Drag car bodies to move, drag the white dot to rotate. Drag obstacles to move, double-click to delete.
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

      <div style={{ position: "absolute", top: 16, right: 16 }}>
        <ReadoutPanel status={status} log={log} path={path} playback={playback} onPlaybackChange={setPlayback} />
      </div>
    </div>
  );
}

