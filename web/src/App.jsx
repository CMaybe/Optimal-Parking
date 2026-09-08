import { useEffect, useRef, useState } from "react";

const SCALE = 12; // pixels per meter
const CANVAS_SIZE = 640;

const DEFAULT_OBSTACLES = [
  { x: -6, y: 0, length: 4, width: 2, yaw: 0 },
  { x: 6, y: 0, length: 4, width: 2, yaw: 0 },
  { x: 0, y: -4, length: 4, width: 3, yaw: 0 },
  { x: 0, y: 8, length: 10, width: 3, yaw: 0 }
];

function worldToCanvas(x, y) {
  return [CANVAS_SIZE / 2 + x * SCALE, CANVAS_SIZE / 2 - y * SCALE];
}

function canvasToWorld(px, py) {
  return [(px - CANVAS_SIZE / 2) / SCALE, -(py - CANVAS_SIZE / 2) / SCALE];
}

function drawScene(ctx, { obstacles, initialPose, goalPose, path, draggingIndex }) {
  ctx.clearRect(0, 0, CANVAS_SIZE, CANVAS_SIZE);
  ctx.fillStyle = "#1a1a1a";
  ctx.fillRect(0, 0, CANVAS_SIZE, CANVAS_SIZE);

  // Obstacles.
  obstacles.forEach((obstacle, index) => {
    const [cx, cy] = worldToCanvas(obstacle.x, obstacle.y);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-obstacle.yaw);
    ctx.fillStyle = index === draggingIndex ? "#e07b39" : "#555";
    ctx.fillRect((-obstacle.length / 2) * SCALE, (-obstacle.width / 2) * SCALE, obstacle.length * SCALE, obstacle.width * SCALE);
    ctx.restore();
  });

  // Planned path.
  if (path) {
    ctx.strokeStyle = "#4ea1ff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    path.x.forEach((x, i) => {
      const [px, py] = worldToCanvas(x, path.y[i]);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  drawCar(ctx, initialPose, "#63d471");
  drawCar(ctx, goalPose, "#ff5c8a");
}

function drawCar(ctx, pose, color) {
  const [cx, cy] = worldToCanvas(pose.x, pose.y);
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = color;
  ctx.fillRect(-1.4 * SCALE, -0.8 * SCALE, 2.8 * SCALE, 1.6 * SCALE);
  ctx.restore();
}

function PoseControls({ label, pose, onChange }) {
  return (
    <fieldset style={{ marginBottom: 8 }}>
      <legend>{label}</legend>
      {["x", "y", "yaw"].map((field) => (
        <label key={field} style={{ marginRight: 8 }}>
          {field}
          <input
            type="number"
            step="0.1"
            value={pose[field]}
            onChange={(e) => onChange({ ...pose, [field]: parseFloat(e.target.value) || 0 })}
            style={{ width: 64, marginLeft: 4 }}
          />
        </label>
      ))}
    </fieldset>
  );
}

export default function App() {
  const canvasRef = useRef(null);
  const plannerRef = useRef(null);
  const [status, setStatus] = useState("loading WASM module...");
  const [obstacles, setObstacles] = useState(DEFAULT_OBSTACLES);
  const [initialPose, setInitialPose] = useState({ x: -6, y: 4, yaw: 0 });
  const [goalPose, setGoalPose] = useState({ x: 0, y: 0, yaw: 0 });
  const [path, setPath] = useState(null);
  const draggingIndexRef = useRef(-1);

  useEffect(() => {
    let cancelled = false;
    // planner.js is a runtime asset (not part of the webpack graph); load it as a plain URL.
    import(/* webpackIgnore: true */ "/wasm/planner.js")
      .then(({ default: createPlannerModule }) => createPlannerModule({ locateFile: (path) => `/wasm/${path}` }))
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

  useEffect(() => {
    const ctx = canvasRef.current.getContext("2d");
    drawScene(ctx, { obstacles, initialPose, goalPose, path, draggingIndex: draggingIndexRef.current });
  }, [obstacles, initialPose, goalPose, path]);

  const handlePlan = () => {
    const planner = plannerRef.current;
    if (!planner) return;
    setStatus("planning...");
    planner.setInitialPose(initialPose.x, initialPose.y, initialPose.yaw, 0, 0);
    planner.setGoalPose(goalPose.x, goalPose.y, goalPose.yaw, 0, 0);
    planner.setObstacles(obstacles);
    const result = planner.plan();
    setPath({ x: Array.from(result.x), y: Array.from(result.y) });
    setStatus("ready");
  };

  const hitTestObstacle = (worldX, worldY) =>
    obstacles.findIndex(
      (o) => Math.abs(worldX - o.x) <= o.length / 2 && Math.abs(worldY - o.y) <= o.width / 2
    );

  const handleMouseDown = (event) => {
    const rect = canvasRef.current.getBoundingClientRect();
    const [worldX, worldY] = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top);
    draggingIndexRef.current = hitTestObstacle(worldX, worldY);
  };

  const handleMouseMove = (event) => {
    const index = draggingIndexRef.current;
    if (index < 0) return;
    const rect = canvasRef.current.getBoundingClientRect();
    const [worldX, worldY] = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top);
    setObstacles((current) => current.map((o, i) => (i === index ? { ...o, x: worldX, y: worldY } : o)));
  };

  const handleMouseUp = () => {
    draggingIndexRef.current = -1;
  };

  return (
    <div style={{ display: "flex", gap: 24, padding: 24 }}>
      <canvas
        ref={canvasRef}
        width={CANVAS_SIZE}
        height={CANVAS_SIZE}
        style={{ background: "#000", cursor: "grab" }}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
      />
      <div style={{ minWidth: 260 }}>
        <h2>Optimal Parking</h2>
        <p>{status}</p>
        <PoseControls label="Initial pose (green)" pose={initialPose} onChange={setInitialPose} />
        <PoseControls label="Goal pose (pink)" pose={goalPose} onChange={setGoalPose} />
        <p>Drag the grey rectangles to move obstacles.</p>
        <button onClick={handlePlan} disabled={status !== "ready" && status !== "planning..."}>
          Plan trajectory
        </button>
      </div>
    </div>
  );
}
