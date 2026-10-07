/** @typedef {import("./core/dashboard-core.js").DashboardCore} DashboardCore */
/** @typedef {import("./core/dashboard-core.js").DashboardTopic} DashboardTopic */

const POSE_TOPIC = "/AdvantageKit/RealOutputs/Odometry/Robot";
const SPEED_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/Drive/SpeedMetersPerSec";
const ALLIANCE_TOPIC = "/FMSInfo/IsRedAlliance";
const STATION_TOPIC = "/FMSInfo/StationNumber";
const B_COUNT_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/BPressCount";
const B_HELD_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/BHeld";

// 2026 field size and robot bumper size, in meters.
const FIELD = { length: 17.548, width: 8.052 };
const ROBOT_SIZE_METERS = 0.7112;

/** A fresh, editable dashboard. Add a card here for everything you want to see. */
export class CustomDashboard extends HTMLElement {
  /** @type {DashboardCore | undefined} */
  #core;
  /** @type {boolean} */
  #started = false;
  /** @type {Array<() => void>} */
  #cleanup = [];

  connectedCallback() {
    const root = this.shadowRoot ?? this.attachShadow({ mode: "open" });
    if (!root.hasChildNodes()) {
      const version =
        new URL(import.meta.url).searchParams.get("v") ?? "initial";
      root.innerHTML = `<link rel="stylesheet" href="custom-dashboard.css?v=${version}">
        <main>
          <section class="card field-card" aria-labelledby="field-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Drivetrain</p><h1 id="field-heading">Field</h1></div>
              <output class="field-pose" aria-live="polite">Waiting for pose…</output>
            </div>
            <div class="field-view"><canvas></canvas></div>
          </section>
          <section class="card" aria-labelledby="buttons-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Tutorial</p><h1 id="buttons-heading">B Button</h1></div>
            </div>
            <div class="button-stats">
              <div><p class="eyebrow">Presses (onTrue)</p><output class="big-number b-count">0</output></div>
              <div><p class="eyebrow">Held (whileTrue)</p><output class="status-light b-held">NOT HELD</output></div>
            </div>
          </section>
        </main>`;
    }
    if (this.#core && !this.#started) this.#start(this.#core);
  }

  disconnectedCallback() {
    this.#cleanup.forEach((cleanup) => cleanup());
    this.#cleanup = [];
    this.#started = false;
    this.shadowRoot?.replaceChildren();
  }

  /** @param {DashboardCore} core The shared protected core API. */
  set core(core) {
    this.#core = core;
    if (!this.#started && this.isConnected && this.shadowRoot?.hasChildNodes())
      this.#start(core);
  }

  /** @returns {DashboardCore | undefined} */
  get core() {
    return this.#core;
  }

  /** @param {DashboardCore} core The shared protected core API. */
  #start(core) {
    this.#started = true;
    this.#startFieldCard(core);
    this.#startButtonCard(core);
  }

  /** Shows the B press counter and whether B is held right now. */
  #startButtonCard(core) {
    const root = this.shadowRoot;
    const countText = root.querySelector(".b-count");
    const heldLight = root.querySelector(".b-held");

    const render = () => {
      const count = Number(core.getTopic(B_COUNT_TOPIC)?.value);
      const held = core.getTopic(B_HELD_TOPIC)?.value === true;
      countText.textContent = Number.isFinite(count) ? String(count) : "0";
      heldLight.textContent = held ? "HELD" : "NOT HELD";
      heldLight.classList.toggle("on", held);
    };

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if ([B_COUNT_TOPIC, B_HELD_TOPIC].includes(topic.name)) render();
    };
    core.addEventListener("topic", topicListener);
    this.#cleanup.push(() => core.removeEventListener("topic", topicListener));
    render();
  }

  /** Draws the field from the driver's point of view, with the robot on it. */
  #startFieldCard(core) {
    const root = this.shadowRoot;
    const view = root.querySelector(".field-view");
    const canvas = root.querySelector(".field-view canvas");
    const poseText = root.querySelector(".field-pose");
    let frame = 0;

    const render = () => {
      frame = 0;
      const isRed = core.getTopic(ALLIANCE_TOPIC)?.value === true;
      const station = Number(core.getTopic(STATION_TOPIC)?.value);
      const pose = decodePose2d(core.getTopic(POSE_TOPIC)?.value);
      const speed = Number(core.getTopic(SPEED_TOPIC)?.value);
      drawField(canvas, view.getBoundingClientRect(), { isRed, station, pose });
      const speedText = Number.isFinite(speed) ? ` · ${speed.toFixed(2)} m/s` : "";
      poseText.textContent = pose
        ? `x ${pose.x.toFixed(2)} · y ${pose.y.toFixed(2)} · ${Math.round((pose.heading * 180) / Math.PI)}°${speedText}`
        : "Waiting for pose…";
    };
    // Pose updates arrive very often, so draw at most once per screen refresh.
    const scheduleRender = () => {
      if (!frame) frame = requestAnimationFrame(render);
    };

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if ([POSE_TOPIC, ALLIANCE_TOPIC, STATION_TOPIC, SPEED_TOPIC].includes(topic.name))
        scheduleRender();
    };
    core.addEventListener("topic", topicListener);
    const resizeObserver = new ResizeObserver(scheduleRender);
    resizeObserver.observe(view);
    this.#cleanup.push(() => {
      core.removeEventListener("topic", topicListener);
      resizeObserver.disconnect();
      cancelAnimationFrame(frame);
    });
    scheduleRender();
  }
}

/**
 * Draws the field so your alliance wall is at the bottom, like standing behind the glass.
 * @param {HTMLCanvasElement} canvas
 * @param {DOMRect} bounds
 * @param {{ isRed: boolean, station: number, pose: { x: number, y: number, heading: number } | null }} state
 */
function drawField(canvas, bounds, { isRed, station, pose }) {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(bounds.width * ratio));
  const height = Math.max(1, Math.round(bounds.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, bounds.width, bounds.height);

  const padding = 10;
  const bandHeight = 30;
  const bandGap = 5;
  const scale = Math.min(
    (bounds.width - padding * 2) / FIELD.width,
    (bounds.height - padding * 2 - bandHeight - bandGap) / FIELD.length,
  );
  const drawnWidth = FIELD.width * scale;
  const drawnHeight = FIELD.length * scale;
  const left = (bounds.width - drawnWidth) / 2;
  const top = (bounds.height - drawnHeight - bandGap - bandHeight) / 2;

  // Carpet and quarter lines.
  context.fillStyle = "#202630";
  context.strokeStyle = "#7c8798";
  context.lineWidth = 1;
  context.fillRect(left, top, drawnWidth, drawnHeight);
  context.strokeRect(left, top, drawnWidth, drawnHeight);
  context.setLineDash([4, 5]);
  context.strokeStyle = "#9ca89b66";
  for (const fraction of [0.25, 0.5, 0.75]) {
    const y = top + drawnHeight * fraction;
    context.beginPath();
    context.moveTo(left, y);
    context.lineTo(left + drawnWidth, y);
    context.stroke();
  }
  context.setLineDash([]);

  // "Your alliance" band below the field.
  const bandTop = top + drawnHeight + bandGap;
  context.fillStyle = isRed ? "#b52c3c" : "#2879cf";
  context.fillRect(left, bandTop, drawnWidth, bandHeight);
  context.strokeStyle = isRed ? "#ff6877" : "#63b2ff";
  context.strokeRect(left, bandTop, drawnWidth, bandHeight);
  const stationLabel =
    Number.isInteger(station) && station >= 1 && station <= 3
      ? ` · STATION ${station}`
      : "";
  context.fillStyle = "#ffffff";
  context.font = "700 10px system-ui, sans-serif";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(
    `YOUR ALLIANCE${stationLabel}`,
    left + drawnWidth / 2,
    bandTop + bandHeight / 2,
  );

  if (pose) drawRobot(context, pose, { isRed, left, top, scale });
}

/** Converts field meters to screen pixels, flipping the view for the red alliance. */
function fieldToScreen(fieldX, fieldY, { isRed, left, top, scale }) {
  return {
    x: left + (isRed ? fieldY : FIELD.width - fieldY) * scale,
    y: top + (isRed ? fieldX : FIELD.length - fieldX) * scale,
  };
}

/** Draws the robot as a square with an arrow pointing where its front faces. */
function drawRobot(context, pose, view) {
  const center = fieldToScreen(pose.x, pose.y, view);
  const front = fieldToScreen(
    pose.x + Math.cos(pose.heading),
    pose.y + Math.sin(pose.heading),
    view,
  );
  const size = Math.max(10, ROBOT_SIZE_METERS * view.scale);
  context.save();
  context.translate(center.x, center.y);
  context.rotate(Math.atan2(front.y - center.y, front.x - center.x));
  context.shadowColor = view.isRed ? "#ff5264" : "#57a6ff";
  context.shadowBlur = 10;
  context.fillStyle = view.isRed ? "#d83b4c" : "#3287df";
  context.strokeStyle = "#f3f7fb";
  context.lineWidth = 2;
  context.fillRect(-size / 2, -size / 2, size, size);
  context.strokeRect(-size / 2, -size / 2, size, size);
  context.beginPath();
  context.moveTo(size / 2, 0);
  context.lineTo(size / 4, -size / 4);
  context.lineTo(size / 4, size / 4);
  context.closePath();
  context.fillStyle = "#ffffff";
  context.fill();
  context.restore();
}

/**
 * AdvantageKit publishes a Pose2d as 24 bytes: x, y, and heading (radians) as doubles.
 * @param {unknown} value
 * @returns {{ x: number, y: number, heading: number } | null}
 */
function decodePose2d(value) {
  if (Array.isArray(value) && value.length >= 3) {
    const pose = {
      x: Number(value[0]),
      y: Number(value[1]),
      heading: Number(value[2]),
    };
    return Object.values(pose).every(Number.isFinite) ? pose : null;
  }
  if (!(value instanceof Uint8Array) || value.byteLength < 24) return null;
  const bytes = new DataView(value.buffer, value.byteOffset, value.byteLength);
  const pose = {
    x: bytes.getFloat64(0, true),
    y: bytes.getFloat64(8, true),
    heading: bytes.getFloat64(16, true),
  };
  return Object.values(pose).every(Number.isFinite) ? pose : null;
}

customElements.define("custom-dashboard", CustomDashboard);
