/** @typedef {import("./core/dashboard-core.js").DashboardCore} DashboardCore */
/** @typedef {import("./core/dashboard-core.js").DashboardTopic} DashboardTopic */

const POSE_TOPIC = "/AdvantageKit/RealOutputs/Odometry/Robot";
const SPEED_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/Drive/SpeedMetersPerSec";
// Hub boxes from FieldBoundaries.java: [minX, minY, maxX, maxY] for blue, then red.
const HUBS_TOPIC = "/AdvantageKit/RealOutputs/Field/Hubs";
// The auto chooser from RobotContainer ("Auto Choices"), and the robot's mode.
const AUTO_CHOOSER = "/SmartDashboard/Auto Choices";
const ENABLED_TOPIC = "/AdvantageKit/DriverStation/Enabled";
const AUTONOMOUS_TOPIC = "/AdvantageKit/DriverStation/Autonomous";
const ALLIANCE_TOPIC = "/FMSInfo/IsRedAlliance";
const STATION_TOPIC = "/FMSInfo/StationNumber";
const B_COUNT_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/BPressCount";
const B_HELD_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/BHeld";
const ROLLER_VOLTS_TOPIC = "/AdvantageKit/IntakeRoller/AppliedVolts";
const ROLLER_VELOCITY_TOPIC = "/AdvantageKit/IntakeRoller/VelocityRadPerSec";
const ROLLER_SPINNING_RPM = 100;
// Where the intake sits on the robot, in meters (from last year's robot drawing in
// PathPlanner's settings.json on main): centered 0.58 m in front, 0.8 m wide, 0.29 m deep.
const INTAKE = { centerX: 0.58, width: 0.8, depth: 0.29 };
// Where each camera sits on the robot (meters, robot-relative) and how it's turned.
// Copied from robotToCamera* in VisionConstants.java, so keep them in sync.
// All 4 are mounted upside down (roll 180°) and tilted up (negative pitch).
const CAMERAS = [
  { name: "Front right", ntName: "camera_frontright", x: 0.022, y: -0.330, z: 0.429, rollDeg: 180, pitchDeg: -15, yawDeg: -43 },
  { name: "Back right", ntName: "camera_backright", x: -0.302, y: -0.276, z: 0.425, rollDeg: 180, pitchDeg: -15, yawDeg: -150 },
  { name: "Front left", ntName: "camera_frontleft", x: 0.022, y: 0.330, z: 0.429, rollDeg: 180, pitchDeg: -15, yawDeg: 43 },
  { name: "Back left", ntName: "camera_backleft", x: -0.302, y: 0.308, z: 0.254, rollDeg: 180, pitchDeg: -18, yawDeg: 150 },
];
// Sim camera image: 960x720 with an 80° diagonal view (VisionIOPhotonVisionSim.java).
const CAMERA_IMAGE = { width: 960, height: 720, diagonalFovDeg: 80 };
const CAMERA_FOCAL_PX =
  Math.hypot(CAMERA_IMAGE.width, CAMERA_IMAGE.height) / 2 /
  Math.tan(((CAMERA_IMAGE.diagonalFovDeg / 2) * Math.PI) / 180);
const HUB_HEIGHT_METERS = 1.829; // 72 in, game manual (FuelPhysicsSim on main)
// Highlight colors in the camera views. Cyan and magenta, so they don't get mixed up
// with the green and red boxes PhotonVision draws around tags.
const TAG_USED_COLOR = "#22d3ee";
const TAG_REJECTED_COLOR = "#e879f9";
const TAG_SIZE_METERS = 0.1651; // 6.5 in tag; the highlight is drawn a bit bigger
const CAMERA_NAMES = CAMERAS.map((camera) => camera.name);
// Sim cameras are 960x720 with an 80° diagonal view, which is about 68° side to side.
const CAMERA_FOV_DEG = 68;
// How far we trust a tag: maxDistance in VisionConstants.java. The robot sends it
// (MAX_DISTANCE_TOPIC); 7.62 m (25 ft) is only used until the first value arrives.
let cameraRangeMeters = 7.62;
const MAX_DISTANCE_TOPIC = "/AdvantageKit/RealOutputs/Vision/MaxDistanceMeters";
const POSE3D_BYTES = 56; // Pose3d struct: x, y, z + quaternion w, x, y, z
const VISION_HOLD_MS = 500; // keep showing the last sighting this long

// Field size and robot bumper size, in meters (2026 game manual, and last year's
// robot from PathPlanner settings). In sim, the robot sends its own values from
// FieldBoundaries.java and they replace these, so the drawing matches the walls.
const FIELD = { length: 16.541, width: 8.052 };
const FIELD_SIZE_TOPIC = "/AdvantageKit/RealOutputs/Field/Size";
const ROBOT_SIZE_TOPIC = "/AdvantageKit/RealOutputs/Field/RobotSizeMeters";
let robotSizeMeters = 0.9;

/** A fresh, editable dashboard. Add a card here for everything you want to see. */
export class CustomDashboard extends HTMLElement {
  /** @type {DashboardCore | undefined} */
  #core;
  /** @type {boolean} */
  #started = false;
  /** @type {Array<() => void>} */
  #cleanup = [];
  /** What each camera saw most recently. The Vision card fills it in, and the Field card draws it. */
  #cameraStates = CAMERAS.map(() => ({ lastTags: 0, lastPose: "—", lastSeenMs: 0, tagPoints: [] }));
  /** Asks the Field card to redraw. */
  #redrawField = () => {};
  /** Asks the Camera Views card to redraw. */
  #redrawCameras = () => {};

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
          <section class="card" aria-labelledby="auto-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Autonomous</p><h1 id="auto-heading">Auto</h1></div>
            </div>
            <label class="auto-pick">
              <span class="eyebrow">Run this auto</span>
              <select class="auto-select"><option>Waiting for robot…</option></select>
            </label>
            <div class="stat-row">
              <div><p class="eyebrow">Robot will run</p><output class="auto-active">—</output></div>
              <div><p class="eyebrow">Auto timer</p><output class="big-number auto-timer">0.0</output></div>
            </div>
          </section>
          <section class="card" aria-labelledby="intake-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Mechanism</p><h1 id="intake-heading">Intake</h1></div>
            </div>
            <div class="stat-row">
              <div><p class="eyebrow">Volts</p><output class="big-number roller-volts">0.0</output></div>
              <div><p class="eyebrow">Roller RPM</p><output class="big-number roller-rpm">0</output></div>
            </div>
            <output class="status-light roller-spinning">STOPPED</output>
          </section>
          <section class="card" aria-labelledby="vision-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Sensors</p><h1 id="vision-heading">Vision</h1></div>
            </div>
            <table class="vision-table">
              <thead><tr><th>Camera</th><th>Connected</th><th>Tags</th><th>Last pose</th></tr></thead>
              <tbody>${CAMERA_NAMES.map((name, i) => `<tr data-camera="${i}"><td>${name}</td><td><span class="dot"></span></td><td class="tags">0</td><td class="last-pose"><span class="pose-badge">N/A</span><span class="pose-reason"></span></td></tr>`).join("")}</tbody>
            </table>
          </section>
          <section class="card camera-card" aria-labelledby="cameras-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Sensors</p><h1 id="cameras-heading">Camera Views</h1></div>
            </div>
            <div class="camera-grid"><p class="camera-empty">Waiting for camera streams…</p></div>
            <p class="camera-legend">
              <span class="swatch used"></span>Tag used for position
              <span class="swatch rejected"></span>Tag seen, but pose rejected
            </p>
          </section>
          <section class="card" aria-labelledby="copy-nt-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Debug</p><h1 id="copy-nt-heading">Copy NetworkTables</h1></div>
            </div>
            <p class="copy-help">Copies what the robot is publishing right now, so you can paste it to Claude.</p>
            <div class="copy-row">
              <input class="copy-filter" type="text" placeholder="Only names containing… (e.g. Vision)" aria-label="Filter topic names">
              <button class="copy-button" type="button">Copy</button>
            </div>
            <output class="copy-status" aria-live="polite"></output>
          </section>
          <section class="card" aria-labelledby="buttons-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Tutorial</p><h1 id="buttons-heading">B Button</h1></div>
            </div>
            <div class="stat-row">
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
    this.#startIntakeCard(core);
    this.#startVisionCard(core);
    this.#startCameraCard(core);
    this.#startCopyCard(core);
    this.#startAutoCard(core);
  }

  /** Picks the auto to run, shows which one the robot will run, and times auto. */
  #startAutoCard(core) {
    const root = this.shadowRoot;
    const select = root.querySelector(".auto-select");
    const activeText = root.querySelector(".auto-active");
    const timerText = root.querySelector(".auto-timer");
    let shownOptions = "";
    let autoStartMs = 0;
    let lastSeconds = 0;

    const render = () => {
      const options = core.getTopic(`${AUTO_CHOOSER}/options`)?.value;
      const active = core.getTopic(`${AUTO_CHOOSER}/active`)?.value;
      if (Array.isArray(options) && options.join("\n") !== shownOptions) {
        shownOptions = options.join("\n");
        select.replaceChildren(
          ...options.map((name) => Object.assign(document.createElement("option"), { value: name, textContent: name })),
        );
      }
      if (typeof active === "string" && document.activeElement !== select) select.value = active;
      activeText.textContent = typeof active === "string" ? active : "—";

      // The timer counts up while auto is running, and holds the last time after.
      const running =
        core.getTopic(ENABLED_TOPIC)?.value === true && core.getTopic(AUTONOMOUS_TOPIC)?.value === true;
      if (running && !autoStartMs) autoStartMs = performance.now();
      if (!running) autoStartMs = 0;
      if (running) lastSeconds = (performance.now() - autoStartMs) / 1000;
      timerText.textContent = lastSeconds.toFixed(1);
    };

    // Tell the robot which auto we picked (RobotContainer's chooser reads "selected").
    const onChange = () => core.publish(`${AUTO_CHOOSER}/selected`, "string", select.value);
    select.addEventListener("change", onChange);

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (topic.name.startsWith(AUTO_CHOOSER) || topic.name === ENABLED_TOPIC || topic.name === AUTONOMOUS_TOPIC)
        render();
    };
    core.addEventListener("topic", topicListener);
    // Tick while auto runs so the timer counts smoothly.
    const timer = setInterval(() => autoStartMs && render(), 100);
    this.#cleanup.push(() => {
      core.removeEventListener("topic", topicListener);
      select.removeEventListener("change", onChange);
      clearInterval(timer);
    });
    render();
  }

  /** Copies every NetworkTables value (or just the ones matching the filter) as text. */
  #startCopyCard(core) {
    const root = this.shadowRoot;
    const button = root.querySelector(".copy-button");
    const filter = root.querySelector(".copy-filter");
    const status = root.querySelector(".copy-status");

    const onClick = async () => {
      const words = filter.value.trim().toLowerCase();
      const topics = core
        .getTopics()
        .filter((topic) => !words || topic.name.toLowerCase().includes(words))
        .sort((a, b) => a.name.localeCompare(b.name));
      const lines = topics.map((topic) => `${topic.name} = ${formatTopicValue(topic.value)}`);
      const text = [
        `NetworkTables snapshot (${new Date().toLocaleTimeString()}), ${topics.length} values` +
          (words ? `, filter "${words}"` : ""),
        ...lines,
      ].join("\n");
      try {
        await copyText(text);
        status.textContent = `Copied ${topics.length} values! Now paste them to Claude.`;
      } catch {
        status.textContent = "Couldn't copy. Try clicking the page first, then Copy again.";
      }
    };
    button.addEventListener("click", onClick);
    this.#cleanup.push(() => button.removeEventListener("click", onClick));
  }

  /**
   * Shows each camera's video. In sim, PhotonVision draws what each camera sees and
   * publishes the stream's address under /CameraPublisher/<name>-processed/streams.
   */
  #startCameraCard(core) {
    const grid = this.shadowRoot.querySelector(".camera-grid");
    /** @type {Map<string, string>} camera name -> stream URL */
    const shown = new Map();
    /** @type {Array<{ canvas: HTMLCanvasElement, overlay: HTMLCanvasElement, camera: typeof CAMERAS[number], state: object }>} */
    let views = [];
    let frame = 0;

    // Draw our own 3D field behind each video, from that camera's point of view.
    const drawScenes = () => {
      frame = 0;
      const pose = decodePose2d(core.getTopic(POSE_TOPIC)?.value);
      const hubs = toNumbers(core.getTopic(HUBS_TOPIC)?.value);
      for (const view of views) {
        drawCameraScene(view.canvas, view.camera, pose, hubs);
        drawTagHighlights(view.overlay, view.camera, pose, view.state);
      }
    };
    const scheduleScenes = () => {
      if (!frame) frame = requestAnimationFrame(drawScenes);
    };
    this.#redrawCameras = scheduleScenes;

    const render = () => {
      const streams = new Map();
      for (const topic of core.getTopics()) {
        const match = /^\/CameraPublisher\/(.+-processed)\/streams$/.exec(topic.name);
        const url = match && streamUrl(topic.value);
        if (url) streams.set(match[1], url);
      }
      const same =
        streams.size === shown.size && [...streams].every(([name, url]) => shown.get(name) === url);
      if (same) return;
      shown.clear();
      streams.forEach((url, name) => shown.set(name, url));
      views = [];
      if (shown.size === 0) {
        grid.innerHTML = `<p class="camera-empty">Waiting for camera streams…</p>`;
        return;
      }
      grid.replaceChildren(
        ...[...shown].sort(([a], [b]) => a.localeCompare(b)).map(([name, url]) => {
          const figure = document.createElement("figure");
          const stack = document.createElement("div");
          stack.className = "camera-view";
          const canvas = document.createElement("canvas");
          canvas.width = CAMERA_IMAGE.width;
          canvas.height = CAMERA_IMAGE.height;
          const img = document.createElement("img");
          img.src = url;
          img.alt = `${cameraLabel(name)} camera view`;
          // Highlights go on top of the video, so they aren't washed out by it.
          const overlay = document.createElement("canvas");
          overlay.className = "camera-overlay";
          overlay.width = CAMERA_IMAGE.width;
          overlay.height = CAMERA_IMAGE.height;
          stack.append(canvas, img, overlay);
          const caption = document.createElement("figcaption");
          caption.textContent = cameraLabel(name);
          figure.append(stack, caption);
          const index = CAMERAS.findIndex((c) => `${c.ntName}-processed` === name);
          const camera = CAMERAS[index];
          if (camera) views.push({ canvas, overlay, camera, state: this.#cameraStates[index] });
          // Upside-down cameras give upside-down pictures. Turn the picture right side up
          // here on the dashboard only; the robot code still uses the real, flipped image.
          stack.classList.toggle("flipped", Math.abs(camera?.rollDeg ?? 0) === 180);
          return figure;
        }),
      );
      scheduleScenes();
    };

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (topic.name.startsWith("/CameraPublisher/")) render();
      if (topic.name === POSE_TOPIC || topic.name === HUBS_TOPIC) scheduleScenes();
    };
    core.addEventListener("topic", topicListener);
    this.#cleanup.push(() => {
      core.removeEventListener("topic", topicListener);
      cancelAnimationFrame(frame);
      // Stop downloading the video when the dashboard closes.
      grid.querySelectorAll("img").forEach((img) => img.removeAttribute("src"));
    });
    render();
  }


  /** Shows, per camera: connected, how many tags it sees, and whether its last pose was accepted. */
  #startVisionCard(core) {
    const root = this.shadowRoot;
    const cameras = CAMERA_NAMES.map((_, i) => {
      const row = root.querySelector(`tr[data-camera="${i}"]`);
      return {
        connectedTopic: `/AdvantageKit/Vision/Camera${i}/Connected`,
        tagsTopic: `/AdvantageKit/Vision/Camera${i}/TagIds`,
        acceptedTopic: `/AdvantageKit/RealOutputs/Vision/Camera${i}/RobotPosesAccepted`,
        rejectedTopic: `/AdvantageKit/RealOutputs/Vision/Camera${i}/RobotPosesRejected`,
        dot: row.querySelector(".dot"),
        tagsText: row.querySelector(".tags"),
        poseCell: row.querySelector(".last-pose"),
        poseBadge: row.querySelector(".pose-badge"),
        poseReason: row.querySelector(".pose-reason"),
        tagPosesTopic: `/AdvantageKit/RealOutputs/Vision/Camera${i}/TagPoses`,
        lastResultTopic: `/AdvantageKit/RealOutputs/Vision/Camera${i}/LastResult`,
        // Cameras only report on loops with a new frame, so remember the last sighting.
        state: this.#cameraStates[i],
      };
    });

    const render = () => {
      const now = performance.now();
      for (const camera of cameras) {
        const connected = core.getTopic(camera.connectedTopic)?.value === true;
        const tags = core.getTopic(camera.tagsTopic)?.value;
        const accepted = countPoses(core.getTopic(camera.acceptedTopic)?.value);
        const rejected = countPoses(core.getTopic(camera.rejectedTopic)?.value);
        const state = camera.state;
        if (Array.isArray(tags) && tags.length > 0) {
          state.lastTags = tags.length;
          state.lastSeenMs = now;
          state.tagPoints = decodePose3ds(core.getTopic(camera.tagPosesTopic)?.value);
        }
        if (accepted > 0) state.lastPose = "ACCEPTED";
        else if (rejected > 0) state.lastPose = "REJECTED";
        const fresh = now - state.lastSeenMs < VISION_HOLD_MS;
        camera.dot.classList.toggle("on", connected);
        camera.tagsText.textContent = String(fresh ? state.lastTags : 0);
        // A small colored badge (ACCEPTED / REJECTED / N/A), with the robot's reason
        // underneath in small text, like "height off (z = 1.20 m)".
        // If the camera sees no tags right now, there's nothing to judge, so show N/A.
        const lastResult = core.getTopic(camera.lastResultTopic)?.value;
        const reason =
          fresh && typeof lastResult === "string" && lastResult.startsWith("REJECTED: ")
            ? lastResult.slice("REJECTED: ".length)
            : "";
        camera.poseBadge.textContent = fresh ? state.lastPose : "N/A";
        camera.poseReason.textContent = reason;
        camera.poseReason.title = reason;
        camera.poseCell.dataset.state = fresh ? state.lastPose : "NA";
      }
      this.#redrawField();
      this.#redrawCameras();
    };

    const topicNames = cameras.flatMap((c) => [
      c.connectedTopic,
      c.tagsTopic,
      c.acceptedTopic,
      c.rejectedTopic,
      c.tagPosesTopic,
      c.lastResultTopic,
    ]);
    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (topicNames.includes(topic.name)) render();
    };
    core.addEventListener("topic", topicListener);
    // Also tick a few times a second so the tag count drops to 0 when a camera stops seeing tags.
    const timer = setInterval(render, 250);
    this.#cleanup.push(() => {
      core.removeEventListener("topic", topicListener);
      clearInterval(timer);
    });
    render();
  }

  /** Shows the intake roller's volts, RPM, and whether it's spinning. */
  #startIntakeCard(core) {
    const root = this.shadowRoot;
    const voltsText = root.querySelector(".roller-volts");
    const rpmText = root.querySelector(".roller-rpm");
    const spinningLight = root.querySelector(".roller-spinning");

    const render = () => {
      const volts = Number(core.getTopic(ROLLER_VOLTS_TOPIC)?.value) || 0;
      const { rpm, state } = readRoller(core);
      voltsText.textContent = volts.toFixed(1);
      rpmText.textContent = String(Math.round(rpm));
      spinningLight.textContent = state;
      spinningLight.classList.toggle("on", state !== "STOPPED");
    };

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if ([ROLLER_VOLTS_TOPIC, ROLLER_VELOCITY_TOPIC].includes(topic.name)) render();
    };
    core.addEventListener("topic", topicListener);
    this.#cleanup.push(() => core.removeEventListener("topic", topicListener));
    render();
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
      const fieldSize = toNumbers(core.getTopic(FIELD_SIZE_TOPIC)?.value);
      if (fieldSize.length >= 2 && fieldSize.every((n) => n > 0)) {
        [FIELD.length, FIELD.width] = fieldSize;
      }
      const robotSize = Number(core.getTopic(ROBOT_SIZE_TOPIC)?.value);
      if (robotSize > 0) robotSizeMeters = robotSize;
      const maxDistance = Number(core.getTopic(MAX_DISTANCE_TOPIC)?.value);
      if (maxDistance > 0) cameraRangeMeters = maxDistance;
      const speed = Number(core.getTopic(SPEED_TOPIC)?.value);
      drawField(canvas, view.getBoundingClientRect(), {
        isRed,
        station,
        pose,
        hubs: toNumbers(core.getTopic(HUBS_TOPIC)?.value),
        cameraStates: this.#cameraStates,
        intakeState: readRoller(core).state,
      });
      const speedText = Number.isFinite(speed) ? ` · ${speed.toFixed(2)} m/s` : "";
      poseText.textContent = pose
        ? `x ${pose.x.toFixed(2)} · y ${pose.y.toFixed(2)} · ${Math.round((pose.heading * 180) / Math.PI)}°${speedText}`
        : "Waiting for pose…";
    };
    // Pose updates arrive very often, so draw at most once per screen refresh.
    const scheduleRender = () => {
      if (!frame) frame = requestAnimationFrame(render);
    };
    this.#redrawField = scheduleRender;

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if ([POSE_TOPIC, ALLIANCE_TOPIC, STATION_TOPIC, SPEED_TOPIC, HUBS_TOPIC, FIELD_SIZE_TOPIC, ROBOT_SIZE_TOPIC, ROLLER_VELOCITY_TOPIC, MAX_DISTANCE_TOPIC].includes(topic.name))
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
function drawField(canvas, bounds, { isRed, station, pose, hubs, cameraStates, intakeState }) {
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

  // Field walls: a thick border the robot can't drive through in sim.
  context.strokeStyle = "#d6dde8";
  context.lineWidth = 4;
  context.strokeRect(left, top, drawnWidth, drawnHeight);
  context.lineWidth = 1;

  drawHubs(context, hubs, { isRed, left, top, scale });

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

  if (pose) drawCameraViews(context, pose, cameraStates, { isRed, left, top, scale });
  if (pose) drawRobot(context, pose, intakeState, { isRed, left, top, scale });
}

/** Draws each hub (goal) as a square: blue hub first, then red. */
function drawHubs(context, hubs, view) {
  for (let i = 0; i + 3 < hubs.length; i += 4) {
    const [minX, minY, maxX, maxY] = hubs.slice(i, i + 4);
    const a = fieldToScreen(minX, minY, view);
    const b = fieldToScreen(maxX, maxY, view);
    const isBlueHub = i === 0;
    context.fillStyle = isBlueHub ? "#2879cf66" : "#b52c3c66";
    context.strokeStyle = isBlueHub ? "#63b2ff" : "#ff6877";
    context.lineWidth = 2;
    const x = Math.min(a.x, b.x);
    const y = Math.min(a.y, b.y);
    const width = Math.abs(a.x - b.x);
    const height = Math.abs(a.y - b.y);
    context.fillRect(x, y, width, height);
    context.strokeRect(x, y, width, height);
    context.fillStyle = "#ffffff";
    context.font = "700 9px system-ui, sans-serif";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText("HUB", x + width / 2, y + height / 2);
  }
  context.lineWidth = 1;
}

/**
 * Draws what each camera can see as a cone: green if its last pose was accepted,
 * red if rejected, and faint gray when it doesn't see any tags. Lines show which
 * tags it's looking at.
 */
function drawCameraViews(context, pose, cameraStates, view) {
  const now = performance.now();
  const cos = Math.cos(pose.heading);
  const sin = Math.sin(pose.heading);
  const halfFov = ((CAMERA_FOV_DEG / 2) * Math.PI) / 180;

  CAMERAS.forEach((camera, i) => {
    const state = cameraStates?.[i];
    const seeing = state && now - state.lastSeenMs < VISION_HOLD_MS;
    const color = !seeing
      ? "#93a0b4"
      : state.lastPose === "REJECTED"
        ? "#f87171"
        : "#4ade80";

    // Camera position on the field: rotate its robot-relative spot by the robot's heading.
    const camX = pose.x + camera.x * cos - camera.y * sin;
    const camY = pose.y + camera.x * sin + camera.y * cos;
    const facing = pose.heading + (camera.yawDeg * Math.PI) / 180;
    const apex = fieldToScreen(camX, camY, view);

    // The cone: from the camera out to its max range, across its field of view.
    context.beginPath();
    context.moveTo(apex.x, apex.y);
    for (let step = 0; step <= 12; step++) {
      const angle = facing - halfFov + (2 * halfFov * step) / 12;
      const edge = fieldToScreen(
        camX + cameraRangeMeters * Math.cos(angle),
        camY + cameraRangeMeters * Math.sin(angle),
        view,
      );
      context.lineTo(edge.x, edge.y);
    }
    context.closePath();
    context.fillStyle = color + (seeing ? "33" : "14");
    context.strokeStyle = color + (seeing ? "aa" : "40");
    context.lineWidth = 1;
    context.fill();
    context.stroke();

    // Rays from the camera to each tag it sees. Tags past our max distance are seen
    // but too far to trust, so they get a dashed yellow ray instead.
    if (!seeing) return;
    context.lineWidth = 1.5;
    for (const tag of state.tagPoints) {
      const tooFar = Math.hypot(tag.x - camX, tag.y - camY) > cameraRangeMeters;
      const rayColor = tooFar ? "#facc15" : color;
      const end = fieldToScreen(tag.x, tag.y, view);
      context.strokeStyle = rayColor;
      context.fillStyle = rayColor;
      context.setLineDash(tooFar ? [4, 4] : []);
      context.beginPath();
      context.moveTo(apex.x, apex.y);
      context.lineTo(end.x, end.y);
      context.stroke();
      context.setLineDash([]);
      context.beginPath();
      context.arc(end.x, end.y, 3, 0, Math.PI * 2);
      context.fill();
    }
  });
}

/** Converts field meters to screen pixels, flipping the view for the red alliance. */
function fieldToScreen(fieldX, fieldY, { isRed, left, top, scale }) {
  return {
    x: left + (isRed ? fieldY : FIELD.width - fieldY) * scale,
    y: top + (isRed ? fieldX : FIELD.length - fieldX) * scale,
  };
}

/** Draws the robot as a square with an arrow pointing where its front faces, plus its intake. */
function drawRobot(context, pose, intakeState, view) {
  const center = fieldToScreen(pose.x, pose.y, view);
  const front = fieldToScreen(
    pose.x + Math.cos(pose.heading),
    pose.y + Math.sin(pose.heading),
    view,
  );
  const size = Math.max(10, robotSizeMeters * view.scale);
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
  context.shadowBlur = 0;
  drawIntake(context, intakeState, view.scale);
  context.restore();
}

/**
 * Draws the intake as a block on the robot's front (already rotated so +x is forward).
 * Gray when stopped, green with arrows pointing in when intaking, orange pointing out
 * when outtaking.
 */
function drawIntake(context, state, scale) {
  const depth = INTAKE.depth * scale;
  const width = INTAKE.width * scale;
  const x = INTAKE.centerX * scale - depth / 2;
  const y = -width / 2;
  const color = state === "INTAKING" ? "#4ade80" : state === "OUTTAKING" ? "#fb923c" : "#93a0b4";
  context.fillStyle = color + (state === "STOPPED" ? "55" : "cc");
  context.strokeStyle = color;
  context.lineWidth = 1.5;
  context.fillRect(x, y, depth, width);
  context.strokeRect(x, y, depth, width);
  if (state === "STOPPED") return;

  // Three chevrons across the intake: ">" pointing back into the robot when intaking,
  // "<"-style pointing out the front when outtaking.
  const pointIn = state === "INTAKING";
  const tip = pointIn ? x + depth * 0.25 : x + depth * 0.75;
  const tail = pointIn ? x + depth * 0.75 : x + depth * 0.25;
  context.strokeStyle = "#0c1017";
  context.lineWidth = 2;
  for (const offset of [-0.3, 0, 0.3]) {
    const midY = offset * width;
    const half = Math.min(depth * 0.35, width * 0.1);
    context.beginPath();
    context.moveTo(tail, midY - half);
    context.lineTo(tip, midY);
    context.lineTo(tail, midY + half);
    context.stroke();
  }
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

/**
 * Draws the field in 3D from one camera's point of view: a floor grid, the field edge,
 * the center line, and the hubs as tall boxes. It uses the same camera math as the
 * PhotonVision sim, so it lines up with the tags in the video drawn on top.
 */
function drawCameraScene(canvas, camera, pose, hubs) {
  const context = canvas.getContext("2d");
  context.fillStyle = "#0c1017";
  context.fillRect(0, 0, canvas.width, canvas.height);
  if (!pose) return;

  const view = cameraView(camera, pose, canvas);
  const line = (a, b, color, width = 2) => {
    const segment = view.segment(a, b);
    if (!segment) return;
    context.strokeStyle = color;
    context.lineWidth = width;
    context.beginPath();
    context.moveTo(segment[0], segment[1]);
    context.lineTo(segment[2], segment[3]);
    context.stroke();
  };

  // Floor grid every meter, so you can tell how far away things are.
  for (let x = 1; x < FIELD.length; x++) line([x, 0, 0], [x, FIELD.width, 0], "#2a3446", 1);
  for (let y = 1; y < FIELD.width; y++) line([0, y, 0], [FIELD.length, y, 0], "#2a3446", 1);
  // Center line and field edge.
  line([FIELD.length / 2, 0, 0], [FIELD.length / 2, FIELD.width, 0], "#7c8798", 2);
  const corners = [[0, 0], [FIELD.length, 0], [FIELD.length, FIELD.width], [0, FIELD.width]];
  corners.forEach(([x, y], i) => {
    const [nx, ny] = corners[(i + 1) % 4];
    line([x, y, 0], [nx, ny, 0], "#d6dde8", 3);
  });

  // Hubs as boxes: blue first, then red.
  for (let i = 0; i + 3 < hubs.length; i += 4) {
    const [minX, minY, maxX, maxY] = hubs.slice(i, i + 4);
    const color = i === 0 ? "#63b2ff" : "#ff6877";
    const base = [[minX, minY], [maxX, minY], [maxX, maxY], [minX, maxY]];
    base.forEach(([x, y], k) => {
      const [nx, ny] = base[(k + 1) % 4];
      line([x, y, 0], [nx, ny, 0], color, 3);
      line([x, y, HUB_HEIGHT_METERS], [nx, ny, HUB_HEIGHT_METERS], color, 3);
      line([x, y, 0], [x, y, HUB_HEIGHT_METERS], color, 3);
    });
  }
}

/**
 * The camera math: where a camera is and which way it's turned, turned into a
 * function that finds where a field point lands in its picture.
 */
function cameraView(camera, pose, canvas) {
  const toRad = (deg) => (deg * Math.PI) / 180;
  const robotTurn = rotationZ(pose.heading);
  const mount = multiply(
    rotationZ(toRad(camera.yawDeg)),
    multiply(rotationY(toRad(camera.pitchDeg)), rotationX(toRad(camera.rollDeg))),
  );
  const turn = multiply(robotTurn, mount);
  const offset = apply(robotTurn, [camera.x, camera.y, camera.z]);
  const eye = [pose.x + offset[0], pose.y + offset[1], offset[2]];

  // Field point -> camera point (x forward, y left, z up) -> pixel.
  const toCamera = (p) => {
    const d = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]];
    return [0, 1, 2].map((col) => turn[0][col] * d[0] + turn[1][col] * d[1] + turn[2][col] * d[2]);
  };
  const toPixel = ([x, y, z]) => [
    canvas.width / 2 - (CAMERA_FOCAL_PX * y) / x,
    canvas.height / 2 - (CAMERA_FOCAL_PX * z) / x,
  ];
  const near = 0.05; // skip what's behind the camera

  return {
    /** A field line as [ax, ay, bx, by] in pixels, or null if it's behind the camera. */
    segment(a, b) {
      let ca = toCamera(a);
      let cb = toCamera(b);
      if (ca[0] < near && cb[0] < near) return null;
      if (ca[0] < near || cb[0] < near) {
        const t = (near - ca[0]) / (cb[0] - ca[0]);
        const cut = ca.map((v, i) => v + (cb[i] - v) * t);
        if (ca[0] < near) ca = cut;
        else cb = cut;
      }
      return [...toPixel(ca), ...toPixel(cb)];
    },
    /** A field point in pixels, or null if it's behind the camera. */
    point(p) {
      const c = toCamera(p);
      return c[0] < near ? null : toPixel(c);
    },
  };
}

/** Outlines each tag this camera sees: cyan if its pose was used, magenta if rejected. */
function drawTagHighlights(canvas, camera, pose, state) {
  const context = canvas.getContext("2d");
  context.clearRect(0, 0, canvas.width, canvas.height);
  const seeing = state && performance.now() - state.lastSeenMs < VISION_HOLD_MS;
  if (!pose || !seeing) return;

  const view = cameraView(camera, pose, canvas);
  const color = state.lastPose === "REJECTED" ? TAG_REJECTED_COLOR : TAG_USED_COLOR;
  const half = (TAG_SIZE_METERS / 2) * 1.6; // a bit bigger than the tag itself
  context.strokeStyle = color;
  context.fillStyle = color + "26";
  context.lineWidth = 5;
  context.lineJoin = "round";
  for (const tag of state.tagPoints) {
    // A tag faces out along its own x axis, so its corners are at y/z = ±half.
    const rotate = quaternionMatrix(tag);
    const corners = [[0, -half, -half], [0, half, -half], [0, half, half], [0, -half, half]].map((c) => {
      const r = apply(rotate, c);
      return view.point([tag.x + r[0], tag.y + r[1], tag.z + r[2]]);
    });
    if (corners.some((c) => !c)) continue;
    context.beginPath();
    corners.forEach(([x, y], i) => (i === 0 ? context.moveTo(x, y) : context.lineTo(x, y)));
    context.closePath();
    context.fill();
    context.stroke();
  }
}

/** Rotation matrix from a quaternion (w, x, y, z). */
function quaternionMatrix({ qw: w, qx: x, qy: y, qz: z }) {
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
}

/** 3x3 rotation matrices (WPILib: x forward, y left, z up). */
function rotationX(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [[1, 0, 0], [0, c, -s], [0, s, c]];
}
function rotationY(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [[c, 0, s], [0, 1, 0], [-s, 0, c]];
}
function rotationZ(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [[c, -s, 0], [s, c, 0], [0, 0, 1]];
}
function multiply(a, b) {
  return a.map((row) => [0, 1, 2].map((col) => row[0] * b[0][col] + row[1] * b[1][col] + row[2] * b[2][col]));
}
function apply(m, v) {
  return m.map((row) => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
}

/**
 * Turns a CameraPublisher "streams" list (like "mjpg:http://10.0.0.5:1181/?action=stream")
 * into a URL this browser can open, using the same computer the dashboard came from.
 */
function streamUrl(value) {
  const first = Array.isArray(value) ? value.find((entry) => String(entry).startsWith("mjpg:")) : null;
  if (!first) return null;
  try {
    const url = new URL(String(first).slice("mjpg:".length));
    if (location.hostname) url.hostname = location.hostname;
    return url.toString();
  } catch {
    return null;
  }
}

/** "camera_frontright-processed" -> "frontright" */
function cameraLabel(name) {
  return name.replace(/^camera_/, "").replace(/-processed$/, "");
}

/**
 * Turns one NetworkTables value into readable text. Structs (like poses) arrive as raw
 * bytes, so those are shown as the numbers inside them (8 bytes per number).
 */
function formatTopicValue(value) {
  const round = (n) => (Number.isInteger(n) ? String(n) : Number(n).toFixed(4));
  if (value instanceof Uint8Array) {
    if (value.byteLength === 0) return "[] (empty)";
    if (value.byteLength % 8 !== 0) return `bytes(${value.byteLength})`;
    const view = new DataView(value.buffer, value.byteOffset, value.byteLength);
    const count = value.byteLength / 8;
    const numbers = Array.from({ length: Math.min(count, 28) }, (_, i) =>
      round(view.getFloat64(i * 8, true)),
    );
    return `struct ${value.byteLength} bytes: [${numbers.join(", ")}${count > 28 ? ", …" : ""}]`;
  }
  if (Array.isArray(value) || ArrayBuffer.isView(value)) {
    const items = Array.from(value, (v) => (typeof v === "number" ? round(v) : JSON.stringify(v)));
    return `[${items.slice(0, 40).join(", ")}${items.length > 40 ? ", …" : ""}]`;
  }
  if (typeof value === "number") return round(value);
  if (typeof value === "string") return JSON.stringify(value);
  return String(value);
}

/** Puts text on the clipboard, with an older fallback for browsers that need it. */
async function copyText(text) {
  if (navigator.clipboard?.writeText) {
    await navigator.clipboard.writeText(text);
    return;
  }
  const box = document.createElement("textarea");
  box.value = text;
  document.body.append(box);
  box.select();
  const ok = document.execCommand("copy");
  box.remove();
  if (!ok) throw new Error("copy failed");
}

/** Reads the intake roller: its RPM, and whether it's STOPPED, INTAKING, or OUTTAKING. */
function readRoller(core) {
  const radPerSec = Number(core.getTopic(ROLLER_VELOCITY_TOPIC)?.value) || 0;
  const rpm = (radPerSec * 60) / (2 * Math.PI);
  const state = Math.abs(rpm) <= ROLLER_SPINNING_RPM ? "STOPPED" : rpm > 0 ? "INTAKING" : "OUTTAKING";
  return { rpm, state };
}

/** Turns a logged number list (plain or typed array) into plain numbers. */
function toNumbers(value) {
  if (!Array.isArray(value) && !ArrayBuffer.isView(value)) return [];
  return Array.from(value, Number).filter(Number.isFinite);
}

/** Reads each Pose3d in a logged Pose3d[] (raw bytes): position plus quaternion. */
function decodePose3ds(value) {
  if (!(value instanceof Uint8Array)) return [];
  const bytes = new DataView(value.buffer, value.byteOffset, value.byteLength);
  const poses = [];
  for (let offset = 0; offset + POSE3D_BYTES <= value.byteLength; offset += POSE3D_BYTES) {
    const [x, y, z, qw, qx, qy, qz] = [0, 1, 2, 3, 4, 5, 6].map((i) =>
      bytes.getFloat64(offset + i * 8, true),
    );
    poses.push({ x, y, z, qw, qx, qy, qz });
  }
  return poses;
}

/** How many Pose3d structs are in a logged Pose3d[] (raw bytes). */
function countPoses(value) {
  if (Array.isArray(value)) return value.length;
  return value instanceof Uint8Array ? Math.floor(value.byteLength / POSE3D_BYTES) : 0;
}

customElements.define("custom-dashboard", CustomDashboard);
