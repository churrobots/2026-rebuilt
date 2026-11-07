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
const ROLLER_VOLTS_TOPIC = "/AdvantageKit/IntakeRoller/AppliedVolts";
const ROLLER_VELOCITY_TOPIC = "/AdvantageKit/IntakeRoller/VelocityRadPerSec";
const ROLLER_SPINNING_RPM = 350;
// Named commands for PathPlanner autos, from configureNamedCommands() in RobotContainer.
const NAMED_COMMANDS_PREFIX = "/AdvantageKit/RealOutputs/Auto/NamedCommands/";
const NAMED_COMMANDS_TOPIC = `${NAMED_COMMANDS_PREFIX}Names`;
// Intake arm, from IntakeArm.periodic(). Angles in radians: 0 = stowed, bigger = lower.
const ARM_MEASURED_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/Arm/MeasuredRad";
const ARM_TARGET_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/Arm/TargetRad";
const ARM_AT_TARGET_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/Arm/AtTarget";
const ARM_LEVEL_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/Arm/LevelRad";
// Spindexer inputs from Spindexer.periodic() (Logger.processInputs("Spindexer", ...)).
const SPINDEXER_POSITION_TOPIC = "/AdvantageKit/Spindexer/PositionRad";
const SPINDEXER_VELOCITY_TOPIC = "/AdvantageKit/Spindexer/VelocityRadPerSec";
const SPINDEXER_VOLTS_TOPIC = "/AdvantageKit/Spindexer/AppliedVolts";
const SPINDEXER_SPINNING_RPM = 2; // it's geared 125:1, so even "fast" is slow
// Shooter + feeder, from Shooter.periodic() and Feeder.periodic().
const SHOOTER_TARGET_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/Shooter/TargetRPM";
const SHOOTER_MEASURED_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/Shooter/MeasuredRPM";
const SHOOTER_READY_TOPIC = "/AdvantageKit/RealOutputs/Tutorial/Shooter/Ready";
const FEEDER_VELOCITY_TOPIC = "/AdvantageKit/Feeder/VelocityRadPerSec";
const SHOOTER_GAUGE_MAX_RPM = 6000; // a Kraken tops out around here
// Live shot trim (RPM), read by RobotContainer as a LoggedNetworkNumber. Negative = slower = shorter.
const SHOT_TRIM_TOPIC = "/Tuning/ShotTrimRpm";
// NetworkTables doesn't send our own changes back to us, so to know what the robot is really
// using, read the value it logs as an AdvantageKit input (DashboardNumber in RobotContainer).
const SHOT_TRIM_ROBOT_TOPIC = "/AdvantageKit/Tuning/ShotTrimRpm/Value";
const SHOT_TRIM_STEP_RPM = 50;
const SHOT_TRIM_LIMIT_RPM = 600; // the slider goes from -600 to +600
// Past this much trim (either way), the table is probably wrong, not just a little off:
// the slider shows a yellow "caution" zone and suggests re-measuring the table instead.
const SHOT_TRIM_CAUTION_RPM = 300;
// Real wheels spin way too fast to see, so the drawing spins them slowed down.
const SHOOTER_DRAW_SLOWDOWN = 0.002; // drawn rad/sec per RPM
// Auto-aim (DriveCommands.autoAim): the Field card draws the aim line from these.
const AIM_ACTIVE_TOPIC = "/AdvantageKit/RealOutputs/Aim/Active";
const AIM_LOCKED_TOPIC = "/AdvantageKit/RealOutputs/Aim/Locked";
// Where the intake sits on the robot, in meters (from last year's robot drawing in
// PathPlanner's settings.json on main): centered 0.58 m in front, 0.8 m wide, 0.29 m deep.
const INTAKE = { centerX: 0.58, width: 0.8, depth: 0.29 };
// Where each camera sits on the robot (meters, robot-relative) and how it's turned.
// Copied from robotToCamera* in VisionConstants.java, so keep them in sync.
// All 4 are mounted upside down (roll 180°) and tilted up (negative pitch).
const CAMERAS = [
  { name: "Front right", ntName: "front right", x: -0.003, y: -0.330, z: 0.365, rollDeg: 180, pitchDeg: -15, yawDeg: -30 },
  { name: "Back right", ntName: "back right", x: -0.302, y: -0.276, z: 0.362, rollDeg: 180, pitchDeg: -7.5, yawDeg: -142 },
  { name: "Front left", ntName: "front left", x: -0.003, y: 0.330, z: 0.365, rollDeg: 180, pitchDeg: -15, yawDeg: 30 },
  { name: "Back left", ntName: "back left", x: -0.302, y: 0.276, z: 0.362, rollDeg: 180, pitchDeg: -7.5, yawDeg: 142 },
];
// The cards shrink to fit the window. Below this they'd be too small to read, so the page
// scrolls instead.
const MIN_FIT_SCALE = 0.4;
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

// HardwareMonitor publishes true when a device is healthy, false when it's faulted.
const HARDWARE_TABLE = "/SmartDashboard/HardwareMonitor/FaultStatus";
// Swerve corners, in the order they sit on the robot with the front at the top.
const ROBOT_CORNERS = [
  {
    name: "FL",
    devices: [
      ["FrontLeftDrive", "Drive"],
      ["FrontLeftTurn", "Turn"],
      ["front left", "Camera"],
    ],
  },
  {
    name: "FR",
    devices: [
      ["FrontRightDrive", "Drive"],
      ["FrontRightTurn", "Turn"],
      ["front right", "Camera"],
    ],
  },
  {
    name: "BL",
    devices: [
      ["BackLeftDrive", "Drive"],
      ["BackLeftTurn", "Turn"],
      ["back left", "Camera"],
    ],
  },
  {
    name: "BR",
    devices: [
      ["BackRightDrive", "Drive"],
      ["BackRightTurn", "Turn"],
      ["back right", "Camera"],
    ],
  },
];
// Drawn in the middle of the robot, where the gyro usually sits.
const GYRO = ["Pigeon2 gyro", "Gyro"];
const OTHER_HARDWARE = [
  ["shooterMotor", "Shooter"],
  ["feederMotor", "Feeder"],
  ["spindexerMotor", "Spindexer"],
  ["intakeRollerMotor", "Intake roller"],
  ["intakeArmMotor", "Intake arm"],
];

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
            <p class="eyebrow named-command-heading">Auto commands</p>
            <ul class="named-command-list"><li class="named-command-empty">Waiting for robot…</li></ul>
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
          <section class="card" aria-labelledby="hardware-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Hardware</p><h1 id="hardware-heading">Health</h1></div>
              <output class="hardware-summary" aria-live="polite"></output>
            </div>
            <div class="hardware-groups"></div>
          </section>
          <section class="card" aria-labelledby="shooter-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Mechanism</p><h1 id="shooter-heading">Shooter</h1></div>
              <output class="status-light shooter-ready">NOT READY</output>
            </div>
            <canvas class="shooter-view" width="360" height="220" aria-label="Feeder and flywheel seen from the side"></canvas>
            <div class="shooter-stats">
              <div><p class="eyebrow">Flywheel RPM</p><output class="big-number shooter-rpm">0</output></div>
              <div><p class="eyebrow">Target RPM</p><output class="big-number shooter-target">0</output></div>
              <div><p class="eyebrow">Feeder RPM</p><output class="big-number feeder-rpm">0</output></div>
            </div>
            <div class="trim">
              <div class="trim-head">
                <p class="eyebrow">Shot trim</p>
                <output class="trim-value">—</output>
              </div>
              <div class="trim-slider">
                <div class="trim-caution trim-caution-low"></div>
                <div class="trim-caution trim-caution-high"></div>
                <div class="trim-zero"></div>
                <div class="trim-default" title="Value in the code"></div>
                <input class="trim-range" type="range" aria-label="Shot trim in RPM">
              </div>
              <div class="trim-scale"><span class="trim-min"></span><span>0</span><span class="trim-max"></span></div>
              <p class="trim-status"></p>
              <p class="trim-warning" hidden>That's a big trim! The RPM table is probably off. Think about re-measuring it.</p>
              <div class="trim-row">
                <button class="trim-button" type="button" data-step="-1">Slower −50</button>
                <button class="trim-button" type="button" data-step="1">Faster +50</button>
                <button class="trim-reset" type="button">Reset</button>
              </div>
              <p class="trim-help">Added to every shot's RPM. Shots landing long? Tap Slower. Short? Tap Faster. Resets when the robot restarts, so copy a good number into <code>ShooterConstants.shotRpmTrim</code>.</p>
            </div>
            <p class="shooter-note">Wheels in the drawing spin slowed down so you can see them.</p>
          </section>
          <section class="card" aria-labelledby="spindexer-heading">
            <div class="card-heading">
              <div><p class="eyebrow">Mechanism</p><h1 id="spindexer-heading">Spindexer</h1></div>
            </div>
            <div class="spindexer-body">
              <canvas class="spindexer-disc" width="320" height="320" aria-label="Spindexer seen from above"></canvas>
              <div class="spindexer-stats">
                <div><p class="eyebrow">Volts</p><output class="big-number spindexer-volts">0.0</output></div>
                <div><p class="eyebrow">Spindexer RPM</p><output class="big-number spindexer-rpm">0.0</output></div>
                <output class="status-light spindexer-state">STOPPED</output>
              </div>
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
            <div class="arm-body">
              <canvas class="arm-view" width="320" height="220" aria-label="Intake arm seen from the side"></canvas>
              <div class="arm-stats">
                <div><p class="eyebrow">Arm (rad)</p><output class="big-number arm-measured">—</output></div>
                <div><p class="eyebrow">Target (rad)</p><output class="big-number arm-target">—</output></div>
                <output class="status-light arm-at-target">NO TARGET</output>
              </div>
            </div>
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
    this.#startFitToWindow();
    this.#startFieldCard(core);
    this.#startIntakeCard(core);
    this.#startSpindexerCard(core);
    this.#startShooterCard(core);
    this.#trackVision(core);
    this.#startCopyCard(core);
    this.#startAutoCard(core);
    this.#startHardwareCard(core);
    this.#startNamedCommandsCard(core);
  }

  /**
   * Shrinks the whole card layout just enough to fit the window, with nothing cut off and
   * no scrolling. The layout is given a wider "virtual" width and then scaled down, so the
   * cards can rearrange into more columns instead of just getting tiny.
   */
  #startFitToWindow() {
    const main = this.shadowRoot.querySelector("main");
    let frame = 0;

    /** Lays the cards out for one scale and reports whether they fit the window. */
    const fitsAt = (scale, width, height) => {
      main.style.width = `${width / scale}px`;
      return main.offsetHeight * scale <= height;
    };
    const fit = () => {
      frame = 0;
      const width = this.clientWidth;
      const height = this.clientHeight;
      if (!width || !height) return;
      let scale = 1;
      if (!fitsAt(1, width, height)) {
        // Binary search for the biggest scale that fits.
        let low = MIN_FIT_SCALE;
        let high = 1;
        for (let i = 0; i < 10; i++) {
          const middle = (low + high) / 2;
          if (fitsAt(middle, width, height)) low = middle;
          else high = middle;
        }
        scale = low;
      }
      const fits = fitsAt(scale, width, height);
      main.style.transform = scale < 1 ? `scale(${scale})` : "";
      // Only if it can't fit even at the smallest readable size (e.g. on a phone).
      this.style.overflowY = fits ? "hidden" : "auto";
    };
    // Refit when the window resizes or a card's contents change size (e.g. camera streams
    // showing up). Refitting gives the same answer for the same content, so it settles.
    const schedule = () => {
      if (!frame) frame = requestAnimationFrame(fit);
    };
    const resizeObserver = new ResizeObserver(schedule);
    resizeObserver.observe(this);
    main.querySelectorAll(".card").forEach((card) => resizeObserver.observe(card));
    this.#cleanup.push(() => {
      resizeObserver.disconnect();
      cancelAnimationFrame(frame);
      main.style.width = "";
      main.style.transform = "";
    });
    schedule();
  }

  /** Shows a light for every device HardwareMonitor checks: green is OK, red is faulted. */
  #startHardwareCard(core) {
    const root = this.shadowRoot;
    const summary = root.querySelector(".hardware-summary");
    const groups = root.querySelector(".hardware-groups");
    /** @type {Map<string, HTMLElement>} */
    const lights = new Map();

    /** @param {string} device @param {string} label */
    const makeLight = (device, label) => {
      const light = document.createElement("span");
      light.className = "light";
      light.textContent = label;
      light.title = device;
      lights.set(`${HARDWARE_TABLE}/${device}`, light);
      return light;
    };

    // The drivetrain as a top-down robot: front at the top, one box per swerve corner.
    const robot = document.createElement("div");
    robot.className = "robot";
    const front = document.createElement("div");
    front.className = "robot-front";
    front.textContent = "▲ Front";
    const corners = document.createElement("div");
    corners.className = "corners";
    for (const corner of ROBOT_CORNERS) {
      const box = document.createElement("div");
      box.className = "corner";
      const name = document.createElement("b");
      name.textContent = corner.name;
      box.append(name, ...corner.devices.map(([device, label]) => makeLight(device, label)));
      corners.append(box);
    }
    const gyro = makeLight(...GYRO);
    gyro.classList.add("gyro");
    corners.append(gyro);
    robot.append(front, corners);

    const otherHeading = document.createElement("h2");
    otherHeading.textContent = "Mechanisms";
    const other = document.createElement("div");
    other.className = "hardware-grid";
    other.append(...OTHER_HARDWARE.map(([device, label]) => makeLight(device, label)));
    groups.append(robot, otherHeading, other);

    const render = () => {
      let faults = 0;
      let missing = 0;
      for (const [topicName, light] of lights) {
        const value = core.getTopic(topicName)?.value;
        light.dataset.state =
          value === true ? "ok" : value === false ? "fault" : "missing";
        if (value === false) faults++;
        if (typeof value !== "boolean") missing++;
      }
      summary.dataset.state = faults ? "fault" : missing ? "missing" : "ok";
      summary.textContent = faults
        ? `${faults} faulted`
        : missing === lights.size
          ? "No data"
          : missing
            ? `${missing} not reporting`
            : "All OK";
    };

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (lights.has(topic.name)) render();
    };
    // Values are cleared when switching robots, so redraw on connection changes too.
    core.addEventListener("topic", topicListener);
    core.addEventListener("connection", render);
    this.#cleanup.push(() => {
      core.removeEventListener("topic", topicListener);
      core.removeEventListener("connection", render);
    });
    render();
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
   * Keeps track of what each camera saw most recently (no card of its own anymore), so the
   * Field card can draw each camera's view cone and the rays to the tags it sees.
   */
  #trackVision(core) {
    const cameras = CAMERAS.map((_, i) => {
      return {
        tagsTopic: `/AdvantageKit/Vision/Camera${i}/TagIds`,
        acceptedTopic: `/AdvantageKit/RealOutputs/Vision/Camera${i}/RobotPosesAccepted`,
        rejectedTopic: `/AdvantageKit/RealOutputs/Vision/Camera${i}/RobotPosesRejected`,
        tagPosesTopic: `/AdvantageKit/RealOutputs/Vision/Camera${i}/TagPoses`,
        // Cameras only report on loops with a new frame, so remember the last sighting.
        state: this.#cameraStates[i],
      };
    });

    const render = () => {
      const now = performance.now();
      for (const camera of cameras) {
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
      }
      this.#redrawField();
    };

    const topicNames = cameras.flatMap((c) => [c.tagsTopic, c.acceptedTopic, c.rejectedTopic, c.tagPosesTopic]);
    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (topicNames.includes(topic.name)) render();
    };
    core.addEventListener("topic", topicListener);
    // Also tick a few times a second so a cone goes gray when its camera stops seeing tags.
    const timer = setInterval(render, 250);
    this.#cleanup.push(() => {
      core.removeEventListener("topic", topicListener);
      clearInterval(timer);
    });
    render();
  }

  /** Lists the auto's named commands and lights up each one while it runs. */
  #startNamedCommandsCard(core) {
    const root = this.shadowRoot;
    const list = root.querySelector(".named-command-list");
    let shownNames = "";

    const render = () => {
      const names = core.getTopic(NAMED_COMMANDS_TOPIC)?.value;
      if (Array.isArray(names) && names.join("\n") !== shownNames) {
        shownNames = names.join("\n");
        list.replaceChildren(
          ...names.map((name) => {
            const row = document.createElement("li");
            row.dataset.name = name;
            row.innerHTML = `<span class="dot"></span><span></span>`;
            row.lastElementChild.textContent = name;
            return row;
          }),
        );
      }

      list.querySelectorAll("li[data-name]").forEach((row) => {
        const running = core.getTopic(`${NAMED_COMMANDS_PREFIX}${row.dataset.name}/Running`)?.value === true;
        row.classList.toggle("on", running);
      });
    };

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (topic.name.startsWith(NAMED_COMMANDS_PREFIX)) render();
    };
    core.addEventListener("topic", topicListener);
    this.#cleanup.push(() => core.removeEventListener("topic", topicListener));
    render();
  }

  /** Shows the intake roller's volts, RPM, and whether it's spinning. */
  /**
   * Shows the feeder and flywheel from the side, an RPM gauge with the target and the
   * "close enough" zone, and a big READY light. The wheels keep animating, so this card
   * redraws every screen refresh while it's open.
   */
  #startShooterCard(core) {
    const root = this.shadowRoot;
    const canvas = root.querySelector(".shooter-view");
    const readyLight = root.querySelector(".shooter-ready");
    const rpmText = root.querySelector(".shooter-rpm");
    const targetText = root.querySelector(".shooter-target");
    const feederText = root.querySelector(".feeder-rpm");
    const wheel = { flywheel: 0, feeder: 0 }; // drawn angles, built up from speed
    let lastMs = performance.now();
    let frame = 0;

    const render = () => {
      const now = performance.now();
      const seconds = Math.min(0.1, (now - lastMs) / 1000);
      lastMs = now;

      const measured = Number(core.getTopic(SHOOTER_MEASURED_TOPIC)?.value) || 0;
      const target = Number(core.getTopic(SHOOTER_TARGET_TOPIC)?.value) || 0;
      const ready = core.getTopic(SHOOTER_READY_TOPIC)?.value === true;
      const feederRadPerSec = Number(core.getTopic(FEEDER_VELOCITY_TOPIC)?.value) || 0;
      const feederRpm = (feederRadPerSec * 60) / (2 * Math.PI);

      wheel.flywheel += measured * SHOOTER_DRAW_SLOWDOWN * seconds;
      wheel.feeder += feederRpm * SHOOTER_DRAW_SLOWDOWN * seconds;

      rpmText.textContent = String(Math.round(measured));
      targetText.textContent = String(Math.round(target));
      feederText.textContent = String(Math.round(feederRpm));
      readyLight.textContent = ready ? "READY" : "NOT READY";
      readyLight.classList.toggle("on", ready);
      drawShooter(canvas, { measured, target, ready, feederRpm, wheel });
      frame = requestAnimationFrame(render);
    };
    frame = requestAnimationFrame(render);
    this.#cleanup.push(() => cancelAnimationFrame(frame));
    this.#startShotTrim(core);
  }

  /**
   * The Shot trim buttons on the Shooter card. They publish a new trim to the robot, which adds
   * it to every table shot's RPM (negative = slower = shorter).
   */
  #startShotTrim(core) {
    const root = this.shadowRoot;
    const valueText = root.querySelector(".trim-value");
    const buttons = [...root.querySelectorAll(".trim-button")];
    const resetButton = root.querySelector(".trim-reset");
    const slider = root.querySelector(".trim-range");
    const sliderBox = root.querySelector(".trim-slider");
    const defaultMark = root.querySelector(".trim-default");
    const warning = root.querySelector(".trim-warning");
    const status = root.querySelector(".trim-status");
    let codeDefault = null; // the first value we see is the robot's ShooterConstants value
    let pending = null; // a value we sent that the robot hasn't confirmed yet
    let pendingSince = 0;
    let dragging = false; // don't move the slider out from under the user's finger

    // Set up the slider's range and where the yellow caution zones start.
    slider.min = String(-SHOT_TRIM_LIMIT_RPM);
    slider.max = String(SHOT_TRIM_LIMIT_RPM);
    slider.step = String(SHOT_TRIM_STEP_RPM);
    root.querySelector(".trim-min").textContent = String(-SHOT_TRIM_LIMIT_RPM);
    root.querySelector(".trim-max").textContent = `+${SHOT_TRIM_LIMIT_RPM}`;
    /** Where a trim value sits along the slider, as a percent from the left. */
    const percent = (rpm) => ((rpm + SHOT_TRIM_LIMIT_RPM) / (2 * SHOT_TRIM_LIMIT_RPM)) * 100;
    sliderBox.style.setProperty("--caution-low", `${percent(-SHOT_TRIM_CAUTION_RPM)}%`);
    sliderBox.style.setProperty("--caution-high", `${percent(SHOT_TRIM_CAUTION_RPM)}%`);

    /** What the robot says it's using. */
    const robotValue = () => Number(core.getTopic(SHOT_TRIM_ROBOT_TOPIC)?.value);
    // Show what we just sent right away, until the robot confirms it.
    const current = () => (pending !== null ? pending : robotValue());
    const render = () => {
      const robot = robotValue();
      if (pending !== null && robot === pending) pending = null; // the robot has it
      const trim = current();
      const known = Number.isFinite(trim);
      const slow = pending !== null && performance.now() - pendingSince > 1500;
      status.textContent = !known
        ? "Waiting for the robot…"
        : pending === null
          ? "✓ Robot is using this"
          : slow
            ? "⚠ Robot hasn't picked this up yet. Is it connected and running the latest code?"
            : "Sending…";
      status.classList.toggle("caution", slow);
      if (known && codeDefault === null) codeDefault = trim;
      const caution = known && Math.abs(trim) > SHOT_TRIM_CAUTION_RPM;
      valueText.textContent = known ? `${trim > 0 ? "+" : ""}${Math.round(trim)} RPM` : "—";
      valueText.classList.toggle("caution", caution);
      warning.hidden = !caution;
      if (known && !dragging) slider.value = String(trim);
      slider.disabled = !known;
      sliderBox.style.setProperty("--trim", `${percent(known ? trim : 0)}%`);
      if (codeDefault !== null) defaultMark.style.left = `${percent(codeDefault)}%`;
      defaultMark.hidden = codeDefault === null;
      for (const button of [...buttons, resetButton]) button.disabled = !known;
      resetButton.disabled = !known || trim === codeDefault;
    };
    const send = (trim) => {
      const clamped = Math.max(-SHOT_TRIM_LIMIT_RPM, Math.min(SHOT_TRIM_LIMIT_RPM, trim));
      core.publish(SHOT_TRIM_TOPIC, "double", clamped);
      pending = clamped;
      pendingSince = performance.now();
      render();
    };

    const handlers = buttons.map((button) => {
      const onClick = () => send(current() + Number(button.dataset.step) * SHOT_TRIM_STEP_RPM);
      button.addEventListener("click", onClick);
      return () => button.removeEventListener("click", onClick);
    });
    const onReset = () => codeDefault !== null && send(codeDefault);
    resetButton.addEventListener("click", onReset);

    // Dragging the slider sends each new value right away, so the RPM changes live.
    const onSlide = () => {
      dragging = true;
      send(Number(slider.value));
    };
    const onRelease = () => {
      dragging = false;
      render();
    };
    slider.addEventListener("input", onSlide);
    slider.addEventListener("change", onRelease);

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (topic.name === SHOT_TRIM_TOPIC || topic.name === SHOT_TRIM_ROBOT_TOPIC) render();
    };
    core.addEventListener("topic", topicListener);
    // Re-check now and then, so "hasn't picked this up" shows even if nothing new arrives.
    const timer = setInterval(() => pending !== null && render(), 500);
    this.#cleanup.push(() => {
      clearInterval(timer);
      core.removeEventListener("topic", topicListener);
      resetButton.removeEventListener("click", onReset);
      slider.removeEventListener("input", onSlide);
      slider.removeEventListener("change", onRelease);
      handlers.forEach((remove) => remove());
    });
    render();
  }

  /**
   * Shows the spindexer from above: a disc that turns with its MEASURED position
   * (so it lags or stops if the motor does), plus volts, RPM, and what it's doing.
   */
  #startSpindexerCard(core) {
    const root = this.shadowRoot;
    const canvas = root.querySelector(".spindexer-disc");
    const voltsText = root.querySelector(".spindexer-volts");
    const rpmText = root.querySelector(".spindexer-rpm");
    const stateLight = root.querySelector(".spindexer-state");
    let frame = 0;

    const render = () => {
      frame = 0;
      const angle = Number(core.getTopic(SPINDEXER_POSITION_TOPIC)?.value) || 0;
      const radPerSec = Number(core.getTopic(SPINDEXER_VELOCITY_TOPIC)?.value) || 0;
      const volts = Number(core.getTopic(SPINDEXER_VOLTS_TOPIC)?.value) || 0;
      const rpm = (radPerSec * 60) / (2 * Math.PI);
      const spinning = Math.abs(rpm) > SPINDEXER_SPINNING_RPM;
      const state = !spinning ? "STOPPED" : rpm > 0 ? "FEEDING" : "UNJAMMING";
      voltsText.textContent = volts.toFixed(1);
      rpmText.textContent = rpm.toFixed(1);
      stateLight.textContent = state;
      stateLight.classList.toggle("on", spinning);
      drawSpindexer(canvas, angle, state);
    };
    const scheduleRender = () => {
      if (!frame) frame = requestAnimationFrame(render);
    };

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if ([SPINDEXER_POSITION_TOPIC, SPINDEXER_VELOCITY_TOPIC, SPINDEXER_VOLTS_TOPIC].includes(topic.name))
        scheduleRender();
    };
    core.addEventListener("topic", topicListener);
    this.#cleanup.push(() => {
      core.removeEventListener("topic", topicListener);
      cancelAnimationFrame(frame);
    });
    render();
  }

  #startIntakeCard(core) {
    const root = this.shadowRoot;
    const voltsText = root.querySelector(".roller-volts");
    const rpmText = root.querySelector(".roller-rpm");
    const spinningLight = root.querySelector(".roller-spinning");
    const armCanvas = root.querySelector(".arm-view");
    const armMeasuredText = root.querySelector(".arm-measured");
    const armTargetText = root.querySelector(".arm-target");
    const armLight = root.querySelector(".arm-at-target");
    let frame = 0;

    const render = () => {
      frame = 0;
      // Roller
      const volts = Number(core.getTopic(ROLLER_VOLTS_TOPIC)?.value) || 0;
      const { rpm, state } = readRoller(core);
      voltsText.textContent = volts.toFixed(1);
      rpmText.textContent = String(Math.round(rpm));
      spinningLight.textContent = state;
      spinningLight.classList.toggle("on", state !== "STOPPED");

      // Arm
      const measured = Number(core.getTopic(ARM_MEASURED_TOPIC)?.value);
      const target = Number(core.getTopic(ARM_TARGET_TOPIC)?.value);
      const level = Number(core.getTopic(ARM_LEVEL_TOPIC)?.value);
      const atTarget = core.getTopic(ARM_AT_TARGET_TOPIC)?.value === true;
      const hasTarget = Number.isFinite(target);
      armMeasuredText.textContent = Number.isFinite(measured) ? measured.toFixed(2) : "—";
      armTargetText.textContent = hasTarget ? target.toFixed(2) : "—";
      armLight.textContent = !hasTarget ? "NO TARGET" : atTarget ? "AT TARGET" : "MOVING";
      armLight.classList.toggle("on", atTarget);
      drawArm(armCanvas, measured, target, Number.isFinite(level) ? level : 0);
    };
    const scheduleRender = () => {
      if (!frame) frame = requestAnimationFrame(render);
    };

    const topics = [
      ROLLER_VOLTS_TOPIC,
      ROLLER_VELOCITY_TOPIC,
      ARM_MEASURED_TOPIC,
      ARM_TARGET_TOPIC,
      ARM_AT_TARGET_TOPIC,
      ARM_LEVEL_TOPIC,
    ];
    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (topics.includes(topic.name)) scheduleRender();
    };
    core.addEventListener("topic", topicListener);
    this.#cleanup.push(() => {
      core.removeEventListener("topic", topicListener);
      cancelAnimationFrame(frame);
    });
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
        aiming: core.getTopic(AIM_ACTIVE_TOPIC)?.value === true,
        aimLocked: core.getTopic(AIM_LOCKED_TOPIC)?.value === true,
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
      if ([POSE_TOPIC, ALLIANCE_TOPIC, STATION_TOPIC, SPEED_TOPIC, HUBS_TOPIC, FIELD_SIZE_TOPIC, ROBOT_SIZE_TOPIC, ROLLER_VELOCITY_TOPIC, MAX_DISTANCE_TOPIC, AIM_ACTIVE_TOPIC, AIM_LOCKED_TOPIC].includes(topic.name))
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
function drawField(canvas, bounds, { isRed, station, pose, hubs, cameraStates, intakeState, aiming, aimLocked }) {
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
  if (pose && aiming) drawAimLine(context, pose, hubs, aimLocked, { isRed, left, top, scale });
  if (pose) drawRobot(context, pose, intakeState, { isRed, left, top, scale });
}

/**
 * While aiming, draws a line from the robot's back (where the shooter is) to our hub.
 * Yellow while turning, green once AIM LOCKED.
 */
function drawAimLine(context, pose, hubs, locked, view) {
  // Our hub: blue is the first box in Field/Hubs, red the second.
  const box = hubs.slice(view.isRed ? 4 : 0, view.isRed ? 8 : 4);
  if (box.length < 4) return;
  const hub = fieldToScreen((box[0] + box[2]) / 2, (box[1] + box[3]) / 2, view);
  const half = robotSizeMeters / 2;
  const back = fieldToScreen(
    pose.x - Math.cos(pose.heading) * half,
    pose.y - Math.sin(pose.heading) * half,
    view,
  );
  context.strokeStyle = locked ? "#4ade80" : "#facc15";
  context.lineWidth = 2;
  context.setLineDash(locked ? [] : [6, 4]);
  context.beginPath();
  context.moveTo(back.x, back.y);
  context.lineTo(hub.x, hub.y);
  context.stroke();
  context.setLineDash([]);
  context.lineWidth = 1;
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
  // The shooter faces out the back: a yellow bar on the back edge.
  context.fillStyle = "#facc15";
  context.fillRect(-size / 2, -size / 4, Math.max(3, size * 0.1), size / 2);
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
/**
 * Draws the intake arm from the side, with the robot's front to the RIGHT. A solid line
 * shows where the arm IS; a faint dashed "ghost" shows where it's TRYING to go.
 * Our angles: 0 = stowed, bigger = lower. "level" is our angle when the arm is flat.
 */
function drawArm(canvas, measured, target, level) {
  const context = canvas.getContext("2d");
  const { width, height } = canvas;
  const pivot = { x: width * 0.35, y: height * 0.42 };
  const length = width * 0.5;
  context.clearRect(0, 0, width, height);

  // Floor and a hint of the robot body behind the pivot.
  const floorY = height * 0.88;
  context.strokeStyle = "#7c8798";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(0, floorY);
  context.lineTo(width, floorY);
  context.stroke();
  context.fillStyle = "#2a3240";
  context.fillRect(pivot.x - width * 0.3, pivot.y, width * 0.3, floorY - pivot.y - 4);
  context.fillStyle = "#93a0b4";
  context.font = "700 11px system-ui, sans-serif";
  context.textAlign = "center";
  context.fillText("ROBOT", pivot.x - width * 0.15, (pivot.y + floorY) / 2 + 4);
  context.textAlign = "right";
  context.fillText("FRONT →", width - 6, floorY - 6);

  // Our angle → a point on screen. Angle from level (up = positive) = level - ours.
  const tip = (angle) => {
    const fromLevel = level - angle;
    return { x: pivot.x + Math.cos(fromLevel) * length, y: pivot.y - Math.sin(fromLevel) * length };
  };
  const drawLine = (angle, style) => {
    const end = tip(angle);
    context.strokeStyle = style.color;
    context.lineWidth = style.width;
    context.setLineDash(style.dash ?? []);
    context.lineCap = "round";
    context.beginPath();
    context.moveTo(pivot.x, pivot.y);
    context.lineTo(end.x, end.y);
    context.stroke();
    context.setLineDash([]);
  };

  if (Number.isFinite(target)) drawLine(target, { color: "#93a0b466", width: 10, dash: [8, 8] });
  if (Number.isFinite(measured)) drawLine(measured, { color: "#4ade80", width: 10 });

  // Pivot point.
  context.beginPath();
  context.arc(pivot.x, pivot.y, 7, 0, Math.PI * 2);
  context.fillStyle = "#edf2f7";
  context.fill();
}

/**
 * Draws the shooter from the side: the feeder roller on the left pushing a game piece
 * into the big flywheel on the right, and an RPM gauge underneath with a target marker
 * and the "close enough to shoot" zone.
 */
function drawShooter(canvas, { measured, target, ready, feederRpm, wheel }) {
  const context = canvas.getContext("2d");
  const { width, height } = canvas;
  context.clearRect(0, 0, width, height);

  const feeding = Math.abs(feederRpm) > 100;
  const flyColor = ready ? "#4ade80" : Math.abs(measured) > 100 ? "#facc15" : "#93a0b4";
  const feedColor = feeding ? "#4ade80" : "#93a0b4";

  // Spoked wheel, turned to a drawn angle.
  const drawWheel = (x, y, r, angle, color, spokes) => {
    context.beginPath();
    context.arc(x, y, r, 0, Math.PI * 2);
    context.fillStyle = "#0c1017";
    context.fill();
    context.lineWidth = 5;
    context.strokeStyle = color;
    context.stroke();
    context.save();
    context.translate(x, y);
    context.rotate(-angle); // positive = counter-clockwise on screen
    context.lineWidth = 3;
    for (let i = 0; i < spokes; i++) {
      const a = (i * Math.PI * 2) / spokes;
      context.beginPath();
      context.moveTo(0, 0);
      context.lineTo(Math.cos(a) * r * 0.85, Math.sin(a) * r * 0.85);
      context.strokeStyle = i === 0 ? color : "#d6dde8";
      context.stroke();
    }
    context.restore();
    context.beginPath();
    context.arc(x, y, r * 0.15, 0, Math.PI * 2);
    context.fillStyle = color;
    context.fill();
  };

  const feeder = { x: width * 0.2, y: height * 0.38, r: 26 };
  const flywheel = { x: width * 0.62, y: height * 0.34, r: 52 };

  // Path the game piece takes: in from the left, past the feeder, out of the flywheel.
  context.setLineDash([6, 6]);
  context.strokeStyle = "#7c8798";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(8, feeder.y + feeder.r + 8);
  context.lineTo(flywheel.x - 10, flywheel.y + flywheel.r + 8);
  context.quadraticCurveTo(width - 20, flywheel.y + flywheel.r, width - 12, 12);
  context.stroke();
  context.setLineDash([]);

  drawWheel(feeder.x, feeder.y, feeder.r, wheel.feeder, feedColor, 4);
  drawWheel(flywheel.x, flywheel.y, flywheel.r, wheel.flywheel, flyColor, 6);

  context.fillStyle = "#93a0b4";
  context.font = "700 11px system-ui, sans-serif";
  context.textAlign = "center";
  context.fillText("FEEDER", feeder.x, feeder.y - feeder.r - 8);
  context.fillText("FLYWHEEL", flywheel.x, flywheel.y - flywheel.r - 8);

  // A game piece riding along while the feeder runs.
  if (feeding) {
    const t = (performance.now() / 600) % 1;
    const px = 8 + (flywheel.x - 18) * t;
    const py = feeder.y + feeder.r + 8 + (flywheel.y + flywheel.r - feeder.y - feeder.r) * t;
    context.beginPath();
    context.arc(px, py - 9, 9, 0, Math.PI * 2);
    context.fillStyle = "#fbbf24";
    context.fill();
  }

  // RPM gauge: bar = flywheel speed, line = target, shaded = "close enough" zone.
  const gauge = { x: 16, y: height - 34, w: width - 32, h: 14 };
  const toX = (rpm) => gauge.x + (Math.max(0, Math.min(SHOOTER_GAUGE_MAX_RPM, rpm)) / SHOOTER_GAUGE_MAX_RPM) * gauge.w;
  context.fillStyle = "#2a3240";
  context.fillRect(gauge.x, gauge.y, gauge.w, gauge.h);
  if (target > 0) {
    context.fillStyle = "#4ade8033";
    context.fillRect(toX(target - 100), gauge.y - 4, toX(target + 100) - toX(target - 100), gauge.h + 8);
  }
  context.fillStyle = flyColor;
  context.fillRect(gauge.x, gauge.y, toX(measured) - gauge.x, gauge.h);
  if (target > 0) {
    context.fillStyle = "#edf2f7";
    context.fillRect(toX(target) - 1.5, gauge.y - 6, 3, gauge.h + 12);
  }
  context.fillStyle = "#93a0b4";
  context.textAlign = "left";
  context.fillText("0", gauge.x, gauge.y + gauge.h + 14);
  context.textAlign = "right";
  context.fillText(`${SHOOTER_GAUGE_MAX_RPM} RPM`, gauge.x + gauge.w, gauge.y + gauge.h + 14);
}

/**
 * Draws the spindexer seen from above: a round floor with paddles, turned to its
 * measured angle. Positive angle (feeding) turns counter-clockwise, like math angles.
 */
function drawSpindexer(canvas, angle, state) {
  const context = canvas.getContext("2d");
  const size = canvas.width;
  const center = size / 2;
  const radius = size * 0.42;
  const color = state === "FEEDING" ? "#4ade80" : state === "UNJAMMING" ? "#fb923c" : "#93a0b4";
  context.clearRect(0, 0, size, size);

  // The floor of the hopper.
  context.beginPath();
  context.arc(center, center, radius, 0, Math.PI * 2);
  context.fillStyle = "#0c1017";
  context.fill();
  context.lineWidth = 6;
  context.strokeStyle = color;
  context.stroke();

  // Paddles that turn with the measured position. Screen y points down, so flip the
  // angle to make positive (counter-clockwise) look counter-clockwise.
  context.save();
  context.translate(center, center);
  context.rotate(-angle);
  context.strokeStyle = "#d6dde8";
  context.lineWidth = 8;
  context.lineCap = "round";
  for (let i = 0; i < 6; i++) {
    const a = (i * Math.PI * 2) / 6;
    context.beginPath();
    context.moveTo(Math.cos(a) * radius * 0.18, Math.sin(a) * radius * 0.18);
    context.lineTo(Math.cos(a) * radius * 0.9, Math.sin(a) * radius * 0.9);
    context.stroke();
  }
  // One colored paddle so you can count turns.
  context.strokeStyle = color;
  context.beginPath();
  context.moveTo(radius * 0.18, 0);
  context.lineTo(radius * 0.9, 0);
  context.stroke();
  context.restore();

  // Hub in the middle.
  context.beginPath();
  context.arc(center, center, radius * 0.16, 0, Math.PI * 2);
  context.fillStyle = color;
  context.fill();

  // Direction arrow around the edge while spinning.
  if (state !== "STOPPED") {
    const ccw = state === "FEEDING";
    const start = -Math.PI * 0.75;
    const end = -Math.PI * 0.25;
    const arrowRadius = radius + 14;
    context.strokeStyle = color;
    context.lineWidth = 4;
    context.beginPath();
    context.arc(center, center, arrowRadius, start, end);
    context.stroke();
    const tip = ccw ? start : end;
    const tx = center + Math.cos(tip) * arrowRadius;
    const ty = center + Math.sin(tip) * arrowRadius;
    const dir = ccw ? -1 : 1; // which way along the circle the arrow points
    const tangent = tip + (dir * Math.PI) / 2;
    context.beginPath();
    context.moveTo(tx, ty);
    context.lineTo(tx - Math.cos(tangent - 0.5) * 14, ty - Math.sin(tangent - 0.5) * 14);
    context.lineTo(tx - Math.cos(tangent + 0.5) * 14, ty - Math.sin(tangent + 0.5) * 14);
    context.closePath();
    context.fillStyle = color;
    context.fill();
  }
}

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
