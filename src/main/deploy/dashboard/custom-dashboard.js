/** @typedef {import("./core/dashboard-core.js").DashboardCore} DashboardCore */
/** @typedef {import("./core/dashboard-core.js").DashboardTopic} DashboardTopic */

/** Owns the editable dashboard UI and its connection to the protected core API. */
export class CustomDashboard extends HTMLElement {
  /** @type {DashboardCore | undefined} */
  #core;
  /** @type {boolean} */
  #started = false;
  /** @type {Array<() => void>} */
  #cleanup = [];

  connectedCallback() {
    if (!this.shadowRoot) {
      const root = this.attachShadow({ mode: "open" });
      const version = new URL(import.meta.url).searchParams.get("v") ?? "initial";
      root.innerHTML = `<link rel="stylesheet" href="custom-dashboard.css?v=${version}"><style>:host { flex: 1; min-height: 0; display: flex; flex-direction: column; }</style>
      <nav><div class="tabs" role="tablist"><button class="tab selected" data-tab="diagnostics">Main</button><button class="tab" data-tab="cameras">Cameras</button><button class="tab" data-tab="networktables">NetworkTables</button></div></nav>
      <main id="app-root"><div class="aim-lock-frame"><div class="aim-lock-label" aria-label="Aim lock active"><span></span>AIM LOCK<span></span></div><section id="diagnostics-panel" class="card tab-panel main-dashboard"><div class="dashboard-column field-section"><div class="auto-bar"><label for="auto-selector">Autonomous</label><select id="auto-selector" disabled><option>Waiting for chooser…</option></select></div><div class="section-title"><h2>Field</h2><span id="field-pose" class="field-pose">Waiting for pose…</span></div><div id="field-view" class="field-view"><canvas id="field-canvas"></canvas></div></div><div class="dashboard-column faults-column"><div class="section-title faults-title"><h2>Faults</h2><span id="diagnostic-summary" class="diagnostic-summary">Waiting for devices…</span></div><div id="diagnostics" class="diagnostics"></div></div></section></div>
      <section id="networktables-panel" class="card tab-panel" hidden><div class="section-title"><h2>NetworkTables</h2><input id="topic-filter" type="search" placeholder="Filter topics" autocomplete="off"></div><div id="topics" class="topics"></div></section>
      <section id="cameras-panel" class="card tab-panel" hidden><div class="section-title"><h2>Camera streams</h2><label class="camera-host-label">PhotonVision host <input id="camera-host" value="photonvision.local" spellcheck="false"></label></div><div id="cameras" class="cameras"></div></section></main>`;
    }
    if (this.#core && !this.#started) { this.#started = true; this.#start(this.#core); }
  }

  disconnectedCallback() {
    this.#cleanup.forEach((cleanup) => cleanup());
    this.#cleanup = [];
    this.#started = false;
  }

  /** @param {DashboardCore} core The shared protected core API. */
  set core(core) {
    this.#core = core;
    if (!this.#started && this.shadowRoot) {
      this.#started = true;
      this.#start(core);
    }
    this.dispatchEvent(new CustomEvent("core-ready", { detail: core }));
  }

  /** @returns {DashboardCore | undefined} */
  get core() { return this.#core; }

  /** @param {DashboardCore} core The shared protected core API. */
  #start(core) {
const dashboard = this;
const ui = dashboard.shadowRoot;
const getTopic = (name) => core.getTopic(name);
const getTopics = () => core.getTopics();
const topicsElement = ui.querySelector("#topics");
const filterElement = ui.querySelector("#topic-filter");
const diagnosticsElement = ui.querySelector("#diagnostics");
const diagnosticSummary = ui.querySelector("#diagnostic-summary");
const autoSelect = ui.querySelector("#auto-selector");
const camerasElement = ui.querySelector("#cameras");
const cameraHostInput = ui.querySelector("#camera-host");
const fieldCanvas = ui.querySelector("#field-canvas");
const fieldView = ui.querySelector("#field-view");
const fieldPose = ui.querySelector("#field-pose");
const aimLockFrame = ui.querySelector(".aim-lock-frame");
const diagnosticPrefix = "/SmartDashboard/HardwareMonitor/FaultStatus/";
const autoPrefix = "/SmartDashboard/Auto Choices/";
const aimLockTopic = "/SmartDashboard/aimLocked";
const cameraDefinitions = [
  { name: "Front left", camera: "camera_frontleft", port: 1185, flipped: true },
  { name: "Front right", camera: "camera_frontright", port: 1181, flipped: true },
  { name: "Back left", camera: "camera_backleft", port: 1187, flipped: true },
  { name: "Back right", camera: "camera_backright", port: 1183, flipped: true },
];
const field = { length: 17.548, width: 8.052 };
let renderPending = false;
let fieldRenderPending = false;
const cameraElements = cameraDefinitions.map((definition) => {
  const card = document.createElement("article");
  card.className = "camera";
  card.innerHTML = `<div class="camera-header"><span>${definition.name}</span><span class="camera-status">Paused</span></div><div class="camera-frame"><img alt="${definition.name} camera"><span class="camera-placeholder">Open the Cameras tab to stream</span></div>`;
  const image = card.querySelector("img");
  image.classList.toggle("flipped", definition.flipped === true);
  const status = card.querySelector(".camera-status");
  const placeholder = card.querySelector(".camera-placeholder");
  image.addEventListener("load", () => { status.textContent = "Live"; status.className = "camera-status live"; placeholder.hidden = true; });
  image.addEventListener("error", () => { status.textContent = "Unavailable"; status.className = "camera-status offline"; });
  camerasElement.append(card);
  return { ...definition, image, status, placeholder };
});
cameraHostInput.value = localStorage.getItem("photonvision-host") || "photonvision.local";
/** @param {CustomEvent<DashboardTopic>} event */
const topicListener = ({ detail: topic }) => {
  if (topic.name === aimLockTopic) updateAimLock();
  if (topic.name === "/AdvantageKit/RealOutputs/Odometry/Robot" || topic.name === "/FMSInfo/IsRedAlliance" || topic.name === "/FMSInfo/StationNumber") {
    scheduleFieldRender();
  }
  scheduleRender();
};
core.addEventListener("topic", topicListener);
this.#cleanup.push(() => core.removeEventListener("topic", topicListener));

filterElement.addEventListener("input", scheduleRender);
autoSelect.addEventListener("change", () => {
  core.publish(`${autoPrefix}selected`, "string", autoSelect.value);
});
cameraHostInput.addEventListener("change", () => {
  localStorage.setItem("photonvision-host", cameraHostInput.value.trim());
  updateCameraStreams(true);
});
ui.querySelectorAll("[data-tab]").forEach((button) => {
  button.addEventListener("click", () => {
    ui.querySelectorAll("[data-tab]").forEach((tab) => tab.classList.toggle("selected", tab === button));
    ui.querySelectorAll(".tab-panel").forEach((panel) => { panel.hidden = panel.id !== `${button.dataset.tab}-panel`; });
    aimLockFrame.hidden = button.dataset.tab !== "diagnostics";
    updateCameraStreams();
  });
});

function updateAimLock() {
  aimLockFrame.classList.toggle("aim-locked", getTopic(aimLockTopic)?.value === true);
}

function scheduleRender() {
  if (renderPending) return;
  renderPending = true;
  setTimeout(renderTopics, 100);
}

function renderTopics() {
  renderPending = false;
  updateAimLock();
  renderAutoChooser();
  renderDiagnostics();
  updateCameraStreams();
  const filter = filterElement.value.toLowerCase();
  const topics = getTopics().filter((topic) => topic.name.toLowerCase().includes(filter)).sort((a, b) => a.name.localeCompare(b.name));
  if (!topics.length) {
    topicsElement.innerHTML = '<div class="empty">Waiting for NetworkTables data…</div>';
    return;
  }
  topicsElement.replaceChildren(...topics.map((topic) => {
    const row = document.createElement("div");
    row.className = "topic";
    const name = document.createElement("div");
    name.className = "topic-name";
    name.textContent = topic.name;
    const value = document.createElement("div");
    value.className = "topic-value";
    value.title = formatValue(topic.value);
    value.textContent = value.title;
    row.append(name, value);
    return row;
  }));
}

function scheduleFieldRender() {
  if (fieldRenderPending) return;
  fieldRenderPending = true;
  requestAnimationFrame(() => {
    fieldRenderPending = false;
    renderField();
  });
}

function renderField() {
  const ratio = window.devicePixelRatio || 1;
  const bounds = fieldView.getBoundingClientRect();
  const width = Math.max(1, Math.round(bounds.width * ratio));
  const height = Math.max(1, Math.round(bounds.height * ratio));
  if (fieldCanvas.width !== width || fieldCanvas.height !== height) {
    fieldCanvas.width = width;
    fieldCanvas.height = height;
  }
  const context = fieldCanvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  const canvasWidth = bounds.width;
  const canvasHeight = bounds.height;
  context.clearRect(0, 0, canvasWidth, canvasHeight);

  const padding = 10;
  const allianceZoneHeight = 30;
  const allianceZoneGap = 5;
  const usableWidth = canvasWidth - padding * 2;
  const usableHeight = canvasHeight - padding * 2;
  const scale = Math.min(usableWidth / field.width, (usableHeight - allianceZoneHeight - allianceZoneGap) / field.length);
  const drawnWidth = field.width * scale;
  const drawnHeight = field.length * scale;
  const left = (canvasWidth - drawnWidth) / 2;
  const top = (canvasHeight - drawnHeight - allianceZoneGap - allianceZoneHeight) / 2;
  const red = getTopic("/FMSInfo/IsRedAlliance")?.value === true;

  context.fillStyle = "#202630";
  context.strokeStyle = "#7c8798";
  context.lineWidth = 1;
  context.fillRect(left, top, drawnWidth, drawnHeight);
  context.strokeRect(left, top, drawnWidth, drawnHeight);

  context.setLineDash([4, 5]);
  context.strokeStyle = "#9ca89b66";
  for (const fraction of [0.25, 0.5, 0.75]) {
    const y = top + drawnHeight * fraction;
    context.beginPath(); context.moveTo(left, y); context.lineTo(left + drawnWidth, y); context.stroke();
  }
  context.setLineDash([]);
  const allianceZoneTop = top + drawnHeight + allianceZoneGap;
  context.fillStyle = red ? "#b52c3c" : "#2879cf";
  context.fillRect(left, allianceZoneTop, drawnWidth, allianceZoneHeight);
  context.strokeStyle = red ? "#ff6877" : "#63b2ff";
  context.strokeRect(left, allianceZoneTop, drawnWidth, allianceZoneHeight);
  const stationValue = Number(getTopic("/FMSInfo/StationNumber")?.value);
  const station = Number.isInteger(stationValue) && stationValue >= 1 && stationValue <= 3 ? stationValue : null;
  context.fillStyle = "#ffffff";
  context.font = "700 10px Inter, ui-sans-serif, system-ui, sans-serif";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(`YOUR ALLIANCE${station ? ` · STATION ${station}` : ""}`, left + drawnWidth / 2, allianceZoneTop + allianceZoneHeight / 2);

  const pose = decodePose2d(getTopic("/AdvantageKit/RealOutputs/Odometry/Robot")?.value);
  if (!pose) {
    fieldPose.textContent = "Waiting for pose…";
    return;
  }
  const { x, y, heading } = pose;

  const project = (fieldX, fieldY) => ({
    x: left + (red ? fieldY : field.width - fieldY) * scale,
    y: top + (red ? fieldX : field.length - fieldX) * scale,
  });
  const position = project(x, y);
  const front = project(x + Math.cos(heading), y + Math.sin(heading));
  const screenHeading = Math.atan2(front.y - position.y, front.x - position.x);
  const robotSize = Math.max(10, 0.7112 * scale);
  context.save();
  context.translate(position.x, position.y);
  context.rotate(screenHeading);
  context.shadowColor = red ? "#ff5264" : "#57a6ff";
  context.shadowBlur = 10;
  context.fillStyle = red ? "#d83b4c" : "#3287df";
  context.strokeStyle = "#f3f7fb";
  context.lineWidth = 2;
  context.fillRect(-robotSize / 2, -robotSize / 2, robotSize, robotSize);
  context.strokeRect(-robotSize / 2, -robotSize / 2, robotSize, robotSize);
  context.beginPath();
  context.moveTo(robotSize / 2, 0);
  context.lineTo(robotSize / 4, -robotSize / 4);
  context.lineTo(robotSize / 4, robotSize / 4);
  context.closePath();
  context.fillStyle = "#ffffff";
  context.fill();
  context.restore();
  fieldPose.textContent = `x ${x.toFixed(2)} · y ${y.toFixed(2)} · ${Math.round(heading * 180 / Math.PI)}°`;
}

function decodePose2d(value) {
  if (Array.isArray(value) && value.length >= 3) {
    const pose = { x: Number(value[0]), y: Number(value[1]), heading: Number(value[2]) };
    return Object.values(pose).every(Number.isFinite) ? pose : null;
  }
  if (!(value instanceof Uint8Array) || value.byteLength < 24) return null;
  const view = new DataView(value.buffer, value.byteOffset, value.byteLength);
  const pose = {
    x: view.getFloat64(0, true),
    y: view.getFloat64(8, true),
    heading: view.getFloat64(16, true),
  };
  return Object.values(pose).every(Number.isFinite) ? pose : null;
}

const fieldObserver = new ResizeObserver(scheduleFieldRender);
fieldObserver.observe(fieldView);
this.#cleanup.push(() => fieldObserver.disconnect());

function updateCameraStreams(force = false) {
  const panelOpen = !ui.querySelector("#cameras-panel").hidden;
  const connection = core.connection;
  const cameraHost = connection.isSimulation ? connection.host : cameraHostInput.value.trim();
  for (const camera of cameraElements) {
    if (!panelOpen || !cameraHost) {
      if (camera.image.src) camera.image.removeAttribute("src");
      camera.status.textContent = "Paused";
      camera.status.className = "camera-status";
      camera.placeholder.hidden = false;
      continue;
    }
    const source = `http://${cameraHost}:${camera.port}/?action=stream`;
    if (force || camera.image.dataset.source !== source || !camera.image.hasAttribute("src")) {
      camera.image.dataset.source = source;
      camera.status.textContent = "Connecting…";
      camera.status.className = "camera-status";
      camera.placeholder.hidden = false;
      camera.image.src = `${source}&view=${Date.now()}`;
    }
  }
}

function renderAutoChooser() {
  const options = getTopic(`${autoPrefix}options`)?.value;
  if (!Array.isArray(options) || !options.length) {
    autoSelect.disabled = true;
    return;
  }
  const signature = JSON.stringify(options);
  if (autoSelect.dataset.options !== signature) {
    autoSelect.replaceChildren(...options.map((option) => {
      const element = document.createElement("option");
      element.value = option;
      element.textContent = option;
      return element;
    }));
    autoSelect.dataset.options = signature;
  }
  const selected = getTopic(`${autoPrefix}selected`)?.value;
  const active = getTopic(`${autoPrefix}active`)?.value;
  const defaultOption = getTopic(`${autoPrefix}default`)?.value;
  const current = selected || active || defaultOption;
  if (typeof current === "string" && options.includes(current) && ui.activeElement !== autoSelect) {
    autoSelect.value = current;
  }
  autoSelect.disabled = false;
}

function renderDiagnostics() {
  const devices = getTopics()
    .filter((topic) => topic.name.startsWith(diagnosticPrefix) && typeof topic.value === "boolean")
    .map((topic) => ({ name: topic.name.slice(diagnosticPrefix.length), good: topic.value }))
    .sort((a, b) => a.name.localeCompare(b.name));

  if (!devices.length) {
    diagnosticsElement.innerHTML = '<div class="empty">Waiting for HardwareMonitor data…</div>';
    diagnosticSummary.textContent = "Waiting for devices…";
    diagnosticSummary.className = "diagnostic-summary";
    return;
  }

  const faults = devices.filter((device) => !device.good).length;
  diagnosticSummary.textContent = faults ? `${faults} fault${faults === 1 ? "" : "s"} · ${devices.length} devices` : `All ${devices.length} devices healthy`;
  diagnosticSummary.className = `diagnostic-summary ${faults ? "bad" : "good"}`;
  const drivetrain = devices.filter((device) => classifyDevice(device.name) === "drivetrain");
  const vision = devices.filter((device) => classifyDevice(device.name) === "vision");
  const mechanisms = devices.filter((device) => classifyDevice(device.name) === "mechanisms");
  diagnosticsElement.replaceChildren(
    createOrientedGroup("Drivetrain", drivetrain),
    createOrientedGroup("Vision", vision),
    createDeviceGroup("Mechanisms", mechanisms),
  );
}

function createDeviceGroup(label, groupDevices) {
    const section = document.createElement("section");
    section.className = "diagnostic-group";
    const heading = document.createElement("h3");
    heading.textContent = label;
    const grid = document.createElement("div");
    grid.className = "device-grid";
    if (groupDevices.length) {
      grid.append(...groupDevices.map(createDeviceCard));
    } else {
      const empty = document.createElement("div");
      empty.className = "group-empty";
      empty.textContent = "No registered devices";
      grid.append(empty);
    }
    section.append(heading, grid);
    return section;
}

function createOrientedGroup(label, devices) {
  const section = document.createElement("section");
  section.className = "diagnostic-group oriented-group";
  const heading = document.createElement("h3");
  heading.textContent = label;
  const direction = document.createElement("div");
  direction.className = "robot-forward";
  direction.textContent = "↑ FRONT";
  const grid = document.createElement("div");
  grid.className = "orientation-grid";
  const positions = [
    ["Front left", "frontleft"], ["Front right", "frontright"],
    ["Back left", "backleft"], ["Back right", "backright"],
  ];
  const positioned = new Set();
  for (const [, token] of positions) {
    const slot = document.createElement("div");
    slot.className = "orientation-slot";
    const matches = devices.filter((device) => normalizeDeviceName(device.name).includes(token));
    matches.forEach((device) => positioned.add(device));
    if (matches.length) slot.append(...matches.map(createDeviceCard));
    else slot.append(createUnregisteredCard());
    grid.append(slot);
  }
  section.append(heading, direction, grid);
  const extras = devices.filter((device) => !positioned.has(device));
  const gyroRegistered = devices.some((device) => {
    const name = normalizeDeviceName(device.name);
    return name.includes("gyro") || name.includes("pigeon");
  });
  if (extras.length || (label === "Drivetrain" && !gyroRegistered)) {
    const extraGrid = document.createElement("div");
    extraGrid.className = "device-grid orientation-extras";
    extraGrid.append(...extras.map(createDeviceCard));
    if (label === "Drivetrain" && !gyroRegistered) extraGrid.append(createUnregisteredCard("Gyro"));
    section.append(extraGrid);
  }
  return section;
}

function createUnregisteredCard(label) {
  const card = document.createElement("div");
  card.className = "device unregistered";
  if (label) {
    const name = document.createElement("span");
    name.textContent = label;
    const state = document.createElement("span");
    state.textContent = "Not registered";
    card.append(name, state);
  } else {
    card.textContent = "Not registered";
  }
  return card;
}

function normalizeDeviceName(name) {
  return name.toLowerCase().replace(/[^a-z0-9]/g, "");
}

function createDeviceCard(device) {
    const card = document.createElement("div");
    card.className = `device ${device.good ? "good" : "bad"}`;
    const name = document.createElement("div");
    name.className = "device-name";
    name.textContent = humanizeDeviceName(device.name);
    name.title = device.name;
    const state = document.createElement("div");
    state.className = "device-state";
    state.textContent = device.good ? "GOOD" : "FAULT";
    card.append(name, state);
    return card;
}

function classifyDevice(name) {
  const normalized = name.toLowerCase();
  if (normalized.includes("camera") || normalized.includes("vision") || normalized.includes("limelight")) return "vision";
  if (normalized.includes("pigeon") || normalized.includes("gyro") || normalized.endsWith("drive") || normalized.endsWith("turn")) return "drivetrain";
  return "mechanisms";
}

function humanizeDeviceName(name) {
  return name.replace(/([a-z0-9])([A-Z])/g, "$1 $2").replace(/[_-]+/g, " ");
}

function formatValue(value) {
  if (value instanceof Uint8Array) return `<${value.length} bytes>`;
  if (Array.isArray(value)) return JSON.stringify(value);
  if (typeof value === "number") return Number.isInteger(value) ? String(value) : value.toFixed(4);
  return String(value);
}

renderTopics();
}

}

customElements.define("custom-dashboard", CustomDashboard);
