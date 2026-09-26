const assetVersion = new URL(import.meta.url).searchParams.get("v") ?? "initial";
const { NT4Client, probeNT4 } = await import(`./nt4.js?v=${assetVersion}`);

const pageHost = location.hostname || "localhost";
let connectedHost = pageHost;
let activeConnectionId = pageHost === "localhost" ? "sim" : "page";
const nt = new NT4Client(connectedHost);
const values = new Map();
const ds = document.querySelector("#driver-station");
const dsPlaceholder = document.querySelector("#driver-station-placeholder");
const topicsElement = document.querySelector("#topics");
const filterElement = document.querySelector("#topic-filter");
const diagnosticsElement = document.querySelector("#diagnostics");
const diagnosticSummary = document.querySelector("#diagnostic-summary");
const autoSelect = document.querySelector("#auto-selector");
const camerasElement = document.querySelector("#cameras");
const cameraHostInput = document.querySelector("#camera-host");
const teamNumberInput = document.querySelector("#team-number");
const connectionPoints = document.querySelector("#connection-points");
const fieldCanvas = document.querySelector("#field-canvas");
const fieldView = document.querySelector("#field-view");
const fieldAlliance = document.querySelector("#field-alliance");
const fieldPose = document.querySelector("#field-pose");
const gamepadState = document.querySelector("#gamepad-state");
const gamepadsElement = document.querySelector("#gamepads");
const axisNames = ["LX", "LY", "LT", "RT", "RX", "RY"];
const buttonNames = ["A", "B", "X", "Y", "LB", "RB", "Back", "Start", "LS", "RS"];
const diagnosticPrefix = "/SmartDashboard/HardwareMonitor/FaultStatus/";
const autoPrefix = "/SmartDashboard/Auto Choices/";
const watchedAssets = ["index.html", "style.css", "app.js", "nt4.js", "msgpack.js", "manifest.webmanifest", "icon.svg"];
const cameraDefinitions = [
  { name: "Front left", camera: "camera_frontleft", port: 1185 },
  { name: "Front right", camera: "camera_frontright", port: 1181 },
  { name: "Back left", camera: "camera_backleft", port: 1187 },
  { name: "Back right", camera: "camera_backright", port: 1183 },
];
const field = { length: 17.548, width: 8.052 };
let selectedMode = "teleop";
let selectedAlliance = "blue";
let enabled = false;
let heartbeat = 0;
let renderPending = false;
let fieldRenderPending = false;
let ntConnected = false;
let discoveryPromise = null;
let teamDiscoveryTimer = 0;
const connectionAvailability = new Map();
const previousShortcutPovs = [-1, -1];

const gamepadViews = [0, 1].map(createGamepadView);
const cameraElements = cameraDefinitions.map((definition) => {
  const card = document.createElement("article");
  card.className = "camera";
  card.innerHTML = `<div class="camera-header"><span>${definition.name}</span><span class="camera-status">Paused</span></div><div class="camera-frame"><img alt="${definition.name} camera"><span class="camera-placeholder">Open the Cameras tab to stream</span></div>`;
  const image = card.querySelector("img");
  const status = card.querySelector(".camera-status");
  const placeholder = card.querySelector(".camera-placeholder");
  image.addEventListener("load", () => { status.textContent = "Live"; status.className = "camera-status live"; placeholder.hidden = true; });
  image.addEventListener("error", () => { status.textContent = "Unavailable"; status.className = "camera-status offline"; });
  camerasElement.append(card);
  return { ...definition, image, status, placeholder };
});
cameraHostInput.value = localStorage.getItem("photonvision-host") || "photonvision.local";
teamNumberInput.value = localStorage.getItem("frc-team-number") || "8048";
renderConnectionPoints();

nt.addEventListener("connected", () => {
  ntConnected = true;
  connectionAvailability.set(connectedHost, true);
  renderConnectionPoints();
});
nt.addEventListener("disconnected", () => {
  ntConnected = false;
  connectionAvailability.set(connectedHost, false);
  renderConnectionPoints();
  enabled = false;
  updateButtons();
});
nt.addEventListener("value", ({ detail: topic }) => {
  values.set(topic.name, topic);
  if (topic.name === "/AdvantageKit/RealOutputs/Odometry/Robot" || topic.name === "/FMSInfo/IsRedAlliance") {
    scheduleFieldRender();
  }
  scheduleRender();
});

document.querySelectorAll("[data-mode]").forEach((button) => {
  button.addEventListener("click", () => {
    selectedMode = button.dataset.mode;
    enabled = false;
    updateButtons();
  });
});
document.querySelectorAll("[data-alliance]").forEach((button) => {
  button.addEventListener("click", () => {
    selectedAlliance = button.dataset.alliance;
    enabled = false;
    updateButtons();
  });
});
document.querySelector("#enable").addEventListener("click", () => { enabled = true; updateButtons(); });
document.querySelector("#disable").addEventListener("click", () => { enabled = false; updateButtons(); });
filterElement.addEventListener("input", scheduleRender);
autoSelect.addEventListener("change", () => {
  nt.publish(`${autoPrefix}selected`, "string", autoSelect.value);
});
cameraHostInput.addEventListener("change", () => {
  localStorage.setItem("photonvision-host", cameraHostInput.value.trim());
  updateCameraStreams(true);
});
teamNumberInput.addEventListener("input", () => {
  localStorage.setItem("frc-team-number", teamNumberInput.value.trim());
  connectionAvailability.clear();
  renderConnectionPoints();
  clearTimeout(teamDiscoveryTimer);
  teamDiscoveryTimer = setTimeout(discoverRobots, 300);
});
teamNumberInput.addEventListener("keydown", (event) => {
  if (event.key === "Enter") discoverRobots();
});
document.querySelectorAll("[data-tab]").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll("[data-tab]").forEach((tab) => tab.classList.toggle("selected", tab === button));
    document.querySelectorAll(".tab-panel").forEach((panel) => { panel.hidden = panel.id !== `${button.dataset.tab}-panel`; });
    updateCameraStreams();
  });
});

function updateButtons() {
  document.querySelectorAll("[data-mode]").forEach((button) => button.classList.toggle("selected", button.dataset.mode === selectedMode));
  document.querySelectorAll("[data-alliance]").forEach((button) => button.classList.toggle("selected", button.dataset.alliance === selectedAlliance));
  ds.classList.toggle("robot-enabled", enabled);
  document.querySelector("#enable").classList.toggle("active", enabled);
}

function scheduleRender() {
  if (renderPending) return;
  renderPending = true;
  setTimeout(renderTopics, 100);
}

function renderTopics() {
  renderPending = false;
  renderAutoChooser();
  renderDiagnostics();
  updateCameraStreams();
  const filter = filterElement.value.toLowerCase();
  const topics = [...values.values()].filter((topic) => topic.name.toLowerCase().includes(filter)).sort((a, b) => a.name.localeCompare(b.name));
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
  const usableWidth = canvasWidth - padding * 2;
  const usableHeight = canvasHeight - padding * 2;
  const scale = Math.min(usableWidth / field.width, usableHeight / field.length);
  const drawnWidth = field.width * scale;
  const drawnHeight = field.length * scale;
  const left = (canvasWidth - drawnWidth) / 2;
  const top = (canvasHeight - drawnHeight) / 2;
  const red = values.get("/FMSInfo/IsRedAlliance")?.value === true;

  context.fillStyle = "#18251d";
  context.strokeStyle = "#758274";
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
  context.fillStyle = red ? "#b52c3c" : "#2879cf";
  context.fillRect(left, top + drawnHeight - 7, drawnWidth, 7);
  context.fillStyle = red ? "#2879cf" : "#b52c3c";
  context.fillRect(left, top, drawnWidth, 7);

  const pose = decodePose2d(values.get("/AdvantageKit/RealOutputs/Odometry/Robot")?.value);
  fieldAlliance.textContent = `${red ? "Red" : "Blue"} alliance · station at bottom`;
  fieldAlliance.className = `field-alliance ${red ? "red" : "blue"}`;
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

new ResizeObserver(scheduleFieldRender).observe(fieldView);

function updateCameraStreams(force = false) {
  const panelOpen = !document.querySelector("#cameras-panel").hidden;
  const simulation = values.get("/SimSupervisor/Available")?.value === true;
  const cameraHost = simulation ? connectedHost : cameraHostInput.value.trim();
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

function connectionCandidates(teamNumber) {
  const team = Number.parseInt(teamNumber, 10);
  const candidates = [
    { id: "sim", host: "localhost", label: "localhost" },
  ];
  if (Number.isInteger(team) && team > 0 && team <= 99999) {
    candidates.push(
      { id: "team-ip", host: `10.${Math.floor(team / 100)}.${team % 100}.2`, label: `10.${Math.floor(team / 100)}.${team % 100}.2` },
      { id: "mdns", host: `roborio-${team}-frc.local`, label: `roborio-${team}-frc.local` },
    );
  }
  if (!candidates.some((candidate) => candidate.host === pageHost)) {
    candidates.push({ id: "page", host: pageHost, label: pageHost });
  }
  return candidates;
}

function renderConnectionPoints() {
  const candidates = connectionCandidates(teamNumberInput.value.trim());
  connectionPoints.replaceChildren(...candidates.map((candidate) => {
    const pill = document.createElement("button");
    const selected = candidate.id === activeConnectionId;
    const connected = ntConnected && selected;
    const unavailable = selected && !ntConnected;
    pill.className = `connection-pill${connectionAvailability.get(candidate.host) ? " available" : ""}${connected ? " connected" : ""}${unavailable ? " unavailable" : ""}`;
    pill.textContent = candidate.label;
    pill.title = candidate.host;
    pill.addEventListener("click", () => connectToRobot(candidate.host, candidate.id));
    return pill;
  }));
}

async function discoverRobots() {
  if (discoveryPromise) return discoveryPromise;
  discoveryPromise = (async () => {
    const teamNumber = teamNumberInput.value.trim();
    localStorage.setItem("frc-team-number", teamNumber);
    const candidates = connectionCandidates(teamNumber);
    const results = await Promise.all(candidates.map(async (candidate) => ({
      ...candidate,
      available: await probeNT4(candidate.host, 800),
    })));
    results.forEach((candidate) => connectionAvailability.set(candidate.host, candidate.available));
    const available = results.filter((candidate) => candidate.available);
    renderConnectionPoints();
    if (!ntConnected && !available.some((candidate) => candidate.host === connectedHost) && available.length) {
      connectToRobot(available[0].host, available[0].id);
    }
  })().finally(() => {
    discoveryPromise = null;
  });
  return discoveryPromise;
}

function connectToRobot(host, connectionId = "manual") {
  if (!host) return;
  activeConnectionId = connectionId;
  if (host === connectedHost) {
    renderConnectionPoints();
    return;
  }
  connectedHost = host;
  values.clear();
  enabled = false;
  updateButtons();
  renderConnectionPoints();
  nt.setHost(host);
  scheduleRender();
  scheduleFieldRender();
}

function renderAutoChooser() {
  const options = values.get(`${autoPrefix}options`)?.value;
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
  const selected = values.get(`${autoPrefix}selected`)?.value;
  const active = values.get(`${autoPrefix}active`)?.value;
  const defaultOption = values.get(`${autoPrefix}default`)?.value;
  const current = selected || active || defaultOption;
  if (typeof current === "string" && options.includes(current) && document.activeElement !== autoSelect) {
    autoSelect.value = current;
  }
  autoSelect.disabled = false;
}

function renderDiagnostics() {
  const devices = [...values.values()]
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
  if (extras.length) {
    const extraGrid = document.createElement("div");
    extraGrid.className = "device-grid orientation-extras";
    extraGrid.append(...extras.map(createDeviceCard));
    section.append(extraGrid);
  }
  return section;
}

function createUnregisteredCard() {
  const card = document.createElement("div");
  card.className = "device unregistered";
  card.textContent = "Not registered";
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

async function readDashboardSource() {
  const contents = await Promise.all(watchedAssets.map(async (path) => {
    const response = await fetch(`${path}?source-check=${Date.now()}`, { cache: "no-store" });
    if (!response.ok) throw new Error(`${path}: HTTP ${response.status}`);
    return response.text();
  }));
  return contents.join("\n---dashboard-asset---\n");
}

async function watchDashboardSource() {
  let previous;
  try {
    previous = await readDashboardSource();
  } catch {
    setTimeout(watchDashboardSource, 1000);
    return;
  }
  setInterval(async () => {
    try {
      const current = await readDashboardSource();
      if (current !== previous) {
        enabled = false;
        location.replace(`${location.pathname}?v=${Date.now()}`);
      }
    } catch {
      // The server disappears briefly while HALSim restarts. Retry next tick.
    }
  }, 1000);
}

async function startOfflineUpdates() {
  if (!("serviceWorker" in navigator) || !window.isSecureContext) {
    watchDashboardSource();
    return;
  }
  try {
    await navigator.serviceWorker.register("./service-worker.js", { scope: "./" });
    const registration = await navigator.serviceWorker.ready;
    navigator.serviceWorker.addEventListener("message", (event) => {
      if (event.data?.type === "SNAPSHOT_STATUS") {
        const loadedVersion = new URLSearchParams(location.search).get("v");
        if (event.data.version && loadedVersion !== event.data.version) {
          enabled = false;
          location.replace(`${location.pathname}?v=${event.data.version}`);
        }
        return;
      }
      if (event.data?.type !== "SNAPSHOT_UPDATED") return;
      enabled = false;
      location.replace(`${location.pathname}?v=${event.data.version}`);
    });
    const check = () => (registration.active ?? navigator.serviceWorker.controller)?.postMessage({ type: "CHECK_UPDATE" });
    check();
    setInterval(check, 3000);
  } catch {
    watchDashboardSource();
  }
}

function formatValue(value) {
  if (value instanceof Uint8Array) return `<${value.length} bytes>`;
  if (Array.isArray(value)) return JSON.stringify(value);
  if (typeof value === "number") return Number.isInteger(value) ? String(value) : value.toFixed(4);
  return String(value);
}

function createGamepadView(port) {
  const monitor = document.createElement("div");
  monitor.className = "gamepad-monitor";
  monitor.innerHTML = `<div class="gamepad-title"><strong>Gamepad ${port + 1}</strong><span>No controller</span></div><div class="gamepad-axes"></div><div class="gamepad-digital"><div class="gamepad-buttons"></div><div class="pov-readout"><span>POV</span><strong>—</strong></div></div>`;
  const axesContainer = monitor.querySelector(".gamepad-axes");
  const buttonsContainer = monitor.querySelector(".gamepad-buttons");
  const axes = axisNames.map((label) => {
    const element = document.createElement("div");
    element.className = "axis";
    element.innerHTML = `<span>${label}</span><span class="axis-track"><span class="axis-fill"></span></span><span class="axis-value">0.00</span>`;
    axesContainer.append(element);
    return element;
  });
  const buttons = buttonNames.map((label) => {
    const element = document.createElement("div");
    element.className = "gamepad-button";
    element.textContent = label;
    buttonsContainer.append(element);
    return element;
  });
  gamepadsElement.append(monitor);
  return { port, name: monitor.querySelector(".gamepad-title span"), axes, buttons, pov: monitor.querySelector(".pov-readout strong") };
}

function readGamepad(gamepad) {
  if (!gamepad) {
    let name = "No gamepad — press a button";
    if (!("getGamepads" in navigator)) name = "Gamepad API unavailable";
    else if (!window.isSecureContext) name = "Gamepad blocked: open localhost";
    return { connected: false, name, axes: [], buttons: [], pov: -1 };
  }
  const pressed = gamepad.buttons.map((button) => button.pressed);
  let pov = -1;
  if (pressed[12]) pov = 0;
  else if (pressed[15]) pov = 90;
  else if (pressed[13]) pov = 180;
  else if (pressed[14]) pov = 270;
  return {
    connected: true,
    name: gamepad.id,
    axes: [gamepad.axes[0] ?? 0, gamepad.axes[1] ?? 0, gamepad.buttons[6]?.value ?? 0,
      gamepad.buttons[7]?.value ?? 0, gamepad.axes[2] ?? 0, gamepad.axes[3] ?? 0],
    // Browser standard: A B X Y LB RB LT RT Back Start LS RS.
    // WPILib Xbox buttons omit the triggers: A B X Y LB RB Back Start LS RS.
    buttons: [pressed[0], pressed[1], pressed[2], pressed[3], pressed[4], pressed[5],
      pressed[8], pressed[9], pressed[10], pressed[11]],
    pov,
  };
}

function renderGamepad(view, gamepad) {
  view.name.textContent = gamepad.name;
  view.axes.forEach((element, index) => {
    const value = gamepad.axes[index] ?? 0;
    const fill = element.querySelector(".axis-fill");
    const trigger = index === 2 || index === 3;
    const normalized = trigger ? value : (value + 1) / 2;
    fill.style.left = trigger ? "0" : `${Math.min(normalized, 0.5) * 100}%`;
    fill.style.width = trigger ? `${normalized * 100}%` : `${Math.abs(value) * 50}%`;
    element.querySelector(".axis-value").textContent = value.toFixed(2);
  });
  view.buttons.forEach((element, index) => element.classList.toggle("pressed", gamepad.buttons[index] === true));
  view.pov.textContent = gamepad.pov < 0 ? "—" : `${gamepad.pov}°`;
}

function applyGamepadModeShortcut(gamepad, port) {
  const armed = gamepad.connected && gamepad.buttons[7] === true;
  const newlyPressed = armed && gamepad.pov >= 0 && gamepad.pov !== previousShortcutPovs[port];
  if (newlyPressed && (gamepad.pov === 0 || gamepad.pov === 180) && !autoSelect.disabled && autoSelect.options.length) {
    const step = gamepad.pov === 0 ? -1 : 1;
    const nextIndex = (autoSelect.selectedIndex + step + autoSelect.options.length) % autoSelect.options.length;
    autoSelect.selectedIndex = nextIndex;
    nt.publish(`${autoPrefix}selected`, "string", autoSelect.value);
  } else if (newlyPressed && gamepad.pov === 270) {
    const alreadyRunning = enabled && selectedMode === "auto";
    selectedMode = "auto";
    enabled = !alreadyRunning;
    updateButtons();
  } else if (newlyPressed && gamepad.pov === 90) {
    const alreadyRunning = enabled && selectedMode === "teleop";
    selectedMode = "teleop";
    enabled = !alreadyRunning;
    updateButtons();
  }
  previousShortcutPovs[port] = armed ? gamepad.pov : -1;
}

setInterval(() => {
  const simulationAvailable = ntConnected && values.get("/SimSupervisor/Available")?.value === true;
  ds.hidden = !simulationAvailable;
  dsPlaceholder.hidden = simulationAvailable;
  if (!simulationAvailable) return;

  const browserGamepads = [...(navigator.getGamepads?.() ?? [])].filter((gamepad) => gamepad?.connected).slice(0, 2);
  const gamepads = gamepadViews.map((view, index) => readGamepad(browserGamepads[index]));
  gamepads.forEach(applyGamepadModeShortcut);
  const connectedCount = gamepads.filter((gamepad) => gamepad.connected).length;
  gamepadState.textContent = `${connectedCount} gamepad${connectedCount === 1 ? "" : "s"}`;
  nt.publish("/SimSupervisor/Heartbeat", "int", ++heartbeat);
  nt.publish("/SimSupervisor/Mode", "string", enabled ? selectedMode : "disabled");
  nt.publish("/SimSupervisor/Alliance", "string", selectedAlliance);
  gamepads.forEach((gamepad, port) => {
    renderGamepad(gamepadViews[port], gamepad);
    nt.publish(`/SimSupervisor/Joystick${port}/Connected`, "boolean", gamepad.connected);
    nt.publish(`/SimSupervisor/Joystick${port}/Name`, "string", gamepad.name);
    nt.publish(`/SimSupervisor/Joystick${port}/Axes`, "double[]", gamepad.axes);
    nt.publish(`/SimSupervisor/Joystick${port}/Buttons`, "boolean[]", gamepad.buttons);
    nt.publish(`/SimSupervisor/Joystick${port}/POV`, "int", gamepad.pov);
  });
}, 20);

updateButtons();
renderTopics();
nt.connect();
discoverRobots();
setInterval(() => { if (!ntConnected) discoverRobots(); }, 1000);
startOfflineUpdates();
