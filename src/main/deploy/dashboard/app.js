const assetVersion = new URL(import.meta.url).searchParams.get("v") ?? "initial";
await import(`./core/dashboard-core.js?v=${assetVersion}`);

const core = document.querySelector("dashboard-core").core;
const values = core.values;
const topicsElement = document.querySelector("#topics");
const filterElement = document.querySelector("#topic-filter");
const diagnosticsElement = document.querySelector("#diagnostics");
const diagnosticSummary = document.querySelector("#diagnostic-summary");
const autoSelect = document.querySelector("#auto-selector");
const camerasElement = document.querySelector("#cameras");
const cameraHostInput = document.querySelector("#camera-host");
const fieldCanvas = document.querySelector("#field-canvas");
const fieldView = document.querySelector("#field-view");
const fieldPose = document.querySelector("#field-pose");
const aimLockFrame = document.querySelector(".aim-lock-frame");
const diagnosticPrefix = "/SmartDashboard/HardwareMonitor/FaultStatus/";
const autoPrefix = "/SmartDashboard/Auto Choices/";
const aimLockTopic = "/SmartDashboard/aimLocked";
const watchedAssets = ["index.html", "index.css", "app.css", "app.js", "core/dashboard-core.js", "core/networktables.js", "core/nt-connectivity.js", "core/sim-driver-station.js", "core/msgpack.js", "core/icon.svg", "manifest.webmanifest"];
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
core.addEventListener("topic", ({ detail: topic }) => {
  if (topic.name === aimLockTopic) updateAimLock();
  if (topic.name === "/AdvantageKit/RealOutputs/Odometry/Robot" || topic.name === "/FMSInfo/IsRedAlliance" || topic.name === "/FMSInfo/StationNumber") {
    scheduleFieldRender();
  }
  scheduleRender();
});

filterElement.addEventListener("input", scheduleRender);
autoSelect.addEventListener("change", () => {
  core.publish(`${autoPrefix}selected`, "string", autoSelect.value);
});
cameraHostInput.addEventListener("change", () => {
  localStorage.setItem("photonvision-host", cameraHostInput.value.trim());
  updateCameraStreams(true);
});
document.querySelectorAll("[data-tab]").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll("[data-tab]").forEach((tab) => tab.classList.toggle("selected", tab === button));
    document.querySelectorAll(".tab-panel").forEach((panel) => { panel.hidden = panel.id !== `${button.dataset.tab}-panel`; });
    aimLockFrame.hidden = button.dataset.tab !== "diagnostics";
    updateCameraStreams();
  });
});

function updateAimLock() {
  aimLockFrame.classList.toggle("aim-locked", values.get(aimLockTopic)?.value === true);
}

function reloadDashboard(url) {
  document.querySelector("sim-driver-station").prepareForReload();
  location.replace(url);
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
  const allianceZoneHeight = 30;
  const allianceZoneGap = 5;
  const usableWidth = canvasWidth - padding * 2;
  const usableHeight = canvasHeight - padding * 2;
  const scale = Math.min(usableWidth / field.width, (usableHeight - allianceZoneHeight - allianceZoneGap) / field.length);
  const drawnWidth = field.width * scale;
  const drawnHeight = field.length * scale;
  const left = (canvasWidth - drawnWidth) / 2;
  const top = (canvasHeight - drawnHeight - allianceZoneGap - allianceZoneHeight) / 2;
  const red = values.get("/FMSInfo/IsRedAlliance")?.value === true;

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
  const stationValue = Number(values.get("/FMSInfo/StationNumber")?.value);
  const station = Number.isInteger(stationValue) && stationValue >= 1 && stationValue <= 3 ? stationValue : null;
  context.fillStyle = "#ffffff";
  context.font = "700 10px Inter, ui-sans-serif, system-ui, sans-serif";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(`YOUR ALLIANCE${station ? ` · STATION ${station}` : ""}`, left + drawnWidth / 2, allianceZoneTop + allianceZoneHeight / 2);

  const pose = decodePose2d(values.get("/AdvantageKit/RealOutputs/Odometry/Robot")?.value);
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
  const cameraHost = simulation ? core.connection.host : cameraHostInput.value.trim();
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
        reloadDashboard(`${location.pathname}?v=${Date.now()}`);
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
          reloadDashboard(`${location.pathname}?v=${event.data.version}`);
        }
        return;
      }
      if (event.data?.type !== "SNAPSHOT_UPDATED") return;
      reloadDashboard(`${location.pathname}?v=${event.data.version}`);
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

renderTopics();
startOfflineUpdates();
