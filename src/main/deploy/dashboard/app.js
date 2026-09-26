const assetVersion = new URL(import.meta.url).searchParams.get("v") ?? "initial";
const { NT4Client } = await import(`./nt4.js?v=${assetVersion}`);

const host = location.hostname || "localhost";
const nt = new NT4Client(host);
const values = new Map();
const ntState = document.querySelector("#nt-state");
const ds = document.querySelector("#driver-station");
const topicsElement = document.querySelector("#topics");
const filterElement = document.querySelector("#topic-filter");
const diagnosticsElement = document.querySelector("#diagnostics");
const diagnosticSummary = document.querySelector("#diagnostic-summary");
const appVersion = document.querySelector("#app-version");
const autoSelect = document.querySelector("#auto-selector");
const gamepadState = document.querySelector("#gamepad-state");
const gamepadAxes = document.querySelector("#gamepad-axes");
const gamepadButtons = document.querySelector("#gamepad-buttons");
const gamepadPov = document.querySelector("#gamepad-pov");
const axisNames = ["LX", "LY", "LT", "RT", "RX", "RY"];
const buttonNames = ["A", "B", "X", "Y", "LB", "RB", "Back", "Start", "LS", "RS"];
const diagnosticPrefix = "/SmartDashboard/HardwareMonitor/FaultStatus/";
const autoPrefix = "/SmartDashboard/Auto Choices/";
const watchedAssets = ["index.html", "style.css", "app.js", "nt4.js", "msgpack.js", "manifest.webmanifest", "icon.svg"];
let selectedMode = "teleop";
let enabled = false;
let heartbeat = 0;
let renderPending = false;
let ntConnected = false;
let lastGamepadEvent = null;

window.addEventListener("gamepadconnected", (event) => {
  lastGamepadEvent = event.gamepad;
  gamepadState.textContent = event.gamepad.id;
});
window.addEventListener("gamepaddisconnected", (event) => {
  if (lastGamepadEvent?.index === event.gamepad.index) lastGamepadEvent = null;
});

const axisElements = axisNames.map((label) => {
  const element = document.createElement("div");
  element.className = "axis";
  element.innerHTML = `<span>${label}</span><span class="axis-track"><span class="axis-fill"></span></span><span class="axis-value">0.00</span>`;
  gamepadAxes.append(element);
  return element;
});
const buttonElements = buttonNames.map((label) => {
  const element = document.createElement("div");
  element.className = "gamepad-button";
  element.textContent = label;
  gamepadButtons.append(element);
  return element;
});

nt.addEventListener("connected", () => {
  ntConnected = true;
  ntState.textContent = `NT connected · ${host}`;
  ntState.classList.add("connected");
});
nt.addEventListener("disconnected", () => {
  ntConnected = false;
  ntState.textContent = "NT disconnected";
  ntState.classList.remove("connected");
  enabled = false;
  updateButtons();
});
nt.addEventListener("value", ({ detail: topic }) => {
  values.set(topic.name, topic);
  scheduleRender();
});

document.querySelectorAll("[data-mode]").forEach((button) => {
  button.addEventListener("click", () => {
    selectedMode = button.dataset.mode;
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
document.querySelectorAll("[data-tab]").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll("[data-tab]").forEach((tab) => tab.classList.toggle("selected", tab === button));
    document.querySelectorAll(".tab-panel").forEach((panel) => { panel.hidden = panel.id !== `${button.dataset.tab}-panel`; });
  });
});

function updateButtons() {
  document.querySelectorAll("[data-mode]").forEach((button) => button.classList.toggle("selected", button.dataset.mode === selectedMode));
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
  const groups = [
    ["Vision", devices.filter((device) => classifyDevice(device.name) === "vision")],
    ["Mechanisms", devices.filter((device) => classifyDevice(device.name) === "mechanisms")],
    ["Drivetrain", devices.filter((device) => classifyDevice(device.name) === "drivetrain")],
  ];
  diagnosticsElement.replaceChildren(...groups.map(([label, groupDevices]) => {
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
  }));
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
    appVersion.textContent = "Web · offline unavailable";
    watchDashboardSource();
    return;
  }
  try {
    await navigator.serviceWorker.register("./service-worker.js", { scope: "./" });
    const registration = await navigator.serviceWorker.ready;
    appVersion.textContent = "PWA · cached";
    navigator.serviceWorker.addEventListener("message", (event) => {
      if (event.data?.type === "SNAPSHOT_STATUS") {
        appVersion.textContent = `Known good · ${event.data.version}`;
        return;
      }
      if (event.data?.type !== "SNAPSHOT_UPDATED") return;
      appVersion.textContent = `Updating · ${event.data.version}`;
      location.replace(`${location.pathname}?v=${event.data.version}`);
    });
    const check = () => (registration.active ?? navigator.serviceWorker.controller)?.postMessage({ type: "CHECK_UPDATE" });
    check();
    setInterval(check, 3000);
  } catch {
    appVersion.textContent = "Web · cache failed";
    watchDashboardSource();
  }
}

function formatValue(value) {
  if (value instanceof Uint8Array) return `<${value.length} bytes>`;
  if (Array.isArray(value)) return JSON.stringify(value);
  if (typeof value === "number") return Number.isInteger(value) ? String(value) : value.toFixed(4);
  return String(value);
}

function readGamepad() {
  const gamepad = [...(navigator.getGamepads?.() ?? [])].find((candidate) => candidate?.connected)
    ?? (lastGamepadEvent?.connected ? lastGamepadEvent : null);
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

function renderGamepad(gamepad) {
  axisElements.forEach((element, index) => {
    const value = gamepad.axes[index] ?? 0;
    const fill = element.querySelector(".axis-fill");
    const trigger = index === 2 || index === 3;
    const normalized = trigger ? value : (value + 1) / 2;
    fill.style.left = trigger ? "0" : `${Math.min(normalized, 0.5) * 100}%`;
    fill.style.width = trigger ? `${normalized * 100}%` : `${Math.abs(value) * 50}%`;
    element.querySelector(".axis-value").textContent = value.toFixed(2);
  });
  buttonElements.forEach((element, index) => element.classList.toggle("pressed", gamepad.buttons[index] === true));
  gamepadPov.textContent = gamepad.pov < 0 ? "—" : `${gamepad.pov}°`;
}

setInterval(() => {
  const simulationAvailable = ntConnected && values.get("/SimSupervisor/Available")?.value === true;
  ds.hidden = !simulationAvailable;
  if (!simulationAvailable) return;

  const gamepad = readGamepad();
  gamepadState.textContent = gamepad.connected ? gamepad.name : "No gamepad";
  renderGamepad(gamepad);
  nt.publish("/SimSupervisor/Heartbeat", "int", ++heartbeat);
  nt.publish("/SimSupervisor/Mode", "string", enabled ? selectedMode : "disabled");
  nt.publish("/SimSupervisor/Joystick0/Connected", "boolean", gamepad.connected);
  nt.publish("/SimSupervisor/Joystick0/Name", "string", gamepad.name);
  nt.publish("/SimSupervisor/Joystick0/Axes", "double[]", gamepad.axes);
  nt.publish("/SimSupervisor/Joystick0/Buttons", "boolean[]", gamepad.buttons);
  nt.publish("/SimSupervisor/Joystick0/POV", "int", gamepad.pov);
}, 20);

updateButtons();
renderTopics();
nt.connect();
startOfflineUpdates();
