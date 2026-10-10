/** @typedef {import("./dashboard-core.js").DashboardCore} DashboardCore */

// Only one open dashboard drives the robot at a time. Clicking in a dashboard's Driver
// Station claims it by publishing that dashboard's id here. The last click wins, and every
// other dashboard goes quiet. Otherwise an idle dashboard keeps sending "disabled" and the
// robot flickers between enabled and disabled.
const OWNER_TOPIC = "/SimSupervisor/Owner";
const HEARTBEAT_TOPIC = "/SimSupervisor/Heartbeat";
// If no dashboard has sent a heartbeat for this long, nobody is driving, so take over.
const TAKE_OVER_IDLE_MS = 2000;
// Two claims this close together are a tie (each dashboard hears the other's claim after
// its own), so the larger id wins. Otherwise both would step aside and nobody would drive.
const TIE_MS = 500;

/** A Shadow-DOM simulated Driver Station and `/SimSupervisor/*` publisher. */
export class SimDriverStation extends HTMLElement {
  /** @type {DashboardCore | undefined} */ #core;
  /** @type {"teleop" | "auto" | "test"} */ #mode = "teleop";
  /** @type {"red" | "blue"} */ #alliance =
    localStorage.getItem("sim-alliance") === "red" ? "red" : "blue";
  /** @type {boolean} */ #enabled = false;
  /** @type {number} */ #heartbeat = 0;
  /** @type {number | undefined} */ #timer;
  /** @type {string} */ #id = Math.random().toString(36).slice(2, 10);
  /** @type {boolean} */ #driving = false;
  /** @type {number} */ #claimedAt = 0;
  /** @type {boolean} */ #wasConnected = false;
  /** @type {unknown} */ #lastHeartbeatSeen;
  /** @type {number} */ #heartbeatChangedAt = performance.now();
  // A random extra wait, so two idle dashboards rarely take over at the same moment.
  /** @type {number} */ #takeOverIdleMs = TAKE_OVER_IDLE_MS + Math.random() * 1000;
  constructor() {
    super();
    const root = this.attachShadow({ mode: "open" });
    const version = new URL(import.meta.url).searchParams.get("v") ?? "initial";
    root.innerHTML = `<link rel="stylesheet" href="core/sim-driver-station.css?v=${version}">
      <div class="offline">Driver Station floats here</div>
      <div class="ds" hidden>
        <button class="take-over" hidden>Another dashboard is driving the robot. Click here to drive from this one.</button>
        <div class="controls">
          <b class="brand">SIM DRIVER STATION</b>
          <div class="modes">
            <button data-mode="teleop">Teleop</button>
            <button data-mode="auto">Autonomous</button>
            <button data-mode="test">Test</button>
          </div>
          <div class="alliances">
            <button data-alliance="blue">Blue</button>
            <button data-alliance="red">Red</button>
          </div>
          <button class="enable">Enable</button>
        </div>
        <div class="pads"></div>
      </div>`;
    root.querySelectorAll("[data-mode]").forEach(
      (b) =>
        (b.onclick = () => {
          this.#claim();
          this.#mode = b.dataset.mode;
          this.#enabled = false;
          this.#render();
        }),
    );
    root.querySelectorAll("[data-alliance]").forEach(
      (b) =>
        (b.onclick = () => {
          this.#claim();
          this.#alliance = b.dataset.alliance;
          localStorage.setItem("sim-alliance", this.#alliance);
          this.#enabled = false;
          this.#render();
        }),
    );
    root.querySelector(".take-over").onclick = () => {
      this.#claim();
      this.#render();
    };
    root.querySelector(".enable").onclick = () => {
      this.#claim();
      this.#enabled = !this.#enabled;
      this.#render();
    };
    [1, 2].forEach((number) =>
      root
        .querySelector(".pads")
        .insertAdjacentHTML(
          "beforeend",
          `<div class="pad"><b>Gamepad ${number}</b><span class="name">No controller</span><span class="values">Waiting…</span></div>`,
        ),
    );
    this.#render();
  }
  /** @param {DashboardCore} core The protected shared core service. */
  set core(core) {
    this.#core = core;
  }
  prepareForReload() {
    sessionStorage.setItem(
      "resume-teleop-after-dashboard-reload",
      this.#enabled && this.#mode === "teleop" ? "true" : "false",
    );
  }
  connectedCallback() {
    this.#timer ??= setInterval(() => this.#tick(), 20);
  }
  disconnectedCallback() {
    clearInterval(this.#timer);
    this.#timer = undefined;
  }
  #render() {
    const root = this.shadowRoot;
    root
      .querySelectorAll("[data-mode]")
      .forEach((b) =>
        b.classList.toggle("selected", b.dataset.mode === this.#mode),
      );
    root
      .querySelectorAll("[data-alliance]")
      .forEach((b) =>
        b.classList.toggle("selected", b.dataset.alliance === this.#alliance),
      );
    root.querySelector(".take-over").hidden = this.#driving;
    root.querySelector(".controls").classList.toggle("standby", !this.#driving);
    const b = root.querySelector(".enable");
    b.classList.toggle("enabled", this.#enabled);
    b.textContent = this.#enabled ? "Disable" : "Enable";
  }
  #read(gamepad) {
    if (!gamepad)
      return {
        connected: false,
        name: "No controller",
        axes: [],
        buttons: [],
        pov: -1,
      };
    const pressed = gamepad.buttons.map((b) => b.pressed);
    return {
      connected: true,
      name: gamepad.id,
      axes: [
        gamepad.axes[0] ?? 0,
        gamepad.axes[1] ?? 0,
        gamepad.buttons[6]?.value ?? 0,
        gamepad.buttons[7]?.value ?? 0,
        gamepad.axes[2] ?? 0,
        gamepad.axes[3] ?? 0,
      ],
      buttons: [
        pressed[0],
        pressed[1],
        pressed[2],
        pressed[3],
        pressed[4],
        pressed[5],
        pressed[8],
        pressed[9],
        pressed[10],
        pressed[11],
      ],
      pov: pressed[12]
        ? 0
        : pressed[15]
          ? 90
          : pressed[13]
            ? 180
            : pressed[14]
              ? 270
              : -1,
    };
  }
  #tick() {
    const available =
      this.#core?.connection.connected &&
      this.#core.getTopic("/SimSupervisor/Available")?.value === true;
    this.shadowRoot.querySelector(".ds").hidden = !available;
    this.shadowRoot.querySelector(".offline").hidden = available;
    if (!available) {
      this.#wasConnected = false;
      return;
    }
    const connectedGamepads = [...(navigator.getGamepads?.() ?? [])]
      .filter((gamepad) => gamepad?.connected)
      .slice(0, 2);
    const pads = [0, 1].map((port) => this.#read(connectedGamepads[port]));
    pads.forEach((pad, port) => this.#updatePad(port, pad));
    this.#checkDriver();
    if (!this.#driving) return;
    this.#core.publish(HEARTBEAT_TOPIC, "int", ++this.#heartbeat);
    this.#core.publish(
      "/SimSupervisor/Mode",
      "string",
      this.#enabled ? this.#mode : "disabled",
    );
    this.#core.publish("/SimSupervisor/Alliance", "string", this.#alliance);
    pads.forEach((pad, port) => this.#publish(port, pad));
  }
  /** Makes this dashboard the one that drives the robot. */
  #claim() {
    this.#driving = true;
    this.#claimedAt = performance.now();
    this.#core?.publish(OWNER_TOPIC, "string", this.#id);
  }
  /** Steps aside when another dashboard claims the robot, or takes over when nobody drives. */
  #checkDriver() {
    const now = performance.now();
    // A claim sent while disconnected is lost, so send it again after reconnecting.
    if (!this.#wasConnected && this.#driving) this.#claim();
    this.#wasConnected = true;
    if (this.#driving) {
      const owner = this.#core.getTopic(OWNER_TOPIC);
      const claimedAt = owner?.updated ?? 0;
      const someoneElseClaimed =
        typeof owner?.value === "string" &&
        owner.value !== this.#id &&
        claimedAt > this.#claimedAt &&
        (claimedAt - this.#claimedAt > TIE_MS || owner.value > this.#id);
      if (someoneElseClaimed) {
        this.#driving = false;
        this.#enabled = false;
        this.#heartbeatChangedAt = now;
        this.#render();
      }
      return;
    }
    const heartbeat = this.#core.getTopic(HEARTBEAT_TOPIC)?.value;
    if (heartbeat !== this.#lastHeartbeatSeen) {
      this.#lastHeartbeatSeen = heartbeat;
      this.#heartbeatChangedAt = now;
    } else if (now - this.#heartbeatChangedAt > this.#takeOverIdleMs) {
      this.#claim();
      this.#render();
    }
  }
  #updatePad(port, pad) {
    const element = this.shadowRoot.querySelectorAll(".pad")[port];
    element.classList.toggle("connected", pad.connected);
    element.querySelector(".name").textContent = pad.name;
    element.querySelector(".values").textContent = [
      "LX",
      "LY",
      "LT",
      "RT",
      "RX",
      "RY",
    ]
      .map((name, index) => `${name} ${(pad.axes[index] ?? 0).toFixed(2)}`)
      .join(" · ");
  }
  #publish(port, pad) {
    const prefix = `/SimSupervisor/Joystick${port}`;
    this.#core.publish(`${prefix}/Connected`, "boolean", pad.connected);
    this.#core.publish(`${prefix}/Name`, "string", pad.name);
    this.#core.publish(`${prefix}/Axes`, "double[]", pad.axes);
    this.#core.publish(`${prefix}/Buttons`, "boolean[]", pad.buttons);
    this.#core.publish(`${prefix}/POV`, "int", pad.pov);
  }
}
customElements.define("sim-driver-station", SimDriverStation);
