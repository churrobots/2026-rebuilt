import { NT4Client } from "./networktables.js";
import "./nt-connectivity.js";
import "./sim-driver-station.js";
import { startDashboardRuntime } from "./dashboard-runtime.js";

/**
 * Protected coordination layer for core dashboard services.
 * UI components consume this narrow interface instead of sharing page globals.
 */
export class DashboardCore extends EventTarget {
  #nt;
  #values = new Map();
  #state;

  constructor(host = location.hostname || "localhost") {
    super();
    this.#state = {
      host,
      id: host === "localhost" ? "sim" : "page",
      connected: false,
    };
    this.#nt = new NT4Client(host);
    this.#nt.addEventListener("connected", () => this.#setConnection(true));
    this.#nt.addEventListener("disconnected", () => this.#setConnection(false));
    this.#nt.addEventListener("value", ({ detail: topic }) => {
      this.#values.set(topic.name, topic);
      this.dispatchEvent(new CustomEvent("topic", { detail: topic }));
    });
  }

  connect() {
    this.#nt.connect();
  }

  setHost(host, id = "manual") {
    if (!host) return;
    this.#state = { ...this.#state, host, id, connected: false };
    this.#values.clear();
    this.#nt.setHost(host);
    this.#emitConnection();
  }

  get connection() {
    return { ...this.#state };
  }

  get values() {
    return this.#values;
  }

  publish(name, type, value) {
    this.#nt.publish(name, type, value);
  }

  #setConnection(connected) {
    this.#state = { ...this.#state, connected };
    this.#emitConnection();
  }

  #emitConnection() {
    this.dispatchEvent(new CustomEvent("connection", { detail: this.connection }));
  }
}

/** Mounts and coordinates the protected dashboard infrastructure. */
export class DashboardCoreElement extends HTMLElement {
  #core = new DashboardCore();

  constructor() {
    super();
    const customDashboard = this.querySelector("custom-dashboard");
    const connectivity = document.createElement("nt-connectivity");
    const driverStation = document.createElement("sim-driver-station");
    connectivity.core = this.#core;
    driverStation.core = this.#core;
    connectivity.addEventListener("connection-request", ({ detail }) => this.#core.setHost(detail.host, detail.id));
    this.style.cssText = "flex: 1; min-height: 0; display: flex; flex-direction: column;";
    this.replaceChildren(driverStation, connectivity, ...(customDashboard ? [customDashboard] : []));
  }

  get core() { return this.#core; }
  connectedCallback() {
    this.#core.connect();
    startDashboardRuntime();
    customElements.whenDefined("custom-dashboard").then(() => {
      this.querySelector("custom-dashboard")?.core = this.#core;
    });
  }
}

customElements.define("dashboard-core", DashboardCoreElement);
