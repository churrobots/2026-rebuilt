import { NT4Client } from "./networktables.js";
import "./nt-connectivity.js";
import "./sim-driver-station.js";
import { startDashboardRuntime } from "./dashboard-runtime.js";

/**
 * A NetworkTables value most recently received by the dashboard.
 * @typedef {Object} DashboardTopic
 * @property {string} name
 * @property {unknown} value
 * @property {number} [updated]
 */

/**
 * The dashboard's current NetworkTables connection state.
 * @typedef {Object} DashboardConnection
 * @property {string} host
 * @property {string} id
 * @property {boolean} connected
 */

/**
 * NetworkTables types the dashboard can publish.
 * @typedef {"boolean" | "double" | "int" | "float" | "string" | "raw" | "boolean[]" | "double[]" | "int[]" | "float[]" | "string[]"} NetworkTableType
 */

/**
 * Protected coordination layer for core dashboard services.
 * UI components consume this narrow interface instead of sharing page globals.
 */
export class DashboardCore extends EventTarget {
  /** @type {NT4Client} */
  #nt;
  /** @type {Map<string, DashboardTopic>} */
  #values = new Map();
  /** @type {DashboardConnection} */
  #state;

  /** @param {string} [host] The robot or simulator hostname. */
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

  /** Opens the NetworkTables connection. */
  connect() {
    this.#nt.connect();
  }

  /**
   * Changes the NetworkTables server and clears values from the old server.
   * @param {string} host
   * @param {string} [id] A short label for the selected connection.
   */
  setHost(host, id = "manual") {
    if (!host) return;
    this.#state = { ...this.#state, host, id, connected: false };
    this.#values.clear();
    this.#nt.setHost(host);
    this.#emitConnection();
  }

  /** @returns {DashboardConnection} A copy that callers cannot use to change core state. */
  get connection() {
    return { ...this.#state };
  }

  /**
   * Values received from NetworkTables, keyed by topic name.
   * Treat this map as read-only; only core updates it.
   * @returns {Map<string, DashboardTopic>}
   */
  get values() {
    return this.#values;
  }

  /**
   * Publishes one value to NetworkTables.
   * @param {string} name
   * @param {NetworkTableType} type
   * @param {unknown} value
   */
  publish(name, type, value) {
    this.#nt.publish(name, type, value);
  }

  /** @param {boolean} connected */
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
  /** @type {DashboardCore} */
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

  /** @returns {DashboardCore} The shared core service for child components. */
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
