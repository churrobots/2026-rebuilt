/** @typedef {import("./dashboard-core.js").DashboardCore} DashboardCore */

// Once connected, wait this long before swapping the cards for the banner after a
// disconnect, so a quick blip doesn't make everything flicker.
const SHOW_AFTER_MS = 1000;

/**
 * While NetworkTables is disconnected, hides the cards and shows "waiting for the robot to
 * boot" in their place, with tips for getting it going underneath. The dashboard always
 * connects to the robot program that served this page, so there's nothing to pick.
 */
export class NtConnectivity extends HTMLElement {
  /** @type {DashboardCore | null} */
  #core = null;
  /** @type {Array<ReturnType<typeof setTimeout>>} */
  #timers = [];
  /** Which server we're waiting on, so retries every second don't restart the timers. */
  /** @type {string | null} */
  #waitingFor = null;
  /** Whether we've ever connected; before that there's no blip to ride out. */
  #everConnected = false;
  /** The cards, hidden while we're disconnected. */
  /** @type {HTMLElement | null} */
  replaces = null;

  constructor() {
    super();
    this.attachShadow({ mode: "open" });
    const version = new URL(import.meta.url).searchParams.get("v") ?? "initial";
    this.shadowRoot.innerHTML = `<link rel="stylesheet" href="core/nt-connectivity.css?v=${version}">
      <p role="status">
        <strong id="waiting"></strong>
        <span id="help"></span>
      </p>`;
    // Don't set this.hidden (or any attribute) here: a custom element's constructor isn't
    // allowed to add attributes, and document.createElement would quietly swap this element
    // for an empty placeholder. render(), called once the core is set, shows or hides it.
  }

  /** @param {DashboardCore | null} core The protected shared core service. */
  set core(core) {
    this.#core = core;
    core?.addEventListener("connection", () => this.render());
    this.render();
  }

  /** @returns {DashboardCore | null} */
  get core() {
    return this.#core;
  }

  render() {
    const state = this.#core?.connection;
    if (state?.connected) this.#everConnected = true;
    if (!state || state.connected) {
      this.#timers.forEach(clearTimeout);
      this.#timers = [];
      this.#waitingFor = null;
      this.#show(false);
      return;
    }
    // The connection retries every second and reports "disconnected" each time; keep the
    // banner and its timers going instead of starting over.
    if (this.#waitingFor === state.host) return;
    this.#waitingFor = state.host;
    this.#timers.forEach(clearTimeout);
    this.#timers = [];

    const root = this.shadowRoot;
    const help = root.querySelector("#help");
    const keys = /Mac|iPhone|iPad/.test(navigator.platform) ? "⌘ Cmd + ⇧ Shift + B" : "Ctrl + Shift + B";
    root.querySelector("p").className = state.isSimulation ? "sim" : "real";
    root.querySelector("#waiting").textContent = state.isSimulation
      ? "Waiting for the simulated robot to boot…"
      : `Waiting for the real robot to boot (${state.host})…`;
    help.innerHTML = state.isSimulation
      ? `Make sure the simulator is running: in VS Code, press <kbd>${keys}</kbd> and pick
         <b>ChurroBots: Develop and Simulate</b>.
         <small>Just saved a file? It takes a few seconds to rebuild and restart.</small>`
      : `Make sure the robot is on and booting up properly, and that this laptop is on the
         robot's network.`;

    // Before the first connection there's nothing to flicker, so swap right away.
    this.#timers.push(setTimeout(() => this.#show(true), this.#everConnected ? SHOW_AFTER_MS : 0));
  }

  /** Shows the banner in place of the cards, or the cards in place of the banner. */
  #show(waiting) {
    this.hidden = !waiting;
    // An inline style, since the custom dashboard's own :host display rule would beat the
    // hidden attribute.
    if (this.replaces) this.replaces.style.display = waiting ? "none" : "";
  }
}

customElements.define("nt-connectivity", NtConnectivity);
