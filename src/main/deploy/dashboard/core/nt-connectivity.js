/** @typedef {import("./dashboard-core.js").DashboardCore} DashboardCore */

// Wait this long before showing the banner, so it doesn't flash while the page reconnects
// after a reload or the simulator restarts after a save.
const SHOW_AFTER_MS = 1000;

/**
 * Shows a banner only while NetworkTables is disconnected. The dashboard always connects to
 * the robot program that served this page, so there's nothing to pick.
 */
export class NtConnectivity extends HTMLElement {
  /** @type {DashboardCore | null} */
  #core = null;
  /** @type {ReturnType<typeof setTimeout> | undefined} */
  #showTimer;

  constructor() {
    super();
    this.attachShadow({ mode: "open" });
    const version = new URL(import.meta.url).searchParams.get("v") ?? "initial";
    this.shadowRoot.innerHTML = `<link rel="stylesheet" href="core/nt-connectivity.css?v=${version}">
      <p id="sim" role="status" hidden>
        <strong>The simulator isn't running.</strong>
        In VS Code, click <kbd>ChurroSim</kbd> to start it.
        <small>Just saved a file? Wait a few seconds while it restarts.</small>
      </p>
      <p id="robot" role="status" hidden>
        <strong>Can't reach the robot.</strong>
        <span id="robot-detail"></span>
        <small>Showing the last values received until it reconnects.</small>
      </p>`;
    this.hidden = true;
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
    clearTimeout(this.#showTimer);
    if (!state || state.connected) {
      this.hidden = true;
      return;
    }
    this.#showTimer = setTimeout(() => {
      this.shadowRoot.querySelector("#sim").hidden = !state.isSimulation;
      this.shadowRoot.querySelector("#robot").hidden = state.isSimulation;
      this.shadowRoot.querySelector("#robot-detail").textContent =
        `Is it on, is the robot code running, and is this laptop on its network (${state.host})?`;
      this.hidden = false;
    }, SHOW_AFTER_MS);
  }
}

customElements.define("nt-connectivity", NtConnectivity);
