/** @typedef {import("./dashboard-core.js").DashboardCore} DashboardCore */

/**
 * One NetworkTables server the driver can select.
 * @typedef {Object} ConnectionCandidate
 * @property {string} id
 * @property {string} host
 * @property {string} label
 */

/** NetworkTables connection picker with fully isolated markup and styles. */
export class NtConnectivity extends HTMLElement {
  /** @type {DashboardCore | null} */
  #core = null;

  constructor() {
    super();
    this.attachShadow({ mode: "open" });
    const version = new URL(import.meta.url).searchParams.get("v") ?? "initial";
    this.shadowRoot.innerHTML = `<link rel="stylesheet" href="core/nt-connectivity.css?v=${version}">
      <section aria-label="NetworkTables connectivity">
        <strong>NetworkTables</strong>
        <input inputmode="numeric" aria-label="Team number" placeholder="Team #">
        <div id="points"></div>
      </section>`;
    this.shadowRoot.querySelector("input").value =
      localStorage.getItem("frc-team-number") || "8048";
    this.shadowRoot
      .querySelector("input")
      .addEventListener("change", () => this.render());
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
    const input = this.shadowRoot.querySelector("input");
    const team = Number.parseInt(input.value, 10);
    localStorage.setItem("frc-team-number", input.value.trim());
    /** @type {ConnectionCandidate[]} */
    const candidates = [{ id: "sim", host: "localhost", label: "Simulator" }];
    if (Number.isInteger(team) && team > 0 && team <= 99999)
      candidates.push(
        {
          id: "team-ip",
          host: `10.${Math.floor(team / 100)}.${team % 100}.2`,
          label: `10.${Math.floor(team / 100)}.${team % 100}.2`,
        },
        {
          id: "mdns",
          host: `roborio-${team}-frc.local`,
          label: `roborio-${team}-frc.local`,
        },
      );
    const state = this.#core?.connection;
    const points = this.shadowRoot.querySelector("#points");
    points.replaceChildren(
      ...candidates.map((candidate) => {
        const button = document.createElement("button");
        const selected = state?.host === candidate.host;
        button.className = selected
          ? state.connected
            ? "connected"
            : "offline"
          : "";
        button.textContent = candidate.label;
        button.addEventListener("click", () =>
          this.dispatchEvent(
            new CustomEvent("connection-request", {
              detail: candidate,
              bubbles: true,
              composed: true,
            }),
          ),
        );
        return button;
      }),
    );
  }
}

customElements.define("nt-connectivity", NtConnectivity);
