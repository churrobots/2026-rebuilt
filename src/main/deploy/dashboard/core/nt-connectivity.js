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
    this.shadowRoot.innerHTML = `<style>
      :host { display:block; padding:7px 18px; background:#10151e; border-block:1px solid #293142; color:#edf2f7; font:inherit }
      section { display:flex; align-items:center; gap:10px; min-height:36px } strong { font-size:.8rem }
      input { width:78px; padding:6px 8px; color:inherit; background:#0d1118; border:1px solid #293142; border-radius:7px }
      #points { display:flex; gap:6px } button { padding:5px 9px; color:#778294; background:#191e27; border:1px solid #343d4c; border-radius:99px; cursor:pointer; font-size:.72rem }
      button.connected { color:#a7f3c1; background:#173b27; border-color:#38a963 } button.offline { color:#fecaca; background:#421d24; border-color:#dc4c5c }
    </style><section aria-label="NetworkTables connectivity"><strong>NetworkTables</strong><input inputmode="numeric" aria-label="Team number" placeholder="Team #"><div id="points"></div></section>`;
    this.shadowRoot.querySelector("input").value = localStorage.getItem("frc-team-number") || "8048";
    this.shadowRoot.querySelector("input").addEventListener("change", () => this.render());
  }

  /** @param {DashboardCore | null} core The protected shared core service. */
  set core(core) {
    this.#core = core;
    core?.addEventListener("connection", () => this.render());
    this.render();
  }

  /** @returns {DashboardCore | null} */
  get core() { return this.#core; }

  render() {
    const input = this.shadowRoot.querySelector("input");
    const team = Number.parseInt(input.value, 10);
    localStorage.setItem("frc-team-number", input.value.trim());
    /** @type {ConnectionCandidate[]} */
    const candidates = [{ id:"sim", host:"localhost", label:"Simulator" }];
    if (Number.isInteger(team) && team > 0 && team <= 99999) candidates.push(
      { id:"team-ip", host:`10.${Math.floor(team / 100)}.${team % 100}.2`, label:`10.${Math.floor(team / 100)}.${team % 100}.2` },
      { id:"mdns", host:`roborio-${team}-frc.local`, label:`roborio-${team}-frc.local` });
    const state = this.#core?.connection;
    const points = this.shadowRoot.querySelector("#points");
    points.replaceChildren(...candidates.map((candidate) => {
      const button = document.createElement("button");
      const selected = state?.host === candidate.host;
      button.className = selected ? (state.connected ? "connected" : "offline") : "";
      button.textContent = candidate.label;
      button.addEventListener("click", () => this.dispatchEvent(new CustomEvent("connection-request", { detail:candidate, bubbles:true, composed:true })));
      return button;
    }));
  }
}

customElements.define("nt-connectivity", NtConnectivity);
