/** @typedef {import("./core/dashboard-core.js").DashboardCore} DashboardCore */
/** @typedef {import("./core/dashboard-core.js").DashboardTopic} DashboardTopic */

const ALLIANCE_TOPIC = "/FMSInfo/IsRedAlliance";

/** A fresh, editable dashboard. Add a card here for everything you want to see. */
export class CustomDashboard extends HTMLElement {
  /** @type {DashboardCore | undefined} */
  #core;
  /** @type {boolean} */
  #started = false;
  /** @type {Array<() => void>} */
  #cleanup = [];

  connectedCallback() {
    const root = this.shadowRoot ?? this.attachShadow({ mode: "open" });
    if (!root.hasChildNodes()) {
      const version =
        new URL(import.meta.url).searchParams.get("v") ?? "initial";
      root.innerHTML = `<link rel="stylesheet" href="custom-dashboard.css?v=${version}">
        <main>
          <section class="card alliance-card" aria-labelledby="alliance-heading">
            <p class="eyebrow">Match status</p>
            <h1 id="alliance-heading">Current alliance</h1>
            <output class="alliance" aria-live="polite">Waiting for alliance…</output>
          </section>
        </main>`;
    }
    if (this.#core && !this.#started) this.#start(this.#core);
  }

  disconnectedCallback() {
    this.#cleanup.forEach((cleanup) => cleanup());
    this.#cleanup = [];
    this.#started = false;
    this.shadowRoot?.replaceChildren();
  }

  /** @param {DashboardCore} core The shared protected core API. */
  set core(core) {
    this.#core = core;
    if (!this.#started && this.isConnected && this.shadowRoot?.hasChildNodes())
      this.#start(core);
  }

  /** @returns {DashboardCore | undefined} */
  get core() {
    return this.#core;
  }

  /** @param {DashboardCore} core The shared protected core API. */
  #start(core) {
    this.#started = true;
    const alliance = this.shadowRoot.querySelector(".alliance");

    const render = () => {
      const isRed = core.getTopic(ALLIANCE_TOPIC)?.value;
      alliance.textContent =
        isRed === true
          ? "Red"
          : isRed === false
            ? "Blue"
            : "Waiting for alliance…";
      alliance.dataset.alliance =
        isRed === true ? "red" : isRed === false ? "blue" : "unknown";
    };

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (topic.name === ALLIANCE_TOPIC) render();
    };
    core.addEventListener("topic", topicListener);
    this.#cleanup.push(() => core.removeEventListener("topic", topicListener));
    render();
  }
}

customElements.define("custom-dashboard", CustomDashboard);
