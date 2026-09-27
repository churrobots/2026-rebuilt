/** @typedef {import("./core/dashboard-core.js").DashboardCore} DashboardCore */
/** @typedef {import("./core/dashboard-core.js").DashboardTopic} DashboardTopic */

const ALLIANCE_TOPIC = "/FMSInfo/IsRedAlliance";
const FAULT_PREFIX = "/SmartDashboard/HardwareMonitor/FaultStatus/";

/** A fresh, editable dashboard for alliance status and mechanism faults. */
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
      const version = new URL(import.meta.url).searchParams.get("v") ?? "initial";
      root.innerHTML = `<link rel="stylesheet" href="custom-dashboard.css?v=${version}">
        <main>
          <section class="card alliance-card" aria-labelledby="alliance-heading">
            <p class="eyebrow">Match status</p>
            <h1 id="alliance-heading">Current alliance</h1>
            <output class="alliance" aria-live="polite">Waiting for alliance…</output>
          </section>
          <section class="card faults-card" aria-labelledby="faults-heading">
            <div class="section-heading">
              <div><p class="eyebrow">Hardware monitor</p><h2 id="faults-heading">Mechanism faults</h2></div>
              <span class="fault-count">Waiting…</span>
            </div>
            <div class="faults" aria-live="polite"></div>
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
    if (!this.#started && this.isConnected && this.shadowRoot?.hasChildNodes()) this.#start(core);
  }

  /** @returns {DashboardCore | undefined} */
  get core() { return this.#core; }

  /** @param {DashboardCore} core The shared protected core API. */
  #start(core) {
    this.#started = true;
    const root = this.shadowRoot;
    const alliance = root.querySelector(".alliance");
    const faults = root.querySelector(".faults");
    const faultCount = root.querySelector(".fault-count");

    const render = () => {
      const isRed = core.getTopic(ALLIANCE_TOPIC)?.value;
      alliance.textContent = isRed === true ? "Red" : isRed === false ? "Blue" : "Waiting for alliance…";
      alliance.dataset.alliance = isRed === true ? "red" : isRed === false ? "blue" : "unknown";

      const mechanisms = core.getTopics().filter(isMechanismTopic);
      const broken = mechanisms.filter((topic) => topic.value === false);
      faultCount.textContent = mechanisms.length === 0 ? "Waiting…" : broken.length === 0 ? "All clear" : `${broken.length} fault${broken.length === 1 ? "" : "s"}`;
      faults.replaceChildren(...createFaultContent(broken, mechanisms.length));
    };

    /** @param {CustomEvent<DashboardTopic>} event */
    const topicListener = ({ detail: topic }) => {
      if (topic.name === ALLIANCE_TOPIC || topic.name.startsWith(FAULT_PREFIX)) render();
    };
    core.addEventListener("topic", topicListener);
    this.#cleanup.push(() => core.removeEventListener("topic", topicListener));
    render();
  }
}

/** @param {DashboardTopic} topic */
function isMechanismTopic(topic) {
  if (!topic.name.startsWith(FAULT_PREFIX) || typeof topic.value !== "boolean") return false;
  const name = topic.name.slice(FAULT_PREFIX.length).toLowerCase();
  return !["camera", "vision", "limelight", "pigeon", "gyro"].some((part) => name.includes(part))
    && !name.endsWith("drive") && !name.endsWith("turn");
}

/** @param {DashboardTopic[]} faults @param {number} mechanismCount */
function createFaultContent(faults, mechanismCount) {
  if (mechanismCount === 0) return [makeMessage("Waiting for HardwareMonitor data…")];
  if (faults.length === 0) return [makeMessage("No mechanism faults reported.", "healthy")];
  return faults.sort((a, b) => a.name.localeCompare(b.name)).map((topic) => {
    const item = document.createElement("div");
    item.className = "fault";
    item.textContent = humanize(topic.name.slice(FAULT_PREFIX.length));
    return item;
  });
}

/** @param {string} text @param {string} [kind] */
function makeMessage(text, kind = "") {
  const message = document.createElement("p");
  message.className = `message ${kind}`;
  message.textContent = text;
  return message;
}

/** @param {string} name */
function humanize(name) {
  return name.replace(/([a-z0-9])([A-Z])/g, "$1 $2").replace(/[_-]+/g, " ");
}

customElements.define("custom-dashboard", CustomDashboard);
