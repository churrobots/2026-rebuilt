const watchedAssets = ["index.html", "index.css", "custom-dashboard.css", "custom-dashboard.js", "core/dashboard-core.js", "core/dashboard-runtime.js", "core/networktables.js", "core/nt-connectivity.js", "core/sim-driver-station.js", "core/msgpack.js", "core/icon.svg", "manifest.webmanifest"];
let started = false;

function reload(url) {
  document.querySelector("sim-driver-station")?.prepareForReload();
  location.replace(url);
}

async function source() {
  const contents = await Promise.all(watchedAssets.map(async (path) => {
    const response = await fetch(`${path}?source-check=${Date.now()}`, { cache: "no-store" });
    if (!response.ok) throw new Error(`${path}: HTTP ${response.status}`);
    return response.text();
  }));
  return contents.join("\n---dashboard-asset---\n");
}

async function watchSource() {
  let previous;
  try { previous = await source(); } catch { setTimeout(watchSource, 1000); return; }
  setInterval(async () => {
    try {
      const current = await source();
      if (current !== previous) reload(`${location.pathname}?v=${Date.now()}`);
    } catch { /* HALSim may briefly stop the server during a restart. */ }
  }, 1000);
}

export async function startDashboardRuntime() {
  if (started) return;
  started = true;
  if (!("serviceWorker" in navigator) || !window.isSecureContext) return watchSource();
  try {
    await navigator.serviceWorker.register("./service-worker.js", { scope: "./" });
    const registration = await navigator.serviceWorker.ready;
    navigator.serviceWorker.addEventListener("message", (event) => {
      if (event.data?.type === "SNAPSHOT_STATUS" && event.data.version) {
        if (new URLSearchParams(location.search).get("v") !== event.data.version) reload(`${location.pathname}?v=${event.data.version}`);
      } else if (event.data?.type === "SNAPSHOT_UPDATED") reload(`${location.pathname}?v=${event.data.version}`);
    });
    const check = () => (registration.active ?? navigator.serviceWorker.controller)?.postMessage({ type: "CHECK_UPDATE" });
    check();
    setInterval(check, 3000);
  } catch { watchSource(); }
}
