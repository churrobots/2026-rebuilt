const version = new URLSearchParams(location.search).get("v") ?? "initial";

try {
  await import(`./dashboard-core.js?v=${version}`);
  await import(`../custom-dashboard.js?v=${version}`);
} catch (error) {
  console.error("Dashboard update could not load", error);
  const restoredVersion = await restoreLastWorkingSnapshot(version);
  if (restoredVersion)
    location.replace(`${location.pathname}?v=${restoredVersion}`);
  showLoadFailure();
}

async function restoreLastWorkingSnapshot(failedVersion) {
  if (!("serviceWorker" in navigator)) return null;
  const registration = await navigator.serviceWorker.getRegistration();
  const worker = registration?.active ?? navigator.serviceWorker.controller;
  if (!worker) return null;
  return new Promise((resolve) => {
    const timeout = setTimeout(() => finish(null), 1500);
    const receive = (event) => {
      if (event.data?.type === "SNAPSHOT_ROLLED_BACK")
        finish(event.data.version);
      if (event.data?.type === "SNAPSHOT_ROLLBACK_UNAVAILABLE") finish(null);
    };
    const finish = (result) => {
      clearTimeout(timeout);
      navigator.serviceWorker.removeEventListener("message", receive);
      resolve(result);
    };
    navigator.serviceWorker.addEventListener("message", receive);
    worker.postMessage({ type: "ROLLBACK_SNAPSHOT", version: failedVersion });
  });
}

function showLoadFailure() {
  const message = document.createElement("p");
  message.textContent =
    "Dashboard update could not load. Check the browser console for details.";
  document.body.replaceChildren(message);
}
