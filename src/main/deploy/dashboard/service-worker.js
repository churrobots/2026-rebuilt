const META_CACHE = "custom-dashboard-meta";
const SNAPSHOT_PREFIX = "custom-dashboard-snapshot-";
const ACTIVE_KEY = new URL("__active_snapshot__", self.registration.scope).href;
const ASSETS = [
  "index.html",
  "index.css",
  "custom-dashboard.css",
  "custom-dashboard.js",
  "core/networktables.js",
  "core/dashboard-core.js",
  "core/dashboard-runtime.js",
  "core/nt-connectivity.js",
  "core/sim-driver-station.js",
  "core/msgpack.js",
  "manifest.webmanifest",
  "core/icon.svg",
];
let updatePromise = null;

self.addEventListener("install", (event) => {
  event.waitUntil(buildSnapshot().then(() => self.skipWaiting()));
});

self.addEventListener("activate", (event) => {
  event.waitUntil((async () => {
    await self.clients.claim();
    const active = await getActive();
    if (!active) return;
    const clients = await self.clients.matchAll({ type: "window", includeUncontrolled: true });
    await Promise.all(clients.map((client) => {
      const url = new URL(client.url);
      if (url.searchParams.get("v") === active.version) return undefined;
      url.searchParams.set("v", active.version);
      return client.navigate(url.href);
    }));
  })());
});

self.addEventListener("message", (event) => {
  if (event.data?.type === "CHECK_UPDATE") event.waitUntil(checkForUpdate(event.source));
  if (event.data?.type === "ROLLBACK_SNAPSHOT") event.waitUntil(rollbackSnapshot(event.data.version, event.source));
});

self.addEventListener("fetch", (event) => {
  if (event.request.method !== "GET") return;
  const requestUrl = new URL(event.request.url);
  if (requestUrl.origin !== self.location.origin) return;
  const relativePath = requestUrl.pathname.slice(new URL(self.registration.scope).pathname.length) || "index.html";
  const asset = ASSETS.includes(relativePath) ? relativePath : (event.request.mode === "navigate" ? "index.html" : null);
  if (!asset) return;
  event.respondWith(fromActiveSnapshot(asset, event.request));
});

async function fromActiveSnapshot(asset, request) {
  const active = await getActive();
  if (active) {
    const cache = await caches.open(active.cache);
    const cached = await cache.match(new URL(asset, self.registration.scope).href);
    if (cached) return cached;
  }
  return fetch(request);
}

async function checkForUpdate(requestingClient) {
  if (!updatePromise) updatePromise = buildSnapshot().finally(() => { updatePromise = null; });
  try {
    const result = await updatePromise;
    if (!result.changed) {
      requestingClient?.postMessage({ type: "SNAPSHOT_STATUS", version: result.version });
      return;
    }
    const clients = await self.clients.matchAll({ type: "window", includeUncontrolled: true });
    clients.forEach((client) => client.postMessage({ type: "SNAPSHOT_UPDATED", version: result.version }));
  } catch {
    // A partial or unavailable server never replaces the active known-good snapshot.
  }
}

async function buildSnapshot() {
  const downloaded = await Promise.all(ASSETS.map(async (asset) => {
    const url = new URL(asset, self.registration.scope);
    url.searchParams.set("snapshot", Date.now().toString());
    const response = await fetch(url, { cache: "no-store" });
    if (!response.ok) throw new Error(`${asset}: HTTP ${response.status}`);
    return { asset, response, bytes: new Uint8Array(await response.clone().arrayBuffer()) };
  }));
  const version = await digest(downloaded);
  const active = await getActive();
  if (active?.version === version) return { changed: false, version };
  if (active?.rejectedVersion === version) return { changed: false, version: active.version };

  const cacheName = `${SNAPSHOT_PREFIX}${version}`;
  const snapshot = await caches.open(cacheName);
  await Promise.all(downloaded.map(({ asset, response }) =>
    snapshot.put(new URL(asset, self.registration.scope).href, response)));

  const meta = await caches.open(META_CACHE);
  const previous = active && { cache: active.cache, version: active.version };
  await meta.put(ACTIVE_KEY, new Response(JSON.stringify({ cache: cacheName, version, previous })));
  await retainCurrentAndPrevious(cacheName, active?.cache);
  return { changed: Boolean(active), version };
}

/** Restores the previous snapshot after the page reports a failed module load. */
async function rollbackSnapshot(version, requestingClient) {
  const active = await getActive();
  if (active?.version !== version || !active.previous) {
    requestingClient?.postMessage({ type: "SNAPSHOT_ROLLBACK_UNAVAILABLE" });
    return;
  }
  const restored = { ...active.previous, rejectedVersion: active.version };
  const meta = await caches.open(META_CACHE);
  await meta.put(ACTIVE_KEY, new Response(JSON.stringify(restored)));
  await retainCurrentAndPrevious(restored.cache, active.cache);
  requestingClient?.postMessage({ type: "SNAPSHOT_ROLLED_BACK", version: restored.version });
}

async function getActive() {
  const meta = await caches.open(META_CACHE);
  const response = await meta.match(ACTIVE_KEY);
  return response ? response.json() : null;
}

async function retainCurrentAndPrevious(current, previous) {
  const keep = new Set([current, previous, META_CACHE].filter(Boolean));
  const names = await caches.keys();
  await Promise.all(names.filter((name) => name.startsWith(SNAPSHOT_PREFIX) && !keep.has(name)).map((name) => caches.delete(name)));
}

async function digest(downloaded) {
  const total = downloaded.reduce((size, item) => size + item.bytes.length, 0);
  const combined = new Uint8Array(total);
  let offset = 0;
  for (const item of downloaded) {
    combined.set(item.bytes, offset);
    offset += item.bytes.length;
  }
  const hash = new Uint8Array(await crypto.subtle.digest("SHA-256", combined));
  return [...hash].slice(0, 8).map((byte) => byte.toString(16).padStart(2, "0")).join("");
}
