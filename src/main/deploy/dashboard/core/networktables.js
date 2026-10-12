const assetVersion = new URL(import.meta.url).searchParams.get("v") ?? "initial";
const { decodeMulti, encode } = await import(`./msgpack.js?v=${assetVersion}`);

const TYPES = { boolean: 0, double: 1, int: 2, float: 3, string: 4, raw: 5,
  "boolean[]": 16, "double[]": 17, "int[]": 18, "float[]": 19, "string[]": 20 };

export class NT4Client extends EventTarget {
  constructor(host) {
    super();
    this.host = host;
    this.socket = null;
    this.topics = new Map();
    this.published = new Map();
    this.nextUid = 1;
    this.reconnectTimer = 0;
    this.shouldReconnect = true;
  }

  connect() {
    clearTimeout(this.reconnectTimer);
    const url = `ws://${this.host}:5810/nt/custom-dashboard`;
    const socket = new WebSocket(url, "v4.1.networktables.first.wpi.edu");
    this.socket = socket;
    socket.binaryType = "arraybuffer";
    let verificationTimer;
    socket.addEventListener("open", () => {
      if (this.socket !== socket) return;
      this.sendControl("subscribe", { subuid: 1, topics: ["/"], options: { prefix: true, periodic: 0.1 } });
      this.sendControl("subscribe", {
        subuid: 2,
        topics: ["/AdvantageKit/RealOutputs/Odometry/Robot"],
        options: { prefix: false, all: true, periodic: 0.02 },
      });
      for (const topic of this.published.values()) this.announce(topic);
      verificationTimer = setTimeout(() => {
        if (!socket.ntVerified) socket.close();
      }, 1500);
    });
    socket.addEventListener("message", (event) => {
      if (!socket.ntVerified && isNT4ControlMessage(event.data)) {
        socket.ntVerified = true;
        clearTimeout(verificationTimer);
        this.dispatchEvent(new Event("connected"));
      }
      this.onMessage(event);
    });
    socket.addEventListener("close", () => {
      clearTimeout(verificationTimer);
      if (this.socket !== socket) return;
      this.dispatchEvent(new Event("disconnected"));
      if (this.shouldReconnect) this.reconnectTimer = setTimeout(() => this.connect(), 1000);
    });
    socket.addEventListener("error", () => socket.close());
  }

  setHost(host) {
    if (!host || host === this.host) return;
    this.shouldReconnect = false;
    clearTimeout(this.reconnectTimer);
    this.socket?.close();
    this.topics.clear();
    this.host = host;
    this.shouldReconnect = true;
    this.connect();
  }

  publish(name, type, value) {
    let topic = this.published.get(name);
    if (!topic) {
      topic = { name, type, pubuid: this.nextUid++ };
      this.published.set(name, topic);
      if (this.socket?.readyState === WebSocket.OPEN) this.announce(topic);
    }
    if (this.socket?.readyState !== WebSocket.OPEN) return;
    this.socket.send(encode([topic.pubuid, 0, TYPES[type], value]));
  }

  announce(topic) {
    this.sendControl("publish", { name: topic.name, pubuid: topic.pubuid, type: topic.type, properties: {} });
  }

  sendControl(method, params) {
    if (this.socket?.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify([{ method, params }]));
  }

  onMessage(event) {
    if (typeof event.data === "string") {
      for (const message of JSON.parse(event.data)) {
        if (message.method === "announce") {
          const topic = { ...message.params, value: undefined, updated: performance.now() };
          this.topics.set(topic.id, topic);
          this.dispatchEvent(new CustomEvent("announce", { detail: topic }));
        } else if (message.method === "unannounce") {
          this.topics.delete(message.params.id);
        }
      }
      return;
    }
    try {
      for (const sample of decodeMulti(event.data)) {
        if (!Array.isArray(sample) || sample.length !== 4) continue;
        const topic = this.topics.get(sample[0]);
        if (!topic) continue;
        topic.value = sample[3];
        topic.updated = performance.now();
        this.dispatchEvent(new CustomEvent("value", { detail: topic }));
      }
    } catch (error) {
      console.warn("Could not decode NT4 value", error);
    }
  }
}

export function probeNT4(host, timeoutMs = 1500) {
  return new Promise((resolve) => {
    let settled = false;
    const finish = (available) => {
      if (settled) return;
      settled = true;
      clearTimeout(timeout);
      try { socket.close(); } catch { /* Already closed. */ }
      resolve(available);
    };
    let socket;
    try {
      socket = new WebSocket(`ws://${host}:5810/nt/custom-dashboard-probe-${Date.now()}`, "v4.1.networktables.first.wpi.edu");
      socket.addEventListener("open", () => socket.send(JSON.stringify([{
        method: "subscribe",
        params: { subuid: 1, topics: ["/"], options: { prefix: true, periodic: 0.1 } },
      }])));
      socket.addEventListener("message", (event) => {
        if (isNT4ControlMessage(event.data)) finish(true);
      });
      socket.addEventListener("error", () => finish(false));
      socket.addEventListener("close", () => finish(false));
    } catch {
      resolve(false);
      return;
    }
    const timeout = setTimeout(() => finish(false), timeoutMs);
  });
}

function isNT4ControlMessage(data) {
  if (typeof data !== "string") return false;
  try {
    const messages = JSON.parse(data);
    return Array.isArray(messages) && messages.some((message) => {
      if (message?.method !== "announce" || typeof message.params?.name !== "string") return false;
      const name = message.params.name;
      return !name.startsWith("$") && !name.startsWith("/SimSupervisor/");
    });
  } catch {
    return false;
  }
}
