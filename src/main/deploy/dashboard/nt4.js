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
  }

  connect() {
    clearTimeout(this.reconnectTimer);
    const url = `ws://${this.host}:5810/nt/chur-dashboard`;
    this.socket = new WebSocket(url, "v4.1.networktables.first.wpi.edu");
    this.socket.binaryType = "arraybuffer";
    this.socket.addEventListener("open", () => {
      this.sendControl("subscribe", { subuid: 1, topics: ["/"], options: { prefix: true, periodic: 0.1 } });
      this.sendControl("subscribe", {
        subuid: 2,
        topics: ["/AdvantageKit/RealOutputs/Odometry/Robot"],
        options: { prefix: false, all: true, periodic: 0.02 },
      });
      for (const topic of this.published.values()) this.announce(topic);
      this.dispatchEvent(new Event("connected"));
    });
    this.socket.addEventListener("message", (event) => this.onMessage(event));
    this.socket.addEventListener("close", () => {
      this.dispatchEvent(new Event("disconnected"));
      this.reconnectTimer = setTimeout(() => this.connect(), 1000);
    });
    this.socket.addEventListener("error", () => this.socket.close());
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
