// Small MessagePack codec covering the NetworkTables value types.
const textEncoder = new TextEncoder();
const textDecoder = new TextDecoder();

export function encode(value) {
  const bytes = [];
  const push = (...values) => bytes.push(...values);
  const number = (value, size, setter) => {
    const buffer = new ArrayBuffer(size);
    new DataView(buffer)[setter](0, value, false);
    push(...new Uint8Array(buffer));
  };
  const write = (item) => {
    if (item === null) return push(0xc0);
    if (item === false) return push(0xc2);
    if (item === true) return push(0xc3);
    if (typeof item === "number") {
      if (Number.isSafeInteger(item) && item >= 0 && item < 128) return push(item);
      if (Number.isSafeInteger(item)) {
        push(0xd3);
        return number(BigInt(item), 8, "setBigInt64");
      }
      push(0xcb);
      return number(item, 8, "setFloat64");
    }
    if (typeof item === "bigint") {
      push(item >= 0n ? 0xcf : 0xd3);
      return number(item, 8, item >= 0n ? "setBigUint64" : "setBigInt64");
    }
    if (typeof item === "string") {
      const data = textEncoder.encode(item);
      if (data.length < 32) push(0xa0 | data.length);
      else if (data.length < 256) push(0xd9, data.length);
      else { push(0xda); number(data.length, 2, "setUint16"); }
      return push(...data);
    }
    if (item instanceof Uint8Array) {
      if (item.length < 256) push(0xc4, item.length);
      else { push(0xc5); number(item.length, 2, "setUint16"); }
      return push(...item);
    }
    if (Array.isArray(item)) {
      if (item.length < 16) push(0x90 | item.length);
      else { push(0xdc); number(item.length, 2, "setUint16"); }
      item.forEach(write);
      return;
    }
    const entries = Object.entries(item);
    if (entries.length < 16) push(0x80 | entries.length);
    else { push(0xde); number(entries.length, 2, "setUint16"); }
    entries.forEach(([key, val]) => { write(key); write(val); });
  };
  write(value);
  return new Uint8Array(bytes);
}

export function decodeMulti(input) {
  const data = input instanceof Uint8Array ? input : new Uint8Array(input);
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  let offset = 0;
  const take = (count) => { const result = data.subarray(offset, offset + count); offset += count; return result; };
  const num = (size, getter) => { const result = view[getter](offset, false); offset += size; return result; };
  const read = () => {
    const tag = data[offset++];
    if (tag <= 0x7f) return tag;
    if (tag >= 0xe0) return tag - 256;
    if ((tag & 0xe0) === 0xa0) return textDecoder.decode(take(tag & 0x1f));
    if ((tag & 0xf0) === 0x90) return Array.from({ length: tag & 0x0f }, read);
    if ((tag & 0xf0) === 0x80) return readMap(tag & 0x0f);
    switch (tag) {
      case 0xc0: return null;
      case 0xc2: return false;
      case 0xc3: return true;
      case 0xc4: return take(num(1, "getUint8"));
      case 0xc5: return take(num(2, "getUint16"));
      case 0xca: return num(4, "getFloat32");
      case 0xcb: return num(8, "getFloat64");
      case 0xcc: return num(1, "getUint8");
      case 0xcd: return num(2, "getUint16");
      case 0xce: return num(4, "getUint32");
      case 0xcf: return Number(num(8, "getBigUint64"));
      case 0xd0: return num(1, "getInt8");
      case 0xd1: return num(2, "getInt16");
      case 0xd2: return num(4, "getInt32");
      case 0xd3: return Number(num(8, "getBigInt64"));
      case 0xd9: return textDecoder.decode(take(num(1, "getUint8")));
      case 0xda: return textDecoder.decode(take(num(2, "getUint16")));
      case 0xdb: return textDecoder.decode(take(num(4, "getUint32")));
      case 0xdc: return Array.from({ length: num(2, "getUint16") }, read);
      case 0xdd: return Array.from({ length: num(4, "getUint32") }, read);
      case 0xde: return readMap(num(2, "getUint16"));
      case 0xdf: return readMap(num(4, "getUint32"));
      default: throw new Error(`Unsupported MessagePack tag 0x${tag.toString(16)}`);
    }
  };
  const readMap = (length) => Object.fromEntries(Array.from({ length }, () => [read(), read()]));
  const values = [];
  while (offset < data.length) values.push(read());
  return values;
}
