// Run with node --test tests/web-transport.test.mjs (no npm dependencies).
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
const source = await readFile(
  new URL("../src/robot_infra/web/xr-transport.js", import.meta.url),
  "utf8",
);
const { InputTransport } = await import(
  "data:text/javascript;base64," + Buffer.from(source).toString("base64")
);
const flush = () => new Promise((resolve) => setImmediate(resolve));

test("connection rejection latches once and stops an ongoing XR frame loop flooding HTTP", async () => {
  const requests = [],
    errors = [];
  const transport = new InputTransport((e) => errors.push(String(e)));
  transport.post = async (path) => {
    requests.push(path);
    if (path === "/api/input") throw Error("stale server freshness ticket");
  };
  transport.submit({ seq: 0 });
  await flush();
  for (let i = 0; i < 300; i++) transport.submit({ seq: i }, i % 5 === 0);
  await flush();
  assert.equal(transport.failed, true);
  assert.equal(errors.length, 1);
  assert.deepEqual(requests, ["/api/input", "/api/stop"]);
  assert.equal(transport.edges.length, 0);
  assert.equal(transport.latest, null);
});

test("slow ACK retains button edges in order and replaces unsent motion with the latest pose", async () => {
  const sent = [],
    acks = [];
  const transport = new InputTransport((e) => assert.fail(e));
  transport.post = async (path, packet) => {
    sent.push(structuredClone(packet));
    await new Promise((resolve) => acks.push(resolve));
  };
  transport.submit({ pose: 0 });
  transport.submit({ pose: 1 });
  transport.submit({ pose: 2 });
  transport.submit({ button: "A down" }, true);
  transport.submit({ button: "A up" }, true);
  transport.submit({ pose: 3 });
  transport.submit({ pose: 4 });
  for (let i = 0; i < 4; i++) {
    acks.shift()();
    await flush();
  }
  assert.deepEqual(
    sent.map((p) => p.button ?? p.pose),
    [0, "A down", "A up", 4],
  );
  assert.deepEqual(
    sent.map((p) => p.seq),
    [0, 1, 2, 3],
  );
  assert.equal(transport.failed, false);
});

test("WebRTC sends consecutive ordered packets before ACK; bounds backlog and stops once", async () => {
  const errors = [],
    sent = [],
    requests = [];
  const t = new InputTransport((e) => errors.push(String(e)));
  t.config = { input: "webrtc" };
  t.rtcReady = true;
  t.inflight = new Map();
  t.channel = { bufferedAmount: 0, send: (raw) => sent.push(JSON.parse(raw)) };
  t.post = async (path) => requests.push(path);
  for (let i = 0; i < 10; i++) t.submit({ pose: i, button: i % 2 }, true);
  assert.deepEqual(
    sent.map((p) => p.pose),
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
  );
  assert.deepEqual(
    sent.map((p) => p.seq),
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
  );
  assert.equal(t.inflight.size, 10);
  assert.equal(errors.length, 0);
  for (let i = 10; i < 100; i++) t.submit({ pose: i });
  await flush();
  assert.equal(sent.length, 32);
  assert.equal(t.failed, true);
  assert.equal(t.inflight.size, 0);
  assert.equal(errors.length, 1);
  assert.equal(t.failureDetails.unacked, 32);
  assert.deepEqual(requests, ["/api/stop"]);
});

test("WebRTC page stays idle outside VR; exit releases peer; re-entry makes one connection", async () => {
  let opened = 0,
    closed = 0;
  const errors = [],
    t = new InputTransport((e) => errors.push(e));
  t.openRTC = async () => {
    opened++;
    t.peer = {
      close() {
        closed++;
      },
    };
    t.channel = { bufferedAmount: 0 };
    t.inflight = new Map();
    t.rtcReady = true;
  };
  t.configure({ input: "webrtc" });
  assert.equal(opened, 0);
  t.start();
  t.start();
  t.configure({ input: "webrtc" });
  assert.equal(opened, 1);
  t.suspend();
  assert.equal(closed, 1);
  assert.equal(t.peer, null);
  assert.equal(t.rtcReady, false);
  assert.equal(t.failed, false);
  t.configure({ input: "webrtc" });
  assert.equal(opened, 1);
  t.start();
  assert.equal(opened, 2);
  t.suspend();
  assert.equal(closed, 2);
  assert.deepEqual(errors, []);
});

test("late negotiation rejection after VR exit does not poison a new connection", async () => {
  const errors = [],
    t = new InputTransport((e) => errors.push(e));
  let rejectOpening;
  t.openRTC = () => {
    t.peer = { close() {} };
    return new Promise((resolve, reject) => {
      rejectOpening = reject;
    });
  };
  t.configure({ input: "webrtc" });
  t.start();
  t.suspend();
  rejectOpening(Error("closed while negotiating"));
  await flush();
  assert.equal(t.failed, false);
  assert.deepEqual(errors, []);
});

test("WebRTC buffer overload records diagnostics before cleanup", async () => {
  const errors = [],
    t = new InputTransport((e) => errors.push(String(e)));
  t.config = { input: "webrtc" };
  t.rtcReady = true;
  t.inflight = new Map();
  t.channel = { bufferedAmount: 70000 };
  t.post = async () => {};
  t.submit({ pose: 1 });
  await flush();
  assert.match(errors[0], /发送缓冲积压/);
  assert.equal(t.failureDetails.buffered_bytes, 70000);
  assert.equal(t.failureDetails.unacked, 0);
});
