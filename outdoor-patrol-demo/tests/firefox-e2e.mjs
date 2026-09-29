// Optional acceptance with a system Firefox through standard WebDriver.
import { spawn } from "node:child_process";
import fs from "node:fs/promises";
import path from "node:path";
import assert from "node:assert/strict";
const root = path.resolve(".");
const results = path.join(root, "test-results");
await fs.mkdir(results, { recursive: true });
const campus = JSON.parse(await fs.readFile(path.join(root, "src/campus.json"), "utf8"));
const base = process.env.BASE_URL || "http://127.0.0.1:5173";
const binary = process.env.FIREFOX_BINARY || "/snap/firefox/current/usr/lib/firefox/firefox";
const gecko = process.env.GECKODRIVER_BINARY || "/snap/firefox/current/usr/lib/firefox/geckodriver";
const driver = spawn(gecko, ["--port", "4445", "--log", "warn"]);
let session, driverLogs = "", launchError;
driver.on("error", (error) => { launchError = error; });
driver.stdout.on("data", (data) => { driverLogs += data; });
driver.stderr.on("data", (data) => { driverLogs += data; });
const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
async function request(endpoint, body, method = body ? "POST" : "GET") {
  const response = await fetch(`http://127.0.0.1:4445${endpoint}`, { method,
    headers: { "content-type": "application/json" }, body: body ? JSON.stringify(body) : undefined });
  const result = await response.json();
  if (result.value?.error) throw new Error(JSON.stringify(result.value));
  return result.value;
}
const call = (endpoint, body, method) => request(`/session/${session}/${endpoint}`, body, method);
const evaluate = (script, ...args) => call("execute/sync", { script, args });
async function waitFor(predicate, description) {
  const start = Date.now();
  while (Date.now() - start < 10000) { if (launchError) throw launchError; if (await predicate()) return; await sleep(100); }
  throw new Error(`Timeout: ${description}`);
}
async function click(selector) {
  const element = await call("element", { using: "css selector", value: selector });
  return call(`element/${element["element-6066-11e4-a52e-4f735466cecf"]}/click`, {});
}
async function mapClick(point) {
  const screen = await evaluate("const svg=document.querySelector('.campus-svg');const p=svg.createSVGPoint();p.x=arguments[0].x;p.y=arguments[0].y;const v=p.matrixTransform(svg.querySelector(':scope>g').getScreenCTM());return {x:Math.round(v.x),y:Math.round(v.y)}", point);
  await call("actions", { actions: [{ type: "pointer", id: "mouse", parameters: { pointerType: "mouse" }, actions: [
    { type: "pointerMove", duration: 0, ...screen }, { type: "pointerDown", button: 0 }, { type: "pointerUp", button: 0 },
  ] }] });
}
try {
  await waitFor(async () => { try { return (await request("/status")).ready; } catch { return false; } }, "driver startup");
  const result = await request("/session", { capabilities: { alwaysMatch: { browserName: "firefox", "moz:firefoxOptions": { binary, args: ["-headless"] } } } });
  session = result.sessionId;
  await call("url", { url: base });
  await waitFor(async () => await evaluate("return document.querySelectorAll('[data-testid^=centerline-]').length===3"), "measured network");
  await click(".workspace-switch button:nth-child(2)");
  await mapClick(campus.nodes[0]); await mapClick(campus.nodes[2]);
  await click("[data-testid=plan-ab]");
  await waitFor(async () => await evaluate("return document.querySelector('[data-testid=ab-message]').textContent.includes('路线已生成')"), "branch route");
  assert.ok(await evaluate("return document.querySelectorAll('[data-testid=important-waypoint]').length>2"));
  assert.equal(await evaluate("return document.querySelectorAll('.campus-svg polygon').length"), 0);
  await click("[title='A/B 道路封闭管理']"); await click("[data-testid=ab-toggle-branch]");
  assert.equal(await evaluate("return document.querySelector('.ab-export button').disabled"), true);
  await click("[aria-label='关闭 A/B 道路管理']"); await click("[data-testid=plan-ab]");
  await waitFor(async () => await evaluate("return document.querySelector('[data-testid=ab-message]').textContent.includes('断开')"), "closed branch rejection");
  for (const [width, height] of [[1366, 768], [390, 844]]) {
    const delta = await evaluate("return [outerWidth-innerWidth,outerHeight-innerHeight]");
    await call("window/rect", { width: width + delta[0], height: height + delta[1] });
    await sleep(200);
    assert.ok(await evaluate("return document.documentElement.scrollWidth<=innerWidth"));
    await fs.writeFile(path.join(results, `firefox-survey-${width}.png`), Buffer.from(await call("screenshot"), "base64"));
  }
  console.log("PASS Firefox measured topology, A/B, closed branch and responsive layouts");
} catch (error) {
  console.error(error); console.error(driverLogs.slice(-1500)); process.exitCode = 1;
} finally {
  if (session) await request(`/session/${session}`, undefined, "DELETE").catch(() => {});
  driver.kill("SIGTERM");
}
