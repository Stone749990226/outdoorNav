// Browser acceptance runner using the system Firefox and the standard WebDriver protocol.
// No browser download or third-party WebDriver client is required when Firefox/geckodriver exist.
import { spawn } from "node:child_process";
import fs from "node:fs/promises";
import path from "node:path";
import assert from "node:assert/strict";
const root = path.resolve(".");
const results = path.join(root, "test-results");
await fs.mkdir(results, { recursive: true });
const downloads = await fs.mkdtemp(path.join(results, "downloads-"));
const binary =
  process.env.FIREFOX_BINARY || "/snap/firefox/current/usr/lib/firefox/firefox";
const gecko =
  process.env.GECKODRIVER_BINARY ||
  "/snap/firefox/current/usr/lib/firefox/geckodriver";
const port = 4445;
let driverLogs = "";
const driver = spawn(gecko, ["--port", String(port), "--log", "warn"]);
driver.stdout.on("data", (b) => (driverLogs += b.toString()));
driver.stderr.on("data", (b) => (driverLogs += b.toString()));
const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
let session;
async function request(endpoint, body, method = body ? "POST" : "GET") {
  const response = await fetch(`http://127.0.0.1:${port}${endpoint}`, {
    method,
    headers: { "content-type": "application/json" },
    body: body ? JSON.stringify(body) : undefined,
  });
  const result = await response.json();
  if (result.value?.error) throw new Error(JSON.stringify(result.value));
  return result.value;
}
const call = (endpoint, body, method) =>
  request(`/session/${session}/${endpoint}`, body, method);
const evaluate = (script, ...args) => call("execute/sync", { script, args });
async function waitFor(predicate, description, timeout = 10000) {
  const start = Date.now();
  while (Date.now() - start < timeout) {
    if (await predicate()) return;
    await sleep(80);
  }
  throw new Error(`Timeout: ${description}`);
}
async function click(selector) {
  const element = await call("element", {
    using: "css selector",
    value: selector,
  });
  return call(
    `element/${element["element-6066-11e4-a52e-4f735466cecf"]}/click`,
    {},
  );
}
const text = (selector) =>
  evaluate(
    "return document.querySelector(arguments[0])?.textContent",
    selector,
  );
async function screenshot(name) {
  await fs.writeFile(
    path.join(results, name),
    Buffer.from(await call("screenshot"), "base64"),
  );
}
async function advanceClock(seconds) {
  await evaluate(
    `
    for(let i=0;i<arguments[0]*5;i++) { window.__clockTime += 200; const callbacks=window.__rafQueue.splice(0); callbacks.forEach(cb=>cb(window.__clockTime)); }
    return true;
  `,
    seconds,
  );
  await sleep(70);
}
try {
  await waitFor(async () => {
    try {
      return (await request("/status")).ready;
    } catch {
      return false;
    }
  }, "geckodriver startup");
  const result = await request("/session", {
    capabilities: {
      alwaysMatch: {
        browserName: "firefox",
        "moz:firefoxOptions": {
          binary,
          args: ["-headless"],
          prefs: {
            "browser.download.folderList": 2,
            "browser.download.dir": downloads,
            "browser.download.useDownloadDir": true,
            "browser.helperApps.neverAsk.saveToDisk":
              "application/json,text/csv,text/plain,application/octet-stream",
            "browser.download.always_ask_before_handling_new_types": false,
            "browser.download.start_downloads_in_tmp_dir": false,
          },
        },
      },
    },
  });
  session = result.sessionId;
  await call("url", { url: "http://127.0.0.1:5173" });
  await waitFor(
    async () => (await text('[data-testid="status"]')) === "路线就绪",
    "initial application",
  );
  for (const [width, height] of [
    [1366, 768],
    [1920, 1080],
  ]) {
    await call("window/rect", { width, height });
    const delta = await evaluate(
      "return [outerWidth-innerWidth,outerHeight-innerHeight]",
    );
    await call("window/rect", {
      width: width + delta[0],
      height: height + delta[1],
    });
    await sleep(150);
    const layout = await evaluate(
      `return {viewport:[innerWidth,innerHeight],size:[document.body.scrollWidth,document.body.scrollHeight],buttons:[...document.querySelectorAll('.demo-button,.plan-button,.play-button')].map(e=>{const r=e.getBoundingClientRect();return r.top>=0&&r.bottom<=innerHeight&&r.left>=0&&r.right<=innerWidth;})}`,
    );
    assert.deepEqual(layout.viewport, [width, height]);
    assert.deepEqual(layout.size, [width, height]);
    assert.ok(layout.buttons.every(Boolean));
    const geometry = await evaluate(
      `const rows=[...document.querySelectorAll('.task-row')];const list=document.querySelector('.task-list').getBoundingClientRect();const m=document.querySelector('.campus-svg>g').getScreenCTM();return {allTasksVisible:rows.every(e=>{const r=e.getBoundingClientRect();return r.top>=list.top-1&&r.bottom<=list.bottom+1}),scaleError:Math.abs(document.querySelector('.scale-bar>div').getBoundingClientRect().width-80*m.a)};`,
    );
    assert.ok(geometry.allTasksVisible, "all five task rows are visible");
    assert.ok(
      geometry.scaleError < 1,
      "scale bar matches the SVG screen transform",
    );
    await screenshot(`desktop-${width}.png`);
    console.log(`PASS layout ${width}×${height}`);
  }
  await click('[title="图层设置"]');
  await evaluate(
    `const label=[...document.querySelectorAll('.layer-panel label')].find(e=>e.textContent.includes('道路拓扑网络'));label.click();`,
  );
  assert.equal(
    await evaluate('return document.querySelectorAll(".node-label").length'),
    11,
  );
  await click('[aria-label="关闭图层设置"]');
  await click(".map-tabs button:nth-child(2)");
  await screenshot("traversability.png");
  console.log("PASS semantic / traversability / road graph layers");
  await click('[title="封闭或恢复道路"]');
  await click('[data-testid="toggle-n-c"]');
  await click('[data-testid="toggle-ne-n"]');
  assert.equal(await text('[data-testid="status"]'), "目标不可达");
  assert.match(await text('[role="alert"]'), /无法到达/);
  await click(".export-menu summary");
  assert.equal(
    await evaluate(
      'return document.querySelector(".dropdown button:nth-child(2)").disabled',
    ),
    true,
  );
  await click(".export-menu summary");
  await screenshot("unreachable.png");
  await click('[data-testid="toggle-n-c"]');
  assert.equal(await text('[data-testid="status"]'), "路线就绪");
  await click('[aria-label="关闭道路管理"]');
  console.log("PASS disconnection, disabled export, restoration");
  await click('[title="重置巡航"]');
  await click('[title="删除东侧楼前"]');
  assert.equal(await text('[data-testid="status"]'), "等待规划");
  assert.equal(
    await evaluate('return document.querySelectorAll(".task-row").length'),
    4,
  );
  await click(".plan-button");
  assert.equal(await text('[data-testid="status"]'), "路线就绪");
  await click('[title="重置巡航"]');
  await click('[title="开始"]');
  await sleep(900);
  await click('[title="暂停"]');
  const paused = await evaluate(
    'return [document.querySelector("[data-testid=robot]").dataset.x,document.querySelector("[data-testid=robot]").dataset.y]',
  );
  await sleep(350);
  assert.deepEqual(
    await evaluate(
      'return [document.querySelector("[data-testid=robot]").dataset.x,document.querySelector("[data-testid=robot]").dataset.y]',
    ),
    paused,
  );
  await click('[title="继续"]');
  await sleep(250);
  assert.notDeepEqual(
    await evaluate(
      'return [document.querySelector("[data-testid=robot]").dataset.x,document.querySelector("[data-testid=robot]").dataset.y]',
    ),
    paused,
  );
  console.log("PASS task editing, invalidation, pause and continue");
  await click('[title="重置巡航"]');
  await click('[title="放大地图"]');
  async function mapClick(x, y) {
    const screen = await evaluate(
      `const p=document.querySelector('.campus-svg').createSVGPoint();p.x=arguments[0];p.y=arguments[1];const v=p.matrixTransform(document.querySelector('.campus-svg>g').getScreenCTM());return [Math.round(v.x),Math.round(v.y)];`,
      x,
      y,
    );
    await call("actions", {
      actions: [
        {
          type: "pointer",
          id: "mouse",
          parameters: { pointerType: "mouse" },
          actions: [
            { type: "pointerMove", duration: 0, x: screen[0], y: screen[1] },
            { type: "pointerDown", button: 0 },
            { type: "pointerUp", button: 0 },
          ],
        },
      ],
    });
    await sleep(100);
  }
  await click('[title="设置起点"]');
  await mapClick(865, 520);
  const picked = await evaluate(
    'return [Number(document.querySelector("[data-testid=robot]").dataset.x),Number(document.querySelector("[data-testid=robot]").dataset.y)]',
  );
  assert.ok(Math.abs(picked[0] - 865) < 1 && Math.abs(picked[1] - 520) < 2);
  await click(".add-task");
  await mapClick(600, 350);
  assert.equal(
    await evaluate('return document.querySelectorAll(".task-row").length'),
    5,
  );
  await mapClick(132, 790);
  assert.equal(
    await evaluate('return document.querySelectorAll(".task-row").length'),
    6,
  );
  await click(".plan-button");
  assert.equal(await text('[data-testid="status"]'), "路线就绪");
  await click('[title="退出编辑模式"]');
  const fileElement = await call("element", {
    using: "css selector",
    value: 'input[type="file"]',
  });
  await call(
    `element/${fileElement["element-6066-11e4-a52e-4f735466cecf"]}/value`,
    { text: path.join(results, "desktop-1366.png") },
  );
  await waitFor(
    async () =>
      await evaluate('return !!document.querySelector(".reference-panel")'),
    "reference image import",
  );
  assert.equal(
    await evaluate(
      'return document.querySelectorAll(".campus-svg image").length',
    ),
    1,
  );
  const positionBefore = await evaluate(
    'return document.querySelector("[data-testid=robot]").dataset.x',
  );
  await evaluate(
    `const input=document.querySelector('input[aria-label="水平偏移"]');Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set.call(input,100);input.dispatchEvent(new Event('input',{bubbles:true}));input.dispatchEvent(new Event('change',{bubbles:true}));`,
  );
  await sleep(100);
  assert.equal(
    await evaluate(
      'return document.querySelector(".campus-svg image").getAttribute("x")',
    ),
    "100",
  );
  assert.equal(
    await evaluate(
      'return document.querySelector("[data-testid=robot]").dataset.x',
    ),
    positionBefore,
  );
  await click(".reference-panel .text-button");
  assert.equal(
    await evaluate(
      'return document.querySelectorAll(".campus-svg image").length',
    ),
    0,
  );
  await click('[title="重置巡航"]');
  await click('[title="复位视角"]');
  console.log(
    "PASS zoom-aware road picking, blocked-ground rejection, custom task and reference image alignment",
  );
  await click(".demo-button");
  // Capture requestAnimationFrame after the already scheduled native frame, then drive it deterministically.
  await evaluate(
    "window.__rafQueue=[];window.__clockTime=performance.now();window.requestAnimationFrame=cb=>{window.__rafQueue.push(cb);return window.__rafQueue.length};",
  );
  await sleep(150);
  await advanceClock(20);
  assert.equal(await text('[data-testid="replans"]'), "01次");
  assert.equal(
    await evaluate(
      'return !!document.querySelector("[data-testid=old-route]")',
    ),
    true,
  );
  await screenshot("replanned.png");
  for (let i = 0; i < 7; i++) await advanceClock(20);
  assert.equal(await text('[data-testid="status"]'), "巡航已完成");
  assert.equal(
    await evaluate('return document.querySelectorAll(".task-row.done").length'),
    5,
  );
  assert.deepEqual(
    await evaluate(
      'return [document.querySelector("[data-testid=robot]").dataset.x,document.querySelector("[data-testid=robot]").dataset.y]',
    ),
    ["480.000", "290.000"],
  );
  await screenshot("completed.png");
  console.log(
    "PASS complete automatic demo, replan, all five inspections and return",
  );
  await click(".export-menu summary");
  await click(".dropdown button:nth-child(1)");
  await click(".dropdown button:nth-child(2)");
  await click(".dropdown button:nth-child(3)");
  for (const filename of [
    "campus-map.json",
    "patrol-route.json",
    "patrol-waypoints.csv",
  ]) {
    await waitFor(async () => {
      try {
        return (await fs.stat(path.join(downloads, filename))).size > 0;
      } catch {
        return false;
      }
    }, `download ${filename}`);
  }
  const route = JSON.parse(
    await fs.readFile(path.join(downloads, "patrol-route.json"), "utf8"),
  );
  assert.equal(route.scale_is_mock, true);
  assert.equal(route.frame_id, "demo_map");
  assert.equal(route.completed_task_ids.length, 5);
  assert.ok(route.blocked_roads.includes("c-e"));
  assert.equal(route.task_order.length, 5);
  assert.ok(route.traveled_path.length > 20);
  assert.match(
    await fs.readFile(path.join(downloads, "patrol-waypoints.csv"), "utf8"),
    /scale_is_mock/,
  );
  console.log("PASS map JSON, route JSON and waypoint CSV downloads");
  // Reload to restore native animation frames after the accelerated patrol test.
  await call("url", { url: "http://127.0.0.1:5173" });
  await click(".workspace-switch button:nth-child(2)");
  await waitFor(
    async () =>
      !!(await evaluate(
        'return document.querySelector("[data-testid=free-area]")',
      )),
    "free-area overlay",
  );
  await mapClick(869, 495);
  await mapClick(478, 844);
  const abA = await evaluate(
    'return {x:+document.querySelector("[data-testid=ab-a]").dataset.x,y:+document.querySelector("[data-testid=ab-a]").dataset.y}',
  );
  assert.ok(Math.abs(abA.x - 869) < 2);
  assert.ok(abA.x > 867, "A is not snapped back to x=865");
  await click("[data-testid=plan-ab]");
  await waitFor(
    async () =>
      !!(await evaluate(
        'return document.querySelector("[data-testid=planned-route]")',
      )),
    "A/B route",
  );
  assert.ok(
    await evaluate(
      'return document.querySelectorAll("[data-testid=important-waypoint]").length>2',
    ),
  );
  for (const [width, height] of [
    [1366, 768],
    [1920, 1080],
  ]) {
    const delta = await evaluate(
      "return [outerWidth-innerWidth,outerHeight-innerHeight]",
    );
    await call("window/rect", {
      width: width + delta[0],
      height: height + delta[1],
    });
    await sleep(100);
    assert.ok(
      await evaluate(
        "return document.body.scrollWidth<=innerWidth&&document.body.scrollHeight<=innerHeight",
      ),
    );
    assert.ok(
      await evaluate(
        'const p=document.querySelector("[data-testid=plan-ab]").getBoundingClientRect();return p.top>=0&&p.bottom<=innerHeight',
      ),
    );
    assert.ok(await evaluate('return [...document.querySelectorAll(".ab-export button")].every(e=>{const r=e.getBoundingClientRect();const p=e.closest("aside").getBoundingClientRect();return r.top>=p.top&&r.bottom<=p.bottom})'), 'all A/B exports visible');
    await screenshot(`ab-${width}.png`);
  }
  await click(".ab-export button:nth-child(1)");
  await click(".ab-export button:nth-child(2)");
  for (const name of ["ab-route.json", "ab-waypoints.csv"])
    await waitFor(async () => {
      try {
        return (await fs.stat(path.join(downloads, name))).size > 0;
      } catch {
        return false;
      }
    }, name);
  const ab = JSON.parse(
    await fs.readFile(path.join(downloads, "ab-route.json"), "utf8"),
  );
  assert.equal(ab.mode, "point_to_point");
  assert.equal(ab.georeferenced, false);
  assert.equal(ab.executable_on_robot, false);
  assert.ok(Math.abs(ab.start.x - abA.x * 0.25) < 0.001);
  assert.equal(
    ab.waypoints.length,
    await evaluate('return document.querySelectorAll(".keypoint-row").length'),
  );
  assert.ok(ab.waypoints.every((w) => w.reasons.length));
  assert.match(
    await fs.readFile(path.join(downloads, "ab-waypoints.csv"), "utf8"),
    /reasons/,
  );
  await click('[title="播放 A/B 预览"]');
  await evaluate(
    "window.__rafQueue=[];window.__clockTime=performance.now();window.requestAnimationFrame=cb=>{window.__rafQueue.push(cb);return window.__rafQueue.length};",
  );
  await sleep(150);
  for (let i = 0; i < 5; i++) await advanceClock(20);
  assert.match(await text(".ab-arrived"), /已到达 B/);
  const arrived = await evaluate(
    'return [document.querySelector("[data-testid=robot]").dataset.x,document.querySelector("[data-testid=robot]").dataset.y]',
  );
  const goal = await evaluate(
    'return [Number(document.querySelector("[data-testid=ab-b]").dataset.x).toFixed(3),Number(document.querySelector("[data-testid=ab-b]").dataset.y).toFixed(3)]',
  );
  assert.deepEqual(arrived, goal);
  await click('[title="A/B 道路封闭管理"]');
  await click("[data-testid=ab-toggle-e-ne]");
  assert.equal(
    await evaluate(
      'return document.querySelector(".ab-export button").disabled',
    ),
    true,
  );
  assert.equal(
    await evaluate(
      'return !!document.querySelector("[data-testid=planned-route]")',
    ),
    false,
  );
  await click('[aria-label="关闭 A/B 道路管理"]');
  await click("[data-testid=plan-ab]");
  await waitFor(
    async () => /A 点位于/.test(await text("[data-testid=ab-message]")),
    "closed endpoint rejection",
  );
  await click('[title="A/B 道路封闭管理"]');
  await click("[data-testid=ab-toggle-e-ne]");
  await click('[aria-label="关闭 A/B 道路管理"]');
  await click("[data-testid=plan-ab]");
  await waitFor(
    async () =>
      !!(await evaluate(
        'return document.querySelector("[data-testid=planned-route]")',
      )),
    "restored A/B path",
  );
  await click("[data-testid=select-b]");
  await mapClick(600, 350);
  assert.match(await text("[data-testid=ab-message]"), /不满足通行条件/);
  assert.deepEqual(
    await evaluate(
      'return [Number(document.querySelector("[data-testid=ab-b]").dataset.x).toFixed(3),Number(document.querySelector("[data-testid=ab-b]").dataset.y).toFixed(3)]',
    ),
    goal,
  );
  await click('[title="放大地图"]');
  await mapClick(861, 575);
  assert.equal(
    await evaluate(
      'return document.querySelector(".ab-export button").disabled',
    ),
    true,
  );
  assert.equal(
    await evaluate(
      'return !!document.querySelector("[data-testid=planned-route]")',
    ),
    false,
  );
  await click("[data-testid=plan-ab]");
  await waitFor(
    async () =>
      !!(await evaluate(
        'return document.querySelector("[data-testid=planned-route]")',
      )),
    "zoomed B edit",
  );
  await click('[title="重置 A/B"]');
  await click('[title="复位视角"]');
  await mapClick(480, 290);
  await mapClick(865, 660);
  await click('[title="A/B 道路封闭管理"]');
  await click("[data-testid=ab-toggle-n-c]");
  await click("[data-testid=ab-toggle-ne-n]");
  await click('[aria-label="关闭 A/B 道路管理"]');
  await click("[data-testid=plan-ab]");
  await waitFor(
    async () => /不连通/.test(await text("[data-testid=ab-message]")),
    "disconnected A/B",
  );
  await screenshot("ab-disconnected.png");
  console.log(
    "PASS free-area A/B exact endpoints, keypoint JSON/CSV, preview, invalidation, closures, forbidden picks, zoom and disconnected areas",
  );
  console.log(
    "All Firefox browser acceptance checks passed. Screenshots: test-results/",
  );
} catch (error) {
  if (session) await screenshot("failure.png").catch(() => {});
  console.error(error);
  console.error(driverLogs.slice(-1500));
  process.exitCode = 1;
} finally {
  if (session)
    await request(`/session/${session}`, undefined, "DELETE").catch(() => {});
  driver.kill("SIGTERM");
}
