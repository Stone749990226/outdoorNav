// ★ A（起点）＝ 小车当前位置 ＋ 覆盖式发路线 —— **真栈上的端到端**用例 ✓
//   ★ 要一座**跑着的**栈 ＋ `frontend-bridge.sh`（它把适配器／他们的桥／前端都起好 ✓）：
//       BRIDGE_LIVE=1 npx playwright test tests/ab-live.spec.ts
//   ★ 默认**跳过** ✓（✗ 不带真栈跑会全是失败 ✗ —— 与 `bridge.spec.ts` 里那条真栈用例同一种做法 ✓）
import { expect, test, type Page } from "@playwright/test";
import { campus, calibration, choose } from "./mapHelpers";

test.skip(process.env.BRIDGE_LIVE !== "1", "needs the live stack + frontend-bridge.sh");

// ★★ 到点门槛 ＝ **≤0.5 m（本仓既有门槛）＋ 末速 0** ✓（★ reviewer 2026-10-14 定的口径 ✓）
//   ★ 原来那条 `≤0.30 m` **已由 reviewer 撤回** ✓ —— ★ 理由：这条链的实际到点精度够不到它 ✓
//     ★ 实测（同一座栈 ✓）：★ 向南那一路 **0.343 / 0.472 / 0.283**、覆盖那条 **0.372**；
//     ★ 向北那一路 **0.246**；★ 车离线 6.48 m 发的那条 **0.259** ⇒ ★ 与**接近方向**强相关 ✓
//   ★ **根因不是这一步的改动** ✓（★ 证据：同栈、同一份新代码，一条直线路径落到 **0.249** ✓），
//     而是 ★ 本仓**早已记过**的那条 ★ **定位的 0.061 m 系统偏移** ✓：
//     `docs/frontend-integration.md` §三「到达精度卡在定位的 0.06 m 偏移上」——
//     ★ 真值尺 30 个停点：★ 中位 **0.2325**／p95 **0.2872**／**max 0.3109**／≥0.25 的 **12/30** ✓；
//     ★ 「方向不对称」（到 A 中位 0.1722／到 B 0.2684 ✓）；★ 「要提物理精度 ⇒ **修的是定位**，
//       ✗ 不是容差」✓ ⇒ ★ 那是**另一件事** ✓
//   ★ `≤0.5 m` 这个数取自 ★ 本仓自己的 ★ `scripts/rtk-nav2/frontend-smoke.py` 头注释 ✓
const ARRIVAL_BAR_M = 0.5;
const ring = campus.roads[0].points;
const scale = calibration.meters_per_unit;
const origin = calibration.map_origin_svg;
/** 图上的坐标（SVG）⇒ 米（frame map）。本标定 rotation_rad=0 ⇒ 就是等比缩放 ＋ y 翻转。 */
const toMeters = (p: { x: number; y: number }) => ({
  x: (p.x - origin.x) * scale,
  y: (origin.y - p.y) * scale,
});
const dist = (a: { x: number; y: number }, b: { x: number; y: number }) =>
  Math.hypot(a.x - b.x, a.y - b.y);

const ACTIVE_PHASES = ["queued", "running", "canceling"];
type Status = {
  phase: string;
  message?: string;
  pose: { x: number; y: number; yaw: number } | null;
  cmd_vel_count?: number;
};
const status = (page: Page) => page.evaluate(() => fetch("/api/status").then((r) => r.json())) as Promise<Status>;
const phase = (page: Page) => status(page).then((s) => s.phase);
const open = async (page: Page) => {
  await page.goto("/");
  await page.getByRole("button", { name: "A→B 规划" }).click();
};
const marker = async (page: Page, id: string) => ({
  x: Number(await page.getByTestId(id).getAttribute("data-x")),
  y: Number(await page.getByTestId(id).getAttribute("data-y")),
});
const planAndSend = async (page: Page, goal: { x: number; y: number }) => {
  await choose(page, goal);
  await page.getByTestId("plan-ab").click();
  await expect(page.getByTestId("ab-message")).toContainText("路线已生成");
  await page.getByRole("button", { name: "发送仿真路径" }).click();
};
/** ★ 等到 phase 落到**终态**为止；超时返回 `timeout(<当时的 phase>)` ✓（✗ 不谎报 ✓） */
const waitTerminal = async (page: Page, timeoutMs: number) => {
  const deadline = Date.now() + timeoutMs;
  for (;;) {
    const s = await status(page);
    if (!ACTIVE_PHASES.includes(s.phase)) return s;
    if (Date.now() > deadline) return { ...s, phase: `timeout(${s.phase})` };
    await page.waitForTimeout(300);
  }
};

test("判据 1：A 锁定为车位置、只读；点地图只改 B", async ({ page }) => {
  await open(page);
  await expect(page.getByTestId("a-source")).toContainText("小车当前位置");
  const a0 = await marker(page, "ab-a");
  // ★ A 卡片是**只读**的（不是按钮）＋ 交换按钮禁用 ✓
  expect(await page.getByTestId("select-a").evaluate((el) => el.tagName)).toBe("DIV");
  expect(await page.getByRole("button", { name: "交换 A / B" }).isDisabled()).toBe(true);
  // ★ 点地图 ⇒ 设的是 **B**，A 一动不动 ✓
  await choose(page, ring[2]);
  const b1 = await marker(page, "ab-b");
  // 点选落点有 ≤1 屏幕像素的取整 ⇒ 图上是 1~2 个单位（≈0.25 m）；够证明"B 落在被点的位置"
  expect(dist(b1, ring[2])).toBeLessThan(4);
  expect(dist(await marker(page, "ab-a"), a0)).toBeLessThan(0.2);
  // ★ 点在 A 自己身上 ⇒ 仍然只动 B ✓
  await choose(page, a0);
  expect(dist(await marker(page, "ab-b"), a0)).toBeLessThan(4);
  expect(dist(await marker(page, "ab-a"), a0)).toBeLessThan(0.2);
  // ★ 有 B 就能规划 ✓
  await choose(page, ring[3]);
  await page.getByTestId("plan-ab").click();
  await expect(page.getByTestId("ab-message")).toContainText("路线已生成");
});

test("判据 2：连发 3 条各自 completed ＋ 一条在上一条 running 时发出（覆盖）", async ({ page }) => {
  test.setTimeout(900_000);
  await open(page);
  await expect(page.getByTestId("a-source")).toContainText("小车当前位置");
  const shots: unknown[] = [];
  for (const [i, goal] of [ring[1], ring[2], ring[3]].entries()) {
    const before = await status(page);
    await planAndSend(page, goal);
    await expect.poll(() => phase(page), { timeout: 240_000, intervals: [400] }).toBe("completed");
    const after = await status(page);
    shots.push({
      label: `seq${i + 1}`,
      phase_before: before.phase,
      phase_after: after.phase,
      goal_m: toMeters(goal),
      final_pose: after.pose,
      final_error_m: +dist(after.pose!, toMeters(goal)).toFixed(3),
    });
  }
  // ★ 覆盖：发一条，等它**真的** running，再发下一条 ✓
  await planAndSend(page, ring[4]);
  await expect.poll(() => phase(page), { timeout: 60_000, intervals: [150] }).toBe("running");
  const trace: string[] = [];
  let pumping = true;
  const pump = (async () => {
    while (pumping) {
      try {
        trace.push(await phase(page));
      } catch {
        /* 忽略 */
      }
      await page.waitForTimeout(120);
    }
  })();
  await planAndSend(page, ring[5]);
  await expect.poll(() => phase(page), { timeout: 240_000, intervals: [400] }).toBe("completed");
  pumping = false;
  await pump;
  const after = await status(page);
  shots.push({
    label: "override",
    phase_trace: trace.join(">"),
    saw_cancel: trace.some((p) => p === "canceling" || p === "canceled"),
    goal_m: toMeters(ring[5]),
    final_pose: after.pose,
    final_error_m: +dist(after.pose!, toMeters(ring[5])).toFixed(3),
  });
  console.log("LIVE_RESULT " + JSON.stringify(shots, null, 2));
  for (const s of shots as { final_error_m: number }[]) expect(s.final_error_m).toBeLessThanOrEqual(ARRIVAL_BAR_M);
  expect((shots.at(-1) as { saw_cancel: boolean }).saw_cancel).toBe(true);
});

// ★★ 判据 2′（★ reviewer 2026-10-14 改的口径 ✓）：★ **中止后不重启、直接在界面重发并跑完** ✓
//   ★ 背景 ✓：`FollowPath` 有一条**偶发** `status 6`（★ 与本仓早记的"1/40 偶发"同一条链 ✓：
//     ★ 车"近乎直行"开出 ~4 m ⇒ 越过 `max_robot_pose_search_dist` ≈3.99 m ⇒ `0 poses`
//     ⇒ patience ⇒ Aborting ✓；★ **旧代码上也有** ✗ ⇒ ✗ 不是本步引入的 ✓）
//   ★ 这条用例就是业主的痛点 ✓：★ 中止之后**不用重起栈、不用刷页面**，再点一次"发送仿真路径"就得能跑完 ✓
//   ★ 做法 ✓：故意让**每条腿都掉头**（`ring[2]` ↔ `ring[3]` 来回）⇒ 让那条偶发更容易出现 ✓；
//     一出现 `status 6` ⇒ ★ **原页面直接再发一次** ✓ ⇒ ★ 必须 `completed` ✓
test("判据 2′：中止后不重启、不刷新，直接在界面重发并跑完", async ({ page }) => {
  test.setTimeout(1_800_000);
  await open(page);
  // ★ 来回 ⇒ 每条腿都要掉头 ⇒ 更容易撞上那条偶发
  const legs = [ring[2], ring[3], ring[2], ring[3], ring[2], ring[3]];
  const rows: Record<string, unknown>[] = [];
  for (const [i, goal] of legs.entries()) {
    const before = await status(page);
    await planAndSend(page, goal);
    const first = await waitTerminal(page, 180_000);
    const row: Record<string, unknown> = {
      leg: i + 1,
      goal_m: toMeters(goal),
      first_phase: first.phase,
      first_pose: first.pose,
      first_error_m: first.pose ? +dist(first.pose, toMeters(goal)).toFixed(3) : null,
      cmd_vel_count_before: before.cmd_vel_count,
      cmd_vel_count_after: first.cmd_vel_count,
      moved: (first.cmd_vel_count ?? 0) - (before.cmd_vel_count ?? 0),
    };
    if (first.phase !== "completed") {
      // ★ 中止 ⇒ **同一个页面**、**不刷新**、**不重起栈**，直接再点一次 ✓
      await page.getByRole("button", { name: "发送仿真路径" }).click();
      const again = await waitTerminal(page, 180_000);
      row.resend_phase = again.phase;
      row.resend_error_m = again.pose ? +dist(again.pose, toMeters(goal)).toFixed(3) : null;
      row.resend_message = again.message;
      rows.push(row);
      console.log("RESEND_RESULT " + JSON.stringify(rows, null, 2));
      expect(again.phase, `腿 ${i + 1} 重发后仍未跑完`).toBe("completed");
      expect(row.resend_error_m as number).toBeLessThanOrEqual(ARRIVAL_BAR_M);
      continue;
    }
    rows.push(row);
  }
  console.log("RESEND_RESULT " + JSON.stringify(rows, null, 2));
  const completed = rows.filter((r) => r.first_phase === "completed" || r.resend_phase === "completed");
  expect(completed.length, "完成的腿数").toBeGreaterThanOrEqual(3);
  // ★ 每条腿都得**真的动起来** ✓（reviewer 的口径：`cmd_vel` 要增长 ✓）
  for (const r of rows) expect(r.moved as number, `腿 ${r.leg} 的 cmd_vel 没增长`).toBeGreaterThan(0);
});

// ★★ 判据 2″（定向 ✓）：★ **定向造出一条真 `status 6`** ⇒ ★ 界面**不刷新、不重起栈**直接再发 ⇒ 必须跑完 ✓
//   ★ 为什么要定向 ✗：那条中止**偶发**（本仓记过 1/40 ✓；我连着 6 条、业主也只在第一轮撞上 ✓）
//     ⇒ ★ 光等没法复核 ✓ —— 定向只替换**触发**，被测的仍是"界面能不能直接重发" ✓
//   ★ 触发方式 ✓：往桥发一条**整条都离车 ~20 m** 的两点路径 ⇒
//     ★ 车到路径的距离远超 `FollowPath.max_robot_pose_search_dist`（≈3.9875 m ✓）
//     ⇒ `Resulting plan has 0 poses in it` ⇒ `Controller patience exceeded` ⇒
//     ★ `Abort` ⇒ ★ 桥如实报 `status 6` ✓（★ 本仓 `docs/frontend-integration.md` §四 记过这条链 ✓）
//   ★ ★ 这条**不是**业主撞上的那次（那次是车自己开出去越过的 ✓）—— ★ 但**终态是同一个**
//     （`status 6` ✓）⇒ ★ 验的正是"界面能不能直接重发" ✓
test("判据 2″（定向）：造出一条真 status 6 ⇒ 界面直接再发并跑完", async ({ page }) => {
  test.setTimeout(900_000);
  await open(page);
  const car = (await status(page)).pose!;
  const off = 20; // 路径整体离车 ~20 m ⇒ 远超搜索半径 ≈3.99 m ✓
  const body = {
    frame_id: calibration.frame_id,
    calibration_version: calibration.version,
    calibration_status: calibration.status,
    map_version: calibration.map_version,
    executable_on_robot: false,
    origin_wgs84: calibration.origin_wgs84,
    path: [
      { x: car.x + off, y: car.y },
      { x: car.x + off, y: car.y + 6 },
    ],
    waypoints: [
      { x: car.x + off, y: car.y, yaw: 0 },
      { x: car.x + off, y: car.y + 6, yaw: 0 },
    ],
  };
  const http = await page.evaluate(async (b) => {
    const r = await fetch("/api/routes", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(b),
    });
    return r.status;
  }, body);
  const first = await waitTerminal(page, 180_000);
  const row: Record<string, unknown> = {
    abort_route_http: http,
    car_m: { x: car.x, y: car.y },
    first_phase: first.phase,
    first_message: first.message,
    first_pose: first.pose,
  };
  // ★ 界面直接再发：★ **同一个页面、不刷新、不重起栈** ✓
  const goal = ring[4];
  await planAndSend(page, goal);
  const again = await waitTerminal(page, 240_000);
  row.goal_m = toMeters(goal);
  row.resend_phase = again.phase;
  row.resend_message = again.message;
  row.resend_error_m = again.pose ? +dist(again.pose, toMeters(goal)).toFixed(3) : null;
  console.log("ABORT_RESULT " + JSON.stringify(row, null, 2));
  expect(first.phase, "定向没造出中止 ⇒ 这条用例没验到东西").toBe("failed");
  expect(String(row.first_message)).toContain("status 6");
  expect(again.phase, "界面再发后仍未跑完").toBe("completed");
  expect(row.resend_error_m as number).toBeLessThanOrEqual(ARRIVAL_BAR_M);
});
