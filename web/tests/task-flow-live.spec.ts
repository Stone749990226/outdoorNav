import { expect, test } from "@playwright/test";
import { mapToSvg, toMap, bundle } from "../src/map";
import { snap } from "../src/planner";
import { choose } from "./mapHelpers";

test("isolated real frontend and Nav2: arrive, cancel, replace, fail and recover", async ({
  page,
  request,
}) => {
  test.skip(
    process.env.OUTDOORNAV_LIVE_TEST !== "1",
    "Only the Docker isolation harness may run this test",
  );
  const base = "http://127.0.0.1:19865";
  const control = "http://127.0.0.1:19866";
  const scope = await (await request.get(control + "/scope")).json();
  expect(scope).toMatchObject({
    domain: 94,
    localhost_only: true,
    virtual_only: true,
    command_topic: "/isolated_task_flow/cmd_vel",
  });
  const status = async () => (await request.get(base + "/api/status")).json();
  const task = async (id: string) =>
    (await request.get(base + "/api/tasks/" + id)).json();
  const record = async (name: string, value: any) => {
    await request.post(control + "/event", { data: { name, ...value } });
  };
  const multiplier = scope.field_policy ? scope.arrival_tolerance_m / 0.5 : 1;
  const goal = (meters: number) => {
    const raw = {
      x: scope.start.x + meters * multiplier * scope.direction[0],
      y: scope.start.y + meters * multiplier * scope.direction[1],
    };
    // The first surveyed segment curves farther ahead. Select real road points
    // rather than extrapolating outside the UI's allowed 2 m road corridor.
    const projected = snap(mapToSvg(raw));
    return projected ? toMap(projected.point) : raw;
  };
  const waitState = async (id: string, state: string) => {
    await expect
      .poll(async () => (await task(id)).state, {
        timeout: 80000,
        intervals: [200, 500],
      })
      .toBe(state);
    return task(id);
  };
  const execute = async (meters: number) => {
    const target = mapToSvg(goal(meters));
    if (scope.field_policy) {
      // Pan long-range targets into the map instead of clicking outside its
      // viewport or shrinking semantic-place hit targets onto one another.
      const screen = await page.getByTestId('campus-map').evaluate((svg, point) => {
        const p=(svg as SVGSVGElement).createSVGPoint(); p.x=point.x;p.y=point.y;
        const q=p.matrixTransform((svg.querySelector(':scope > g') as SVGGElement).getScreenCTM()!);
        const box=svg.getBoundingClientRect();
        return { x:q.x,y:q.y,cx:box.x+box.width/2,cy:box.y+box.height/2 };
      }, target);
      await page.mouse.move(screen.cx,screen.cy);await page.mouse.down();
      await page.mouse.move(screen.cx+(screen.cx-screen.x),screen.cy+(screen.cy-screen.y),{steps:5});
      await page.mouse.up();
    }
    if (scope.field_policy) {
      console.log('PICK_HIT', meters, await page.getByTestId('campus-map').evaluate((svg, point) => {
        const p=(svg as SVGSVGElement).createSVGPoint();p.x=point.x;p.y=point.y;
        const q=p.matrixTransform((svg.querySelector(':scope > g') as SVGGElement).getScreenCTM()!);
        return {x:q.x,y:q.y,hit:document.elementFromPoint(q.x,q.y)?.outerHTML.slice(0,220)};
      }, target));
    }
    await choose(page, target);
    if (scope.field_policy) console.log('PICK_RESULT', meters, await page.getByTestId('select-b').innerText(), await page.getByTestId('ab-message').innerText());
    const planned = page.waitForResponse(
      (r) => r.url() === base + "/api/plans" && r.request().method() === "POST",
    );
    await page.getByTestId("plan-ab").click();
    expect((await planned).status()).toBe(200);
    await expect(page.getByTestId("ab-message")).toContainText("后端参考路线");
    const response = page.waitForResponse(
      (r) => r.url() === base + "/api/tasks" && r.request().method() === "POST",
    );
    await page
      .getByRole("button", { name: /^(执行导航|替换导航目标)$/ })
      .click();
    const accepted = await response;
    expect(accepted.status()).toBe(202);
    const sent = accepted.request().postDataJSON().goal;
    expect(
      Math.hypot(sent.x - goal(meters).x, sent.y - goal(meters).y),
    ).toBeLessThanOrEqual(2.01);
    return {
      value: await accepted.json(),
      payload: accepted.request().postDataJSON(),
    };
  };

  await page.goto(base);
  if (scope.localization_mode === "rtk_imu") {
    expect(scope.external_odometry_published).toBe(false);
    await expect(page.getByTestId("feedback-source-status")).toContainText(
      "不使用本体里程计",
    );
    await expect(page.getByTestId("wit-heading-status")).toContainText(
      "新 IMU Z 角",
    );
    expect((await status()).velocity_source).toBe("ublox_doppler_and_wit_gyro");
  }
  await expect(page.getByTestId("navigation-status")).toContainText(
    "山猫室外导航就绪",
  );
  if (scope.field_policy) {
    await request.post(control + "/blockage", { data: { blocked: true } });
    // Wait for two global-map update periods, not an arbitrary long timeout.
    await page.waitForTimeout(1200);
  }
  const first = await execute(1.8);
  expect(first.value.state).toBe("accepted");
  const duplicate = await request.post(base + "/api/tasks", {
    data: first.payload,
  });
  expect((await duplicate.json()).task_id).toBe(first.value.task_id);
  if (scope.field_policy) {
    await expect
      .poll(async () => (await task(first.value.task_id)).reason, {
        timeout: 10000,
      })
      .toContain("停车等待");
    const waiting = await task(first.value.task_id);
    expect(waiting.state).toBe("planning");
    const stopped = await (await request.get(control + "/scope")).json();
    expect(Math.abs(stopped.v)).toBeLessThanOrEqual(0.01);
    await request.post(control + "/blockage", { data: { blocked: false } });
    await record("blockage_wait_and_resume", {
      task_id: waiting.task_id,
      waited_without_failing: true,
    });
  }
  const arrived = await waitState(first.value.task_id, "succeeded");
  const at = await (await request.get(control + "/scope")).json();
  const error = Math.hypot(at.x - arrived.goal.x, at.y - arrived.goal.y);
  expect(error).toBeLessThanOrEqual(scope.arrival_tolerance_m + 0.05);
  expect(Math.abs(at.v)).toBeLessThanOrEqual(0.01);
  expect(Math.abs(at.w)).toBeLessThanOrEqual(0.01);
  expect(at.nonzero_commands).toBeGreaterThan(0);
  await expect(page.getByTestId("navigation-status")).toHaveText(
    "已到达并停车",
  );
  await record("arrival", {
    task_id: arrived.task_id,
    final_state: arrived.state,
    error_m: error,
    virtual_speed: [at.v, at.w],
    completion_pose: arrived.completion_pose,
  });

  const canceled = await execute(5.5);
  await waitState(canceled.value.task_id, "navigating");
  await page.getByTitle("取消导航并停车", { exact: true }).click();
  const cancelDone = await waitState(canceled.value.task_id, "canceled");
  const stopped = await (await request.get(control + "/scope")).json();
  expect(Math.abs(stopped.v)).toBeLessThanOrEqual(0.01);
  expect(Math.abs(stopped.w)).toBeLessThanOrEqual(0.01);
  await expect(page.getByTestId("navigation-status")).toHaveText(
    "已取消并停车",
  );
  await record("cancel", {
    task_id: cancelDone.task_id,
    final_state: cancelDone.state,
  });

  const old = await execute(6.0);
  await waitState(old.value.task_id, "navigating");
  const replacement = await execute(3.6);
  expect(replacement.value.task_id).not.toBe(old.value.task_id);
  await expect
    .poll(
      async () => {
        const oldTask = await task(old.value.task_id);
        const newTask = await task(replacement.value.task_id);
        if (["navigating", "succeeded"].includes(newTask.state)) {
          expect(oldTask.state).toBe("canceled");
        }
        return oldTask.state;
      },
      { timeout: 15000 },
    )
    .toBe("canceled");
  const oldDone = await task(old.value.task_id);
  const newDone = await waitState(replacement.value.task_id, "succeeded");
  await record("replace", {
    old_id: oldDone.task_id,
    old_state: oldDone.state,
    new_id: newDone.task_id,
    new_state: newDone.state,
  });

  if (scope.localization_mode === "rtk_imu") {
    const interrupted = await execute(6.5);
    await waitState(interrupted.value.task_id, "navigating");
    await request.post(control + "/heading", {
      data: { valid: true, stream: false },
    });
    expect(
      (await (await request.get(control + "/scope")).json()).heading_stream,
    ).toBe(false);
    const failed = await waitState(interrupted.value.task_id, "failed");
    expect(failed.reason).toMatch(/Localization|unavailable/);
    const lostStatus = await status();
    expect(lostStatus.ready).toBe(false);
    expect(lostStatus.heading.heading_fresh).toBe(false);
    expect(scope.external_odometry_published).toBe(false);
    expect(lostStatus.heading.age_s).toBeGreaterThanOrEqual(0.5);
    expect(lostStatus.heading.gyro_fresh).toBe(true);
    const denied = await request.post(base + "/api/tasks", {
      data: { ...interrupted.payload, request_id: "f".repeat(32) },
    });
    expect(denied.status()).toBe(503);
    const failureStop = await (await request.get(control + "/scope")).json();
    expect(Math.abs(failureStop.v)).toBeLessThanOrEqual(0.01);
    expect(Math.abs(failureStop.w)).toBeLessThanOrEqual(0.01);
    await record("heading_loss", {
      task_id: failed.task_id,
      final_state: failed.state,
      reason: failed.reason,
      heading_stream_dropped: true,
      heading_fresh_after_loss: lostStatus.heading.heading_fresh,
      new_request_status: denied.status(),
    });

    await request.post(control + "/heading", {
      data: { valid: true, stream: true },
    });
    await expect
      .poll(async () => (await status()).ready, { timeout: 12000 })
      .toBe(true);
    const recovery = await execute(5.0);
    const recovered = await waitState(recovery.value.task_id, "succeeded");
    await record("recovery", {
      task_id: recovered.task_id,
      final_state: recovered.state,
    });
  }
  expect(
    (await (await request.get(control + "/scope")).json()).backend_pid,
  ).toBe(scope.backend_pid);

  const blocked = await request.post(base + "/api/plans", {
    data: {
      frame_id: "map",
      scene_version: bundle.scene_version,
      route_version: bundle.route_version,
      goal: goal(6),
      blocked_road_ids: bundle.network.roads.map((r) => r.id),
    },
  });
  expect(blocked.status()).toBe(400);
  await expect(page.getByTestId("navigation-status")).toHaveText(
    "已到达并停车",
  );
  expect((await status()).ready).toBe(true);
  await record("retained_application", {
    blocked_plan_status: blocked.status(),
    backend_pid: scope.backend_pid,
  });
  await page.screenshot({
    path: test.info().outputPath("task-flow-final.png"),
    fullPage: true,
  });
});
