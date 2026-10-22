import { useCallback, useEffect, useState } from "react";
import {
  cancelTask,
  executeGoal,
  getStatus,
  type NavigationStatus,
} from "../navigationApi";
import type { Point } from "../types";

export function useNavigation() {
  const [status, setStatus] = useState<NavigationStatus | null>(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  useEffect(() => {
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    const refresh = async () => {
      try {
        const next = await getStatus(controller.signal);
        if (!controller.signal.aborted) setStatus(next);
      } catch {
        if (!controller.signal.aborted) setStatus(null);
      }
      if (!controller.signal.aborted) timer = setTimeout(refresh, 500);
    };
    void refresh();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, []);
  const execute = useCallback(
    async (goal: Point, blocked: string[], placeId?: string) => {
      setBusy(true);
      setError("");
      try {
        await executeGoal(goal, blocked, placeId);
        setStatus(await getStatus());
      } catch (e) {
        setError(e instanceof Error ? e.message : "导航提交失败");
      } finally {
        setBusy(false);
      }
    },
    [],
  );
  const cancel = useCallback(async () => {
    if (!status?.task) return;
    setBusy(true);
    setError("");
    try {
      await cancelTask(status.task.task_id);
      setStatus(await getStatus());
    } catch (e) {
      setError(e instanceof Error ? e.message : "取消失败");
    } finally {
      setBusy(false);
    }
  }, [status?.task]);
  return { status, error, busy, execute, cancel };
}
