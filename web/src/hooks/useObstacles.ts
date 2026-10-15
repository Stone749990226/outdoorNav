import { useEffect, useState } from "react";
import { obstacleRequest, type SceneObstacle } from "../obstacleApi";

export function useObstacles(enabled: boolean) {
  const [items, setItems] = useState<SceneObstacle[]>([]);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  useEffect(() => {
    if (!enabled) {
      setItems([]);
      return;
    }
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      try {
        const response = await obstacleRequest<{ obstacles: SceneObstacle[] }>(
          "",
          "GET",
          undefined,
          controller.signal,
        );
        if (!controller.signal.aborted) setItems(response.obstacles);
      } catch (error) {
        if (!controller.signal.aborted)
          setError(error instanceof Error ? error.message : "障碍物服务不可用");
      } finally {
        if (!controller.signal.aborted) timer = setTimeout(poll, 500);
      }
    };
    void poll();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, [enabled]);
  const operate = async (action: () => Promise<unknown>) => {
    setBusy(true);
    setError("");
    try {
      await action();
      const response = await obstacleRequest<{ obstacles: SceneObstacle[] }>(
        "",
      );
      setItems(response.obstacles);
      return true;
    } catch (error) {
      setError(error instanceof Error ? error.message : "操作失败");
      return false;
    } finally {
      setBusy(false);
    }
  };
  return { items, error, busy, operate };
}
