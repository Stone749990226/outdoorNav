import os
import io
import json
import signal
from contextlib import ExitStack
from types import SimpleNamespace
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import uuid

from outdoornav.runtime.environment import render_environment
from outdoornav.runtime.processes import birth, identity, matches, owned, stop_owned
from outdoornav.runtime.run import check_port, domain_available, select_domain, start
import socket


class RuntimeTests(unittest.TestCase):
    def test_pid_identity_includes_start_time(self):
        current = identity(os.getpid())
        self.assertTrue(matches(current))
        self.assertFalse(matches({**current, "birth": "not-this-process"}))
        self.assertIsNone(birth(999999999))

    def test_stop_targets_only_exact_owned_run(self):
        run_id = "test-" + uuid.uuid4().hex
        child = subprocess.Popen(
            ["/bin/sleep", "30"],
            env={**os.environ, "OUTDOORNAV_RUN_ID": run_id},
            start_new_session=True,
        )
        unrelated = subprocess.Popen(["/bin/sleep", "30"], start_new_session=True)
        try:
            self.assertEqual([p["pid"] for p in owned(run_id)], [child.pid])
            stop_owned(run_id, grace=0.2)
            child.wait(timeout=2)
            self.assertIsNone(unrelated.poll())
        finally:
            for process in (child, unrelated):
                if process.poll() is None:
                    process.terminate()
                process.wait(timeout=2)

    def test_native_renderer_does_not_inject_wsl_variables_and_auto_falls_back(self):
        calls = []

        def context(command, env, **kwargs):
            calls.append(env.copy())
            return subprocess.CompletedProcess(
                command,
                0,
                "GL_ARB_copy_image"
                if env["LIBGL_ALWAYS_SOFTWARE"] == "1"
                else "unsupported",
                "",
            )

        with (
            tempfile.TemporaryDirectory() as directory,
            patch("outdoornav.runtime.environment.is_wsl", return_value=False),
            patch(
                "outdoornav.runtime.environment.shutil.which",
                return_value="/usr/bin/glxinfo",
            ),
            patch("outdoornav.runtime.environment.subprocess.run", side_effect=context),
        ):
            env, renderer = render_environment(
                {"PATH": "/usr/bin"}, "auto", Path(directory)
            )
            self.assertEqual(renderer, "software")
            self.assertEqual(len(calls), 2)
            self.assertNotIn("MESA_D3D12_DEFAULT_ADAPTER_NAME", env)
            self.assertEqual(env["GALLIUM_DRIVER"], "llvmpipe")

    def test_forced_hardware_does_not_silently_fall_back(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch("outdoornav.runtime.environment.is_wsl", return_value=False),
            patch(
                "outdoornav.runtime.environment.shutil.which",
                return_value="/usr/bin/glxinfo",
            ),
            patch(
                "outdoornav.runtime.environment.subprocess.run",
                return_value=subprocess.CompletedProcess([], 0, "unsupported", ""),
            ),
        ):
            with self.assertRaises(RuntimeError):
                render_environment({"PATH": "/usr/bin"}, "hardware", Path(directory))


class RuntimeFailureTests(unittest.TestCase):
    def test_domain_selection_rejects_invalid_or_occupied_domains(self):
        for value in ("bad", "-1", "233"):
            with self.assertRaises(ValueError):
                select_domain(value)
        with patch("outdoornav.runtime.run.domain_available", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "occupied"):
                select_domain("23")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as owner:
            domain = next(d for d in range(80, 102) if domain_available(d))
            owner.bind(("127.0.0.1", 7400 + 250 * domain + 10))
            self.assertFalse(domain_available(domain))

    def test_corrupted_runtime_record_is_not_treated_as_a_live_process(self):
        from outdoornav.runtime.state import active

        with tempfile.TemporaryDirectory() as directory:
            record = Path(directory) / "active.json"
            record.write_text("{broken")
            with patch("outdoornav.runtime.state.ACTIVE", record):
                self.assertIsNone(active())

    def test_busy_port_is_rejected_without_disturbing_its_owner(self):
        with socket.socket() as owner:
            owner.bind(("127.0.0.1", 0))
            owner.listen()
            port = owner.getsockname()[1]
            with self.assertRaisesRegex(RuntimeError, "busy"):
                check_port(port)
            self.assertEqual(owner.getsockname()[1], port)
        check_port(port)

    def test_stale_supervisor_identity_does_not_match_reused_pid(self):
        import json
        from outdoornav.runtime.state import active

        with tempfile.TemporaryDirectory() as directory:
            record = Path(directory) / "active.json"
            record.write_text(
                json.dumps(
                    {
                        "supervisor": {"pid": os.getpid(), "birth": "stale"},
                        "run_id": "old",
                    }
                )
            )
            with patch("outdoornav.runtime.state.ACTIVE", record):
                self.assertIsNone(active())


class SupervisorHealthTests(unittest.TestCase):
    """Exercise the real startup/watchdog/finally flow without launching an app."""

    def supervise(self, status_at, duration=45, startup_ready=True):
        with tempfile.TemporaryDirectory() as directory, ExitStack() as patches:
            root = Path(directory)
            runs = root / "runs"
            active = runs / "active.json"
            stamp = root / "stamp.json"
            stamp.write_text("{}")
            install = root / "setup.bash"
            install.touch()
            clock = [100.0]
            signals = {}
            snapshots = []
            manual_stop = [False]
            shutdowns = []
            runtime_reads = []

            def patch_run(name, **kwargs):
                return patches.enter_context(patch("outdoornav.runtime.run." + name, **kwargs))

            def sleep(seconds):
                clock[0] += seconds
                if active.exists():
                    snapshot = json.loads(active.read_text())
                    self.assertEqual(
                        snapshot,
                        json.loads((Path(snapshot["directory"]) / "run.json").read_text()),
                    )
                    snapshots.append(snapshot)
                if clock[0] >= 100 + duration:
                    manual_stop[0] = True
                    signals[signal.SIGINT]()

            def response(url, **kwargs):
                if ":8765/" in url:
                    elapsed = clock[0] - 100
                    runtime_reads.append(elapsed)
                    status = status_at(elapsed)
                    if isinstance(status, Exception):
                        raise status
                else:
                    status = {"ready": startup_ready}
                return io.StringIO(json.dumps(status))

            def spawn(command, **kwargs):
                health = command[:4] == ["ros2", "run", "outdoornav_ros", "health"]
                code = 0 if health else None
                return SimpleNamespace(
                    pid=123, returncode=code, poll=lambda: code, wait=lambda **kw: code
                )

            def stop(run_id):
                shutdowns.append(manual_stop[0])

            patch_run("RUNS", new=runs)
            patch_run("ACTIVE", new=active)
            patch_run("STAMP", new=stamp)
            patch_run("INSTALL", new=install)
            patch_run("check_port")
            patch_run("select_domain", return_value="23")
            patch_run("ensure_scene")
            patch_run("load_bundle", return_value={"scene_version": "test", "route_version": "test"})
            patch_run("fingerprints", return_value={})
            patch_run("build", side_effect=AssertionError("must not build"))
            patch_run("ros_environment", return_value={})
            patch_run("render_environment", return_value=({"PATH": "/usr/bin", "RMW_IMPLEMENTATION": "test"}, "software"))
            patch_run("identity", return_value={"pid": 123, "birth": "test"})
            patch_run("subprocess.check_output", return_value="test-revision")
            popen = patch_run("subprocess.Popen", side_effect=spawn)
            patch_run("signal.signal", side_effect=lambda sig, handler: signals.update({sig: handler}))
            patch_run("time.monotonic", side_effect=lambda: clock[0])
            patch_run("time.time", side_effect=lambda: clock[0])
            patch_run("time.sleep", side_effect=sleep)
            patch_run("urlopen", side_effect=response)
            patch_run("aim_camera")
            patch_run("stop_owned", side_effect=stop)
            printed = patches.enter_context(patch("builtins.print"))
            error = None
            try:
                start(SimpleNamespace(port=5174, render="software", rviz=True))
            except RuntimeError as caught:
                error = caught
            records = list(runs.glob("*/run.json"))
            final = json.loads(records[0].read_text())
            return SimpleNamespace(
                snapshots=snapshots, final=final, shutdowns=shutdowns,
                error=error, reads=runtime_reads, spawns=popen.call_count,
                messages=[call.args[0] for call in printed.call_args_list],
                active_exists=active.exists(),
            )

    def test_healthy_terminal_tasks_wait_for_manual_shutdown(self):
        for terminal in ("succeeded", "failed", "canceled"):
            with self.subTest(terminal=terminal):
                status = {"ready": True, "health": {"controller_stopped": True}, "task": {"state": terminal}}
                result = self.supervise(lambda elapsed: status)
                self.assertIsNone(result.error)
                self.assertEqual(result.shutdowns, [True])
                self.assertGreater(max(result.reads), 40)
                self.assertEqual(result.spawns, 5)  # no retries/relaunch
                self.assertTrue(all(s["ready"] for s in result.snapshots))
                self.assertTrue(all(not s["degraded"] for s in result.snapshots))
                self.assertEqual(result.final["last_navigation_status"], status)
                self.assertNotIn("failure", result.final)

    def test_stop_failure_keeps_app_alive_and_readiness_false(self):
        status = {"ready": False, "health": {"controller_stopped": False}, "task": {"state": "failed", "reason": "stop failed"}}
        result = self.supervise(lambda elapsed: status)
        self.assertIsNone(result.error)
        self.assertEqual(result.shutdowns, [True])
        self.assertGreater(max(result.reads), 40)  # beyond both old watchdog budgets
        degraded = [s for s in result.snapshots if s.get("degraded")]
        self.assertTrue(degraded)
        self.assertTrue(all(not s["ready"] for s in degraded))
        self.assertIn("controller_stopped", degraded[-1]["degraded_reason"])
        self.assertEqual(degraded[-1]["last_navigation_status"], status)
        self.assertNotIn("failure", result.final)
        self.assertEqual(sum("keeping app alive" in m for m in result.messages), 1)
        self.assertFalse(result.active_exists)

    def test_recovery_clears_degraded_metadata_in_both_records(self):
        unhealthy = {"ready": False, "health": {"controller_stopped": False}}
        healthy = {"ready": True, "health": {"controller_stopped": True}, "task": {"state": "failed"}}
        result = self.supervise(lambda elapsed: unhealthy if elapsed < 20 else healthy)
        self.assertIsNone(result.error)
        self.assertEqual(result.shutdowns, [True])
        self.assertTrue(any(s.get("degraded") for s in result.snapshots))
        recovered = result.snapshots[-1]
        self.assertTrue(recovered["ready"])
        for record in (recovered, result.final):
            self.assertFalse(record["degraded"])
            self.assertNotIn("degraded_reason", record)
            self.assertNotIn("degraded_since", record)
            self.assertEqual(record["last_navigation_status"], healthy)
        self.assertEqual(result.messages.count("Navigation readiness recovered"), 1)

    def test_navigation_api_loss_is_degraded_not_a_shutdown(self):
        result = self.supervise(lambda elapsed: OSError("API unavailable"))
        self.assertIsNone(result.error)
        self.assertEqual(result.shutdowns, [True])
        self.assertGreater(max(result.reads), 40)
        self.assertFalse(result.snapshots[-1]["ready"])
        self.assertTrue(result.snapshots[-1]["degraded"])
        self.assertEqual(result.snapshots[-1]["last_navigation_status"], {
            "ready": False, "health": {"api": False},
        })

    def test_startup_still_fails_at_readiness_deadline(self):
        # False readiness responses also advance this fake clock, exercising the
        # same bounded web startup path as a server that never becomes ready.
        from outdoornav.runtime import run
        real_load = run.json.load

        def load_and_tick(stream):
            value = real_load(stream)
            if value == {"ready": False}:
                run.time.sleep(1)
            return value

        with patch("outdoornav.runtime.run.json.load", side_effect=load_and_tick):
            result = self.supervise(lambda elapsed: {"ready": False}, duration=300, startup_ready=False)
        self.assertRegex(str(result.error), "Web proxy did not become ready")
        self.assertEqual(result.shutdowns, [False])
        self.assertFalse(result.final["ready"])
        self.assertIn("failure", result.final)


if __name__ == "__main__":
    unittest.main()
