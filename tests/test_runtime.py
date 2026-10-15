import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import uuid

from outdoornav.runtime.environment import render_environment
from outdoornav.runtime.processes import birth, identity, matches, owned, stop_owned
from outdoornav.runtime.run import check_port, domain_available, select_domain
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


if __name__ == "__main__":
    unittest.main()
