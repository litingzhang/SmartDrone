#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "scripts" / "run_in_px4_gz_container.sh"


class ContainerRenderingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir_context = tempfile.TemporaryDirectory()
        self.temp_dir = Path(self.temp_dir_context.name)
        self.fake_bin = self.temp_dir / "bin"
        self.fake_bin.mkdir()
        self.dri_root = self.temp_dir / "dri"
        self.dri_root.mkdir()
        self.render_node = self.dri_root / "renderD128"
        self.render_node.touch()
        self.second_render_node = self.dri_root / "renderD129"
        self.second_render_node.touch()
        self.drm_sysfs_root = self.temp_dir / "sys" / "class" / "drm"
        self.drm_sysfs_root.mkdir(parents=True)
        self.docker_log = self.temp_dir / "docker.log"
        self._write_fake_commands()

    def tearDown(self) -> None:
        self.temp_dir_context.cleanup()

    def _write_executable(self, name: str, source: str) -> None:
        path = self.fake_bin / name
        path.write_text(textwrap.dedent(source).lstrip(), encoding="utf-8")
        path.chmod(0o755)

    def _write_fake_commands(self) -> None:
        self._write_executable(
            "find",
            """
            #!/usr/bin/env bash
            if [[ -n "${FAKE_RENDER_NODES:-}" ]]; then
                tr ':' '\\n' <<<"$FAKE_RENDER_NODES"
            fi
            """,
        )
        self._write_executable(
            "stat",
            """
            #!/usr/bin/env bash
            printf '109\\n'
            """,
        )
        self._write_executable(
            "docker",
            """
            #!/usr/bin/env bash
            {
                printf 'CALL\\n'
                printf '%s\\n' "$@"
                printf 'END\\n'
            } >>"$FAKE_DOCKER_LOG"
            if [[ "${1:-}" == "image" && "${2:-}" == "inspect" ]]; then
                exit 0
            fi
            for argument in "$@"; do
                if [[ "$argument" == "eglinfo" ]]; then
                    if [[ -n "${FAKE_RUNTIME_STATUS_AFTER_PROBE:-}" ]]; then
                        printf 'error\\n' >"$FAKE_RUNTIME_STATUS_AFTER_PROBE"
                    fi
                    printf 'OpenGL core profile renderer: %s\\n' \
                        "${FAKE_EGL_RENDERER:-llvmpipe}"
                    exit "${FAKE_EGL_STATUS:-0}"
                fi
            done
            exit 0
            """,
        )

    def _run(
        self, rendering: str, renderer: str = "llvmpipe",
        render_nodes: str | None = None, runtime_error_after_probe: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment.update(
            {
                "PATH": f"{self.fake_bin}:{environment['PATH']}",
                "FAKE_DOCKER_LOG": str(self.docker_log),
                "FAKE_EGL_RENDERER": renderer,
                "FAKE_RENDER_NODES": (
                    str(self.render_node) if render_nodes is None else render_nodes
                ),
                "FAKE_RUNTIME_STATUS_AFTER_PROBE": (
                    str(
                        self.drm_sysfs_root / self.render_node.name
                        / "device" / "power" / "runtime_status"
                    )
                    if runtime_error_after_probe else ""
                ),
                "SMART_DRONE_DRI_ROOT": str(self.dri_root),
                "SMART_DRONE_DRM_SYSFS_ROOT": str(self.drm_sysfs_root),
                "SMART_DRONE_GZ_RENDERING": rendering,
                "SMART_DRONE_PX4_GZ_IMAGE": "fake:image",
            }
        )
        environment.pop("DISPLAY", None)
        environment.pop("WAYLAND_DISPLAY", None)
        return subprocess.run(
            [str(RUNNER), "--", "true"],
            cwd=REPO_ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )

    def _docker_arguments(self) -> str:
        return self.docker_log.read_text(encoding="utf-8")

    def _docker_calls(self) -> list[list[str]]:
        calls: list[list[str]] = []
        current: list[str] | None = None
        for line in self.docker_log.read_text(encoding="utf-8").splitlines():
            if line == "CALL":
                current = []
            elif line == "END":
                assert current is not None
                calls.append(current)
                current = None
            elif current is not None:
                current.append(line)
        return calls

    def _set_runtime_status(self, status: str, render_node: Path | None = None) -> None:
        node = render_node or self.render_node
        power_dir = (
            self.drm_sysfs_root / node.name / "device" / "power"
        )
        power_dir.mkdir(parents=True, exist_ok=True)
        (power_dir / "runtime_status").write_text(status + "\n", encoding="utf-8")

    def test_auto_uses_headless_hardware_when_egl_reports_gpu(self) -> None:
        result = self._run("auto", "AMD Radeon RX 6800 (radeonsi)")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("requested=auto selected=hardware", result.stderr)
        self.assertIn("verified=1", result.stderr)
        self.assertIn("renderer=AMD Radeon RX 6800 (radeonsi)", result.stderr)
        arguments = self._docker_arguments()
        self.assertIn("eglinfo", arguments)
        self.assertIn(str(self.render_node), arguments)
        self.assertIn("SMART_DRONE_GZ_RENDERING_SELECTED=hardware", arguments)
        self.assertIn("SMART_DRONE_GZ_RENDERING_REQUESTED=auto", arguments)
        self.assertIn("SMART_DRONE_GZ_RENDERING_REASON=hardware_egl", arguments)
        self.assertIn("SMART_DRONE_GZ_RENDERING_VERIFIED=1", arguments)
        self.assertIn("EGL_PLATFORM=surfaceless", arguments)
        self.assertNotIn("LIBGL_ALWAYS_SOFTWARE=1", arguments)
        self.assertIn(str(self.render_node), self._docker_calls()[-1])

    def test_auto_falls_back_when_egl_reports_llvmpipe(self) -> None:
        result = self._run("auto", "llvmpipe (LLVM 20.1.2, 256 bits)")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("requested=auto selected=software", result.stderr)
        self.assertIn("verified=1", result.stderr)
        arguments = self._docker_arguments()
        self.assertIn("eglinfo", arguments)
        self.assertIn("SMART_DRONE_GZ_RENDERING_SELECTED=software", arguments)
        self.assertIn("SMART_DRONE_GZ_RENDERING_REQUESTED=auto", arguments)
        self.assertIn("LIBGL_ALWAYS_SOFTWARE=1", arguments)
        self.assertIn("MESA_LOADER_DRIVER_OVERRIDE=llvmpipe", arguments)

    def test_auto_skips_node_with_host_runtime_pm_error(self) -> None:
        self._set_runtime_status("error")

        result = self._run("auto", "AMD Radeon RX 6800 (radeonsi)")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("selected=software", result.stderr)
        self.assertIn("reason=host_runtime_pm_error", result.stderr)
        self.assertIn("runtime PM status=error", result.stderr)
        arguments = self._docker_arguments()
        self.assertNotIn("eglinfo", arguments)
        self.assertIn(
            "SMART_DRONE_GZ_RENDERING_REASON=host_runtime_pm_error", arguments,
        )
        self.assertNotIn(str(self.render_node), self._docker_calls()[-1])

    def test_auto_reports_runtime_pm_error_triggered_by_probe(self) -> None:
        self._set_runtime_status("suspended")

        result = self._run(
            "auto", "llvmpipe (LLVM 20.1.2)", runtime_error_after_probe=True,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("reason=host_runtime_pm_error", result.stderr)
        self.assertIn("runtime PM status=error", result.stderr)

    def test_auto_preserves_runtime_pm_reason_when_later_node_probe_fails(self) -> None:
        self._set_runtime_status("error")

        result = self._run(
            "auto", "llvmpipe (LLVM 20.1.2)",
            render_nodes=f"{self.render_node}:{self.second_render_node}",
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("reason=host_runtime_pm_error", result.stderr)

    def test_auto_final_container_exposes_only_selected_node(self) -> None:
        result = self._run(
            "auto", "AMD Radeon RX 6800 (radeonsi)",
            render_nodes=f"{self.render_node}:{self.second_render_node}",
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        final_call = self._docker_calls()[-1]
        self.assertIn(str(self.render_node), final_call)
        self.assertNotIn(str(self.second_render_node), final_call)

    def test_explicit_software_skips_probe(self) -> None:
        result = self._run("software", "AMD Radeon RX 6800 (radeonsi)")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("requested=software selected=software", result.stderr)
        arguments = self._docker_arguments()
        self.assertNotIn("eglinfo", arguments)
        self.assertIn("LIBGL_ALWAYS_SOFTWARE=1", arguments)

    def test_explicit_hardware_skips_probe_and_exposes_dri(self) -> None:
        result = self._run("hardware")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("requested=hardware selected=hardware", result.stderr)
        self.assertIn("verified=0", result.stderr)
        arguments = self._docker_arguments()
        self.assertNotIn("eglinfo", arguments)
        self.assertIn(str(self.render_node), arguments)
        self.assertNotIn("LIBGL_ALWAYS_SOFTWARE=1", arguments)

    def test_explicit_hardware_fails_without_render_node(self) -> None:
        result = self._run("hardware", render_nodes="")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("no render node", result.stderr)

    def test_explicit_hardware_fails_on_host_runtime_pm_error(self) -> None:
        self._set_runtime_status("error")

        result = self._run("hardware")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("runtime PM error state", result.stderr)
        self.assertNotIn("eglinfo", self._docker_arguments())

    def test_explicit_hardware_skips_error_node_and_exposes_next_node(self) -> None:
        self._set_runtime_status("error")
        self._set_runtime_status("active", self.second_render_node)

        result = self._run(
            "hardware",
            render_nodes=f"{self.render_node}:{self.second_render_node}",
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"render_node={self.second_render_node}", result.stderr)
        final_call = self._docker_calls()[-1]
        self.assertNotIn(str(self.render_node), final_call)
        self.assertIn(str(self.second_render_node), final_call)


if __name__ == "__main__":
    unittest.main()
