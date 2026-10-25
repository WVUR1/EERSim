"""Desktop launcher and reproducible headless runs."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path


def _show_startup_error(message):
    """A windowed Windows executable must explain failures even without Qt.

    Headless entry points never call this. On other platforms, use an existing
    Qt application only; initializing a broken display plugin can abort Python.
    """
    from .runtime_paths import user_data_dir
    details = ("EERSim could not start.\n\n" + str(message) +
               "\n\nFor an installed copy, try reinstalling the complete application. "
               "For a source checkout, run the setup script in the repository to install its dependencies." +
               "\n\nWindowed executable logs: " + str(user_data_dir() / "logs" / "eersim.log"))
    try:
        if sys.platform == "win32":
            import ctypes
            ctypes.windll.user32.MessageBoxW(None, details, "EERSim startup error", 0x10)
        else:
            from PySide6.QtWidgets import QApplication, QMessageBox
            if QApplication.instance() is not None:
                QMessageBox.critical(None, "EERSim startup error", details)
    except Exception:
        # The caller has already printed the original failure to stderr.
        pass


def main(argv=None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments and arguments[0] == "--gpu-probe-output":
        if len(arguments) != 2:
            return 2
        from .gpu_probe import write_probe_result
        return write_probe_result(arguments[1])
    if '--diagnostics' in arguments or '--diagnostic-case' in arguments:
        from .diagnostics import main as diagnostics_main
        return diagnostics_main(arguments)
    from .runtime_paths import ensure_console_streams
    ensure_console_streams()
    import argparse
    parser = argparse.ArgumentParser(description="EERKit Autonomy Lab")
    parser.add_argument("--headless", action="store_true", help="Run without the desktop")
    parser.add_argument("--renderer", choices=["auto", "gpu", "cpu"], help="Graphics renderer; independent of the physics engine")
    parser.add_argument("--physics-backend", choices=["auto", "mujoco", "bullet", "physx_gpu", "physx_cpu"],
                        help="Physics engine: auto selects MuJoCo CPU, with a reported Bullet fallback if unavailable; explicit selections do not fall back")
    parser.add_argument("--physics-device", type=int,
                        help="Logical CUDA device index (0 or greater, after CUDA_VISIBLE_DEVICES filtering)")
    parser.add_argument("--solver-iterations", type=int,
                        help="Physics solver iterations (1–255; default 100)")
    parser.add_argument("--workspace", type=Path, help="PROS workspace to compile")
    experiments = parser.add_mutually_exclusive_group()
    experiments.add_argument("--config", type=Path, help="Saved simulator configuration")
    experiments.add_argument("--demo", choices=["claw"], help="Load the X-drive pick-and-place experiment")
    parser.add_argument("--mode", choices=["skills_driver", "skills_auto", "match_auto", "driver_auto"], default="skills_auto")
    parser.add_argument("--seconds", type=float, default=5.0, help="Headless competition duration in simulated seconds")
    parser.add_argument("--csv", type=Path, help="Export ground-truth pose telemetry")
    parser.add_argument("--controller", type=Path, help="JSON analog/digital controller values for headless driver runs")
    args = parser.parse_args(arguments)
    try:
        from .config import SimConfig, default_config, load_config
        if args.demo:
            from .demos import claw_demo_config
            config = claw_demo_config()
        else:
            config = load_config(args.config) if args.config else default_config()
        if args.workspace:
            config["workspace"] = str(args.workspace.resolve())
        if args.renderer:
            config["renderer"] = args.renderer
        if args.physics_backend is not None:
            config["physics_backend"] = args.physics_backend
        if args.physics_device is not None:
            config["physics_device"] = args.physics_device
        if args.solver_iterations is not None:
            config["physics_solver_iterations"] = args.solver_iterations
        config = SimConfig.from_dict(config).to_dict()
        if not args.headless:
            from .gui import run
            return run(config, str(args.workspace) if args.workspace else None)
        if not math.isfinite(args.seconds) or args.seconds <= 0:
            parser.error("--seconds must be positive and finite")
        from .runner import Session
        if hasattr(config, "to_dict"):
            config = config.to_dict()
        config.setdefault("duration_overrides", {})[args.mode] = args.seconds
        session = Session(config, lambda line: print(line, file=sys.stderr))
        try:
            if config.get("workspace"):
                from .bridge import build_workspace
                session.start(args.mode, build_workspace(Path(config["workspace"]), lambda line: print(line, file=sys.stderr)))
                if args.controller:
                    session.controller = json.loads(args.controller.read_text(encoding="utf-8"))
                while session.running:
                    session.step()
            else:
                for _ in range(math.ceil(args.seconds / session.dt)):
                    session.step()
            if args.csv:
                session.export_csv(args.csv)
            print(json.dumps(session.state(), indent=2, default=str))
        finally:
            session.close()
        return 0
    except (RuntimeError, ValueError, OSError, ImportError) as exc:
        print(f"EERKit simulator: {exc}", file=sys.stderr)
        if not args.headless:
            _show_startup_error(exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
