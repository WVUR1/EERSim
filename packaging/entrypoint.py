"""Windowed PyInstaller entrypoint, including the isolated graphics helper."""
from __future__ import annotations

import multiprocessing
import os
from pathlib import Path
import sys
import traceback


def _show_failure(log_path: Path | None) -> None:
    message = "EERSim could not complete startup."
    if log_path is not None:
        message += f"\n\nDetails were written to:\n{log_path}"
    if os.name == "nt":
        import ctypes
        ctypes.windll.user32.MessageBoxW(None, message, "EERSim startup error", 0x10)


def launch() -> int:
    # Required before normal argument parsing if a bundled dependency starts
    # multiprocessing workers; has no effect on ordinary desktop launches.
    multiprocessing.freeze_support()
    helper = len(sys.argv) > 1 and sys.argv[1] == "--gpu-probe-output"
    log_path = None
    try:
        from eersim.runtime_paths import ensure_console_streams
        log_path = ensure_console_streams()
        if helper:
            if len(sys.argv) != 3:
                print("--gpu-probe-output requires one output path.", file=sys.stderr)
                return 2
            # Never import QApplication or normal CLI parsing in the isolated
            # OpenGL probe. Its JSON file works even with console=False.
            from eersim.gpu_probe import write_probe_result
            return write_probe_result(sys.argv[2])
        from eersim.__main__ import main
        result = main()
        code = 0 if result is None else int(result)
    except SystemExit as exc:
        code = exc.code if isinstance(exc.code, int) else (0 if exc.code is None else 1)
        if code and exc.code and not isinstance(exc.code, int):
            print(exc.code, file=sys.stderr)
    except Exception:
        traceback.print_exc()
        code = 1
    if code and not helper:
        if sys.stderr is not None:
            sys.stderr.flush()
        _show_failure(log_path)
    return code


if __name__ == "__main__":
    raise SystemExit(launch())
