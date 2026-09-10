"""Ready-to-run experiments using ordinary PROS source."""
from pathlib import Path
import shutil
import sys

from .config import ASSETS, default_config


def _demo_workspace() -> Path:
    examples = Path(__file__).resolve().parents[1] / "examples"
    if not examples.is_dir():
        examples = Path(sys.prefix) / "share" / "eersim" / "examples"
    source = examples / "claw_demo"
    if not getattr(sys, "frozen", False):
        return source
    from .runtime_paths import user_data_dir
    if not (source / "src").is_dir() or not (source / "include").is_dir():
        raise RuntimeError("The EERSim release is missing examples/claw_demo/src or include. Reinstall this release.")
    destination = user_data_dir() / "examples" / "claw_demo"
    for original in sorted(source.rglob("*")):
        relative = original.relative_to(source)
        if any(part in {".eersim", ".git", "__pycache__"} for part in relative.parts):
            continue
        if not original.is_file():
            continue
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        try:
            # Exclusive creation also preserves edits if two app instances
            # initialize their shared example workspace at the same time.
            with target.open("xb") as output, original.open("rb") as input_file:
                shutil.copyfileobj(input_file, output)
        except FileExistsError:
            pass
    return destination


def claw_demo_config() -> dict:
    config = default_config()
    config.update(
        robot_urdf=str(ASSETS / "robots" / "xdrive_claw.urdf"),
        robot_config=str(ASSETS / "robots" / "xdrive_claw.json"),
        game_pack=str(ASSETS / "games" / "override-2026-27.json"),
        workspace=str(_demo_workspace()),
        motors=None,
        motor_overrides={},
        start_pose=[-1.45, -.90, .085, 0],
        field_objects=[{
            "name": "demo_preload", "urdf": str(ASSETS / "objects" / "pin_red_yellow.urdf"),
            "fixed": False, "position": [-1.11, -.90, .083],
            "kind": "pin", "colors": ["red", "yellow"],
        }],
        notes="One red/yellow preload inside the open claw. Skills Auto picks it up and places it in the near red goal.",
        duration_overrides={"skills_auto": 30, "skills_driver": 60, "match_auto": 30, "driver_auto": 90},
    )
    return config
