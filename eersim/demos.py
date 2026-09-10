"""Ready-to-run experiments using ordinary PROS source."""
from pathlib import Path
import sys

from .config import ASSETS, default_config


def claw_demo_config() -> dict:
    examples = Path(__file__).resolve().parents[1] / "examples"
    if not examples.is_dir():
        examples = Path(sys.prefix) / "share" / "eersim" / "examples"
    config = default_config()
    config.update(
        robot_urdf=str(ASSETS / "robots" / "xdrive_claw.urdf"),
        robot_config=str(ASSETS / "robots" / "xdrive_claw.json"),
        game_pack=str(ASSETS / "games" / "override-2026-27.json"),
        workspace=str(examples / "claw_demo"),
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
