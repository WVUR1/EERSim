"""A missing GPU must preserve a usable viewport and identical physics state."""
import numpy as np

from eersim.config import ASSETS, default_config
from eersim.physics import World
from eersim import render_backend


def test_unavailable_gpu_falls_back_without_advancing_or_resampling_world(monkeypatch):
    monkeypatch.delenv("EERSIM_RENDERER", raising=False)
    probes = []

    def unavailable():
        probes.append(True)
        return {"available": False, "hardware_accelerated": False, "error": "Test driver unavailable"}

    monkeypatch.setattr(render_backend, "probe_gpu", unavailable)
    config = default_config()
    config.update(game_pack=str(ASSETS / "games" / "practice.json"), renderer="auto")
    config["noise"] = {"position_std_m": .03, "heading_std_deg": .5}
    with World(config) as world:
        before = world.snapshot()
        first = world.render(160, 120)
        world.render(160, 120, yaw=15)
        assert first.shape == (120, 160, 4)
        assert first.dtype == np.uint8
        assert np.unique(first[:, :, :3].reshape(-1, 3), axis=0).shape[0] > 10
        assert world.snapshot() == before
        assert world.renderer_status["backend"] == "cpu"
        assert "Test driver unavailable" in world.renderer_status["reason"]
        assert len(probes) == 1
        world.reset(config)
        world.render(160, 120)
        assert len(probes) == 2
