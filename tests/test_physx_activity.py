"""Native diagnostic contract tests; no CUDA, GUI, or simulation is created."""
import ctypes
from pathlib import Path
from types import SimpleNamespace

import pytest

from eersim import physx_verification as verification


class Function:
    def __init__(self, result):
        self.result = result
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        return self.result(*args) if callable(self.result) else self.result


def actor(name=None, kind=1, sleeping=False, links=0, wake=0, linear_squared=0, angular_squared=0):
    return verification._ActivityRow(name, kind, sleeping, links, wake, linear_squared, angular_squared)


def activity_library(monkeypatch, rows, *, flags=11):
    library = SimpleNamespace(
        eersim_scene_gpu_flags=Function(flags), eersim_scene_cuda_device=Function(0),
        eersim_scene_cuda_name=Function(b"Mock CUDA"),
        eersim_probe_physx_version=Function(verification.PHYSX_HEADER_VERSION),
        eersim_activity_abi_version=Function(1),
        eersim_activity_row_size=Function(ctypes.sizeof(verification._ActivityRow)),
        eersim_activity_statistics_size=Function(ctypes.sizeof(verification._ActivityStatistics)))

    def query(pointer, output, capacity, required):
        assert pointer.value == 123
        ctypes.cast(required, ctypes.POINTER(ctypes.c_uint))[0] = len(rows)
        if output is None:
            return 1
        assert capacity == len(rows)
        for index, row in enumerate(rows):
            output[index] = row
        return 0

    def statistics(pointer, output):
        data = ctypes.cast(output, ctypes.POINTER(verification._ActivityStatistics))[0]
        data.cpu_worker_count = 2
        data.geometry_count = 11
        data.active_dynamic_bodies = 3
        data.active_constraints = 5
        data.axis_solver_constraints = 31
        data.discrete_contact_pairs_total = 15
        data.shape_counts[5] = 64
        data.discrete_contact_pairs[5 * 11 + 8] = 7
        data.discrete_contact_pairs[8 * 11 + 5] = 7
        return 0

    library.eersim_scene_activity = Function(query)
    library.eersim_scene_activity_statistics = Function(statistics)
    monkeypatch.setattr(verification, "version", lambda name: "0.5.11")
    monkeypatch.setattr(verification, "probe_library_path", lambda: Path("unused.dll"))
    monkeypatch.setattr(verification.ctypes, "CDLL", lambda path: library)
    monkeypatch.setattr(verification, "_scene_pointer", lambda *args: ctypes.c_void_p(123))
    return library


def test_activity_distinguishes_sleep_from_awake_zero_and_articulation_links(monkeypatch):
    library = activity_library(monkeypatch, [
        actor(b"/World/floor", kind=0),
        actor(b"/World/pin", sleeping=True),
        actor(b"/World/cup", wake=.2),
        actor(b"/World/toggle", linear_squared=.09, angular_squared=4),
        actor(b"/World/robot", kind=2, links=11, linear_squared=.25),
        actor(kind=2, sleeping=True, links=3),
    ])
    result = verification.inspect_scene_activity(object())
    assert result["counts"] == {
        "static_actors": 1, "regular_dynamic_actors": 3, "regular_sleeping": 1,
        "regular_awake": 2, "regular_awake_moving": 1, "regular_awake_zero_velocity": 1,
        "articulations": 2, "articulations_sleeping": 1, "articulations_awake": 1,
        "articulations_awake_moving": 1, "articulations_awake_zero_velocity": 0,
        "articulation_links": 14, "articulation_links_sleeping": 3, "articulation_links_awake": 11,
    }
    assert result["actors"][0]["sleeping"] is None
    assert result["actors"][2]["moving"] is False
    assert result["actors"][2]["sleeping"] is False
    assert result["actors"][-1]["name"] is None
    assert result["max_linear_speed_m_s"] == .5
    assert result["max_angular_speed_rad_s"] == 2
    assert result["cpu_worker_count"] == 2
    stats = result["simulation_statistics"]
    assert stats["shape_counts"]["convex_mesh"] == 64
    assert stats["discrete_contact_pairs_by_geometry"]["convex_mesh"]["triangle_mesh"] == 7
    assert stats["active_constraints"] == 5
    assert len(library.eersim_scene_activity.calls) == 2
    assert len(library.eersim_scene_activity_statistics.calls) == 1


def test_tiny_nonzero_velocity_is_not_classified_as_rest(monkeypatch):
    activity_library(monkeypatch, [actor(linear_squared=1e-80)])
    result = verification.inspect_scene_activity(object())
    assert result["counts"]["regular_awake_moving"] == 1
    assert result["max_linear_speed_m_s"] == 1e-40


def test_empty_scene_is_valid(monkeypatch):
    activity_library(monkeypatch, [])
    result = verification.inspect_scene_activity(object())
    assert result["actors"] == []
    assert not any(result["counts"].values())


def test_existing_gpu_verification_needs_no_new_activity_symbols(monkeypatch):
    library = activity_library(monkeypatch, [])
    del library.eersim_scene_activity
    assert verification.verify_gpu_scene(object())["scene_gpu_flags"] == 11
    with pytest.raises(RuntimeError, match="lacks activity diagnostics"):
        verification.inspect_scene_activity(object())


def test_direct_gpu_does_not_call_unavailable_cpu_state_getters(monkeypatch):
    library = activity_library(monkeypatch, [], flags=15)
    with pytest.raises(RuntimeError, match="DirectGPU getters are unavailable"):
        verification.inspect_scene_activity(object())
    assert not library.eersim_scene_activity.calls


@pytest.mark.parametrize("symbol", ["eersim_activity_abi_version", "eersim_activity_row_size",
                                    "eersim_activity_statistics_size"])
def test_activity_abi_rejected_before_getters(monkeypatch, symbol):
    library = activity_library(monkeypatch, [])
    getattr(library, symbol).result = 999
    with pytest.raises(RuntimeError, match="ABI is incompatible"):
        verification.inspect_scene_activity(object())
    assert not library.eersim_scene_activity.calls


@pytest.mark.parametrize("row,error", [
    (actor(kind=3), "actor type"),
    (actor(links=2), "links for a regular actor"),
    (actor(wake=float("nan")), "nonfinite"),
    (actor(angular_squared=float("inf")), "nonfinite"),
    (actor(linear_squared=-1), "negative motion"),
    (actor(sleeping=True, wake=.2), "inconsistent"),
    (actor(sleeping=True, linear_squared=1e-80), "inconsistent"),
])
def test_invalid_or_inconsistent_native_rows_do_not_become_evidence(monkeypatch, row, error):
    activity_library(monkeypatch, [row])
    with pytest.raises(RuntimeError, match=error):
        verification.inspect_scene_activity(object())


@pytest.mark.parametrize("code", [-1, -2, -3, -4])
def test_native_failures_do_not_return_partial_evidence(monkeypatch, code):
    library = activity_library(monkeypatch, [])
    library.eersim_scene_activity.result = code
    with pytest.raises(RuntimeError):
        verification.inspect_scene_activity(object())


def test_query_rejects_topology_change_between_count_and_fill(monkeypatch):
    library = activity_library(monkeypatch, [])
    def query(pointer, output, capacity, required):
        ctypes.cast(required, ctypes.POINTER(ctypes.c_uint))[0] = 1 if output is None else 0
        return 1 if output is None else 0
    library.eersim_scene_activity.result = query
    with pytest.raises(RuntimeError, match="topology changed"):
        verification.inspect_scene_activity(object())


def test_activity_checks_runtime_before_loading_helper(monkeypatch):
    monkeypatch.setattr(verification, "version", lambda name: "0.6.0")
    monkeypatch.setattr(verification, "probe_library_path", lambda: pytest.fail("helper must not load"))
    with pytest.raises(RuntimeError, match="pinned runtime"):
        verification.inspect_scene_activity(object())
