"""Backend evidence tests use mock SDK calls; they never create CUDA or GUI state."""
import ctypes
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

from eersim import physx_probe_build as build
from eersim import physx_verification as verification


class Function:
    def __init__(self, result):
        self.result = result

    def __call__(self, *args):
        return self.result(*args) if callable(self.result) else self.result


def native_library(flags=11, device=0, header_version=verification.PHYSX_HEADER_VERSION):
    return SimpleNamespace(
        eersim_scene_gpu_flags=Function(flags), eersim_scene_cuda_device=Function(device),
        eersim_scene_cuda_name=Function(b"Test GPU"),
        eersim_probe_physx_version=Function(header_version),
    )


@pytest.mark.parametrize("flags", [11, 15])
def test_gpu_evidence_requires_actual_native_flags_with_optional_direct_api(monkeypatch, flags):
    expected = dict(scene_gpu_flags=flags, logical_cuda_device=0, cuda_device_name="GPU", direct_gpu_api=bool(flags & 4))
    monkeypatch.setattr(verification, "inspect_scene", lambda *args, **kwargs: expected)
    assert verification.verify_gpu_scene(object()) is expected


@pytest.mark.parametrize("flags,device", [(0, -1), (8, 0), (9, 0), (10, 0), (3, 0), (11, -1)])
def test_requested_gpu_cannot_hide_missing_native_components(monkeypatch, flags, device):
    monkeypatch.setattr(verification, "inspect_scene", lambda *args, **kwargs: {
        "scene_gpu_flags": flags, "logical_cuda_device": device})
    with pytest.raises(RuntimeError, match="CPU fallback cannot run as a GPU session"):
        verification.verify_gpu_scene(object())


def test_inspect_uses_public_scene_pointer_and_keeps_scene_path(monkeypatch):
    calls = []

    def get_pointer(handle, path, kind, output):
        calls.append((handle, path, kind))
        ctypes.cast(output, ctypes.POINTER(ctypes.c_void_p))[0] = ctypes.c_void_p(0x123456)
        return SimpleNamespace(status=0)

    library = native_library(flags=0, device=-1)
    library.eersim_scene_cuda_name.result = b""
    monkeypatch.setattr(verification, "version", lambda name: "0.5.11")
    monkeypatch.setattr(verification, "probe_library_path", lambda: Path("unused.dll"))
    monkeypatch.setattr(verification.ctypes, "CDLL", lambda path: library)
    monkeypatch.setitem(sys.modules, "ovphysx", SimpleNamespace(_bindings=SimpleNamespace(
        ovphysx_string_t=ctypes.c_char_p, ovphysx_result_t=ctypes.c_int)))
    # Supply a bytes-accepting test wrapper while retaining ctypes signatures.
    sys.modules["ovphysx"]._bindings.ovphysx_string_t = type("String", (ctypes.Structure,), {
        "_fields_": [("unused", ctypes.c_int)], "__init__": lambda self, text: setattr(self, "text", text)})
    physx = SimpleNamespace(_lib=SimpleNamespace(ovphysx_get_physx_ptr=Function(get_pointer)),
                           _omni_physx_sdk_handle=ctypes.c_uint64(77))
    result = verification.inspect_scene(physx, "/Custom/Scene")
    assert calls[0][0] == 77 and calls[0][1].text == "/Custom/Scene" and calls[0][2] == 1
    assert result["scene_gpu_flags"] == 0 and result["logical_cuda_device"] == -1
    assert library.eersim_scene_gpu_flags.argtypes == [ctypes.c_void_p]


def test_incompatible_helper_rejected_before_scene_vtable_call(monkeypatch):
    library = native_library(header_version=0x05090000)
    monkeypatch.setattr(verification.ctypes, "CDLL", lambda path: library)
    with pytest.raises(RuntimeError, match="incompatible headers"):
        verification._load_library(Path("other.dll"))


def test_runtime_version_is_checked_before_loading_any_native_code(monkeypatch):
    monkeypatch.setattr(verification, "version", lambda name: "0.6.0")
    monkeypatch.setattr(verification, "probe_library_path", lambda: pytest.fail("loaded incompatible runtime"))
    with pytest.raises(RuntimeError, match="pinned runtime"):
        verification.inspect_scene(object())


def test_missing_explicit_library_does_not_fall_back(monkeypatch, tmp_path):
    monkeypatch.setenv("EERSIM_PHYSX_PROBE_LIBRARY", str(tmp_path / "missing.dll"))
    monkeypatch.setattr(verification, "build_probe", lambda *args, **kwargs: pytest.fail("unexpected fallback"))
    with pytest.raises(FileNotFoundError):
        verification.probe_library_path()


def test_frozen_app_never_builds_or_downloads_missing_library(monkeypatch, tmp_path):
    monkeypatch.delenv("EERSIM_PHYSX_PROBE_LIBRARY", raising=False)
    monkeypatch.setattr(verification, "NATIVE", tmp_path)
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(verification, "build_probe", lambda *args, **kwargs: pytest.fail("unexpected build"))
    with pytest.raises(RuntimeError, match="reinstall"):
        verification.probe_library_path()


def test_vendored_header_mutation_is_detected_before_compilation(monkeypatch, tmp_path):
    import hashlib
    import json
    header = tmp_path / "physx/include/test.h"
    header.parent.mkdir(parents=True)
    header.write_text("original")
    (tmp_path / "physx_scene_probe.cpp").write_text("source")
    (tmp_path / "physx/headers-manifest.json").write_text(json.dumps({"tag": build.TAG,
        "sha256": {"test.h": hashlib.sha256(b"original").hexdigest()}}))
    monkeypatch.setattr(build, "NATIVE", tmp_path)
    before = build.source_digest()
    assert len(before) == 20
    header.write_text("changed")
    with pytest.raises(RuntimeError, match="header was modified"):
        build.source_digest()


def test_header_download_rejects_bad_bytes_without_persisting(monkeypatch, tmp_path):
    from io import BytesIO
    monkeypatch.setattr(build.urllib.request, "urlopen", lambda *args, **kwargs: BytesIO(b"wrong payload"))
    with pytest.raises(RuntimeError, match="SHA256"):
        build._download("crt.vsix", tmp_path)
    assert not (tmp_path / "crt.vsix").exists()
