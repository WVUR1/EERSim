# Native PhysX scene verification

`physx_scene_probe.cpp` calls public virtual PhysX SDK methods on the scene
returned by the pinned ovphysx 0.5.11 runtime. It reports actual GPU dynamics,
broadphase, Direct GPU API, CUDA context validity, logical device and device
name. Requesting GPU configuration alone is insufficient evidence of GPU use.

The 64 public headers in `physx/include` are an unmodified dependency closure
from NVIDIA's `ovphysx-0.5.11` tag (630,959 bytes). The upstream BSD license and
per-header SHA256 provenance are included. Do not mix this helper with another
ovphysx version: a matching PhysX version number alone does not establish ABI
compatibility across different runtime releases.

Windows runtime binaries use the Microsoft x64 C++ ABI. The build explicitly
targets that ABI even when using LLVM-MinGW's Clang; compiling the helper with
MinGW's default C++ ABI is incorrect. The DLL has no external library imports.
It is linked without a DLL entry point or CRT, since all calls use existing
scene/context objects. Linux builds use the installed C++ compiler and matching
public SDK headers; no full PhysX/CUDA SDK build or administrator access is needed.

Build from the repository root:

```powershell
python -m eersim.physx_probe_build --output build/native/physx_scene_probe.dll
```

```sh
python -m eersim.physx_probe_build --output build/native/physx_scene_probe.so
```

`--cxx` or `EERSIM_PHYSX_CXX` selects a compiler executable. On Windows the normal
setup LLVM toolchain is discovered. Microsoft C++ and UCRT standard headers are
downloaded from official Microsoft URLs, SHA256 verified, and extracted into
the build cache (about 2.7 MB total download). This reads an MSI file table; it
does not install the MSI or modify system settings. Existing headers can be used
with `--msvc-include` and `--ucrt-include`. These Microsoft build inputs are not
included in the repository or release. `--cache` controls their cache location.

Windows releases include the prebuilt DLL; end users need no build or download.
Source checkouts build a missing helper in their per-user EERSim data directory
at the first explicit PhysX use. `EERSIM_PHYSX_PROBE_LIBRARY` can select an
already built matching library, including on a compute host without a compiler.

The inspector does not initialize CUDA, advance simulation, or own SDK objects.
Call it after the first normal-timestep bootstrap; do not invoke the SDK's 1 ns
GPU warmup merely to inspect a scene. Host-readback GPU scenes require bits 1,
2 and 8 (11); Direct GPU API adds bit 4 (15) and is optional.
