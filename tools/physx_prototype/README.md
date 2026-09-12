This directory preserves the earlier headless PhysX/CUDA research prototype. It exports the claw-demo field, measures passive CPU/GPU stepping, and audits geometry and joints. It does not execute PROS, motor controllers, tire forces, sensors, or scoring. The application now has a separate [integrated physics backend](../../docs/gpu-physics.md); the measurements here do not establish that backend's throughput or accuracy, and this benchmark runner is not used by the GUI or installer.

Tests on an A100-SXM4-40GB and EPYC 7H12 reproduced the original Bullet slowdown. These are one-second measurements at 240 Hz, excluding loading; the prototype timings include pose readback and validation:

| Scene and solver | Verified CPU workers | Position/velocity iterations | Realtime factor |
| --- | ---: | ---: | ---: |
| Original EERSim, Bullet 3.2.7 | Existing implementation | 100 Bullet iterations | 0.0391x |
| PhysX CUDA | 2 | 100/1 | 0.1880x |
| PhysX CPU, native dispatcher | 2 | 100/1 | 0.1896x |
| PhysX CPU, native dispatcher | 16 | 100/1 | 0.1336x |
| PhysX CUDA experiment | 2 | 32/1 | 0.4064x |
| PhysX CUDA experiment | 2 | 16/1 | 0.6069x |

All five corrected-field snapshots pass the explicit 1 mm / 0.5 degree joint-position checks for all 13 joints. At 100 iterations, maximum GPU forbidden anchor error was 0.43 micrometres. This fixes the previous four broken toggle constraints, but **does not establish equivalent accuracy or an application speedup**. Final loose-piece COM positions differ between CPU and GPU by up to about 145 mm. CPU actors sleep during the run while GPU actors remain active, so the timings include different active workloads. Joint damping is also omitted. Lower iterations remain convergence experiments, not recommended application settings. See the [measurement record](../../docs/physx-prototype-results.json).

Sixteen actual native CPU workers were slower than two in this one-second scene. Earlier comparisons were invalid: the runtime's typed `num_threads` option silently left eight workers. The runner now sets `/persistent/physics/numThreads`, offers `--dispatcher native`, and checks the live worker count before timing. These are single runs on a shared server, not a general thread-scaling result.

The corrected-scene CUDA trace contains roughly 38,000 recorded kernel executions. Most summed kernel time is in TGS constraint solving; kernel-launch overhead is also visible. These sums can overlap and include initialization, so they are not wall-clock percentages or evidence that the GPU is saturated. No supported CUDA-graph switch was found in the pinned runtime.

The scene retains 88 dynamic link actors, 13 passive joints and 4,514 colliders. Oversized pin hulls are split without changing their union; all 4,432 cooked convex instances report GPU compatibility. PhysX's default cooking changed individual hull leaves by up to 0.699 mm. Multiplying raw convex coordinates by 100 and applying a compensating collider scale reduced the measured maximum to 0.02042 mm. **That still fails the experimental 0.010 mm target.** These are per-leaf measurements, not compound-exterior or real-hardware accuracy claims. Cups, static socket meshes and physical dimensions are retained; contact offsets and several solver behaviors remain approximations. See [export details](README-export.md) and the [cooked-geometry audit](README-cooked-geometry.md).

A separate cube free-fall/contact test verified a live CUDA solver, GPU broadphase and DirectGPU on the selected A100. Initialization uses a normal fixed step: `ovphysx==0.5.11`'s explicit `warmup_gpu()` performs a real one-nanosecond step, which caused the initially touching full field to eject objects. A normal step before the first tensor read removed that failure. Four later hinge failures were traced to ignored collision exclusions authored on static parent transforms; authoring those exclusions on the dynamic paddles resolved the measured joint errors.

Run the exporter from a source checkout using the existing EERSim Python environment:

```sh
python -m pip install -r tools/physx_prototype/requirements-export.txt
python tools/physx_prototype/export_field.py --repository . --output build/physx-field --convex-policy split-exact --convex-cook-scale 100
python tools/physx_prototype/validate_export.py build/physx-field
```

The measured runtime is Linux x86-64 with AVX, glibc 2.35 or newer and a CUDA 12.8-compatible NVIDIA driver. The tested driver was 595.71.05. Use a separate environment for the pinned runtime; no administrator access or full Isaac Sim installation is required. Keep the generated field, manifest and `geometry/` directory together when transferring with SCP.

```sh
python3 -m venv build/physx-runtime
build/physx-runtime/bin/python -m pip install -r tools/physx_prototype/requirements-runtime.txt
build/physx-runtime/bin/python tools/physx_prototype/fetch_headers.py build/physx-include
g++ -std=c++17 -O2 -DNDEBUG -fPIC -shared -I build/physx-include tools/physx_prototype/scene_inspect.cpp -o build/scene_inspect.so
g++ -std=c++17 -O2 -DNDEBUG -fPIC -shared -I build/physx-include tools/physx_prototype/cooked_geometry.cpp -o build/cooked_geometry.so
nvidia-smi --query-gpu=index,uuid,name --format=csv
```

Select one available device from that list. A UUID avoids ambiguity between CUDA ordinals and `nvidia-smi` indices; the isolated runtime addresses it as logical device 0.

```sh
CUDA_VISIBLE_DEVICES="<selected-gpu-uuid>" timeout 150s build/physx-runtime/bin/python tools/physx_prototype/benchmark.py --scene build/physx-field/field.usda --work-dir build/physx-gpu --probe-library build/scene_inspect.so --cooked-geometry-library build/cooked_geometry.so
CUDA_VISIBLE_DEVICES='' timeout 150s build/physx-runtime/bin/python tools/physx_prototype/benchmark.py --backend cpu --dispatcher native --scene build/physx-field/field.usda --work-dir build/physx-cpu --probe-library build/scene_inspect.so
```

Both modes perform one initialization step and two warmup steps at the normal timestep before timing 240 steps. `passed` means the runner's basic checks passed; it does not mean valid dynamics. Check `completed_requested_steps`, inspect native warnings, and run the separate audits. Iteration experiments require explicit `--position-iterations`; defaults remain 100/1. Detailed commands and report semantics are in [README-runner.md](README-runner.md).

Use the export environment for offline cooked-geometry validation:

```sh
python tools/physx_prototype/audit_cooked_geometry.py --scene build/physx-field/field.usda --dump build/physx-gpu/cooked_geometry.json --output build/cooked-audit.json --max-error-m 0.00001
python tools/physx_prototype/audit_joints.py --scene build/physx-field/field.usda --poses build/physx-gpu/final_poses.npy --velocities build/physx-gpu/final_velocities.npy --rows build/physx-gpu/pose_rows.json --output build/joint-audit.json
python -m unittest discover -s tools/physx_prototype -p 'test_*.py'
```

The measured field intentionally returns a failed 10-micrometre audit; preserve that result. Generated scenes, caches, headers, environments and reports belong under the ignored `build/` directory. Retain the results you need, then remove only your own temporary remote directory. No machine-specific source paths, SSH hosts, credentials, binary SDKs or generated fields are checked in.

Application acceptance requires URDF articulation and full robot checks, including loaded motor behavior, braking, lift motion, contact-based omni-wheel forces, sensor timing, sleep/wake behavior and the unchanged native claw routine. The new integration renders authoritative PhysX poses, but that implementation is not validated by the passive runs above. Windows CUDA and the GPU-enabled installer remain pending separate validation; follow the current [validation record](../../docs/validation.md).

Primary references: the pinned [PhysX scene setup](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/docs/simulation_setup/physics_scene.md), [tensor bindings](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/docs/tutorials/tensor_bindings.md), [contact bindings](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/docs/tutorials/contact_binding.md), and [Linux platform requirements](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/physx/documentation/platformreadme/linux/README_LINUX.md).
