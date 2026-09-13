# Compare 100 and 32 iterations with the same claw robot

Extract this folder somewhere writable. Close other EERSim simulations, open
PowerShell in this folder, and run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\compare-claw-iterations.ps1
```

This uses the installed **EERSim 0.1.3** application. No Python installation,
administrator access, new installer, or CUDA toolkit is needed. It finds the
registered/default installation automatically. To select an extracted directory
release or another install location, add `-Executable "path\to\EERSim.exe"`.
Keep the complete `_internal` folder beside the executable; a standalone one-file
build is not supported by this recipe.

The script copies the application's bundled claw example into a fresh private
workspace. It uses the bundled claw URDF, full Override field, one red/yellow
preload, fixed starting pose, and the same Skills Auto program in both cases.
It does not use or modify your selected PROS workspace or your editable demo copy.
The **only configuration difference is 100 versus 32 solver iterations**.

Both cases request PhysX GPU explicitly, at **120 Hz**, on logical CUDA device 0.
There is no silent CPU fallback. Use `-PhysicsDevice 1` only if your intended GPU
has that logical index. Existing CUDA device visibility settings still apply.
`-PhysicsHz 240` repeats this matched comparison at 240 Hz instead.

Runs are sequential. Each case has a 120-second wall-time physics budget and can
finish earlier when its 30-second competition period completes. Compilation,
scene loading, hash checks, and cleanup add time. `-WallSeconds 300` increases
the physics budget if either case cannot finish. The script measures headless
physics/PROS throughput; it creates no GUI or rendering probe. Keep other GPU
workloads closed when practical.

Send back **claw-comparison-.../EERSim-claw-iterations-comparison.zip**. It contains
the two diagnostic report ZIPs and a comparison record with settings, executable,
asset, source, profile-template and recipe-script hashes, stopping reasons, and
validation results. Diagnostic
logs can include messages printed by the bundled program. Raw source, profiles,
compiler output, and caches remain under the local `private` folder and are not
added to the comparison ZIP. Nothing uploads automatically. Available reports
are retained even if a case fails; existing files are never deleted.

The script checks requested/actual backends, iteration settings, preload and
workspace selection, recorded PROS calls, and unchanged program/assets. A wall
budget or step limit is explicitly marked as incomplete competition. Completion
does not prove equivalent contact accuracy, gripping, or scoring at 32 iterations.
The reports include final state and clock/tick progress, not a full motion
trajectory; contact and autonomy correctness need separate validation.

For preparation only, use `-PrepareOnly`; this creates the fresh workspace and
profiles without launching EERSim. For a developer's tiny CPU packaging check,
use `-PhysicsBackend physx_cpu -MaxSteps 1`. Step-limited reports are labeled
**partial smoke** and do not count as a completed autonomy run. These options
are not needed for the normal GPU comparison above.
