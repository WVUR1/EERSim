# Tools

## Physics performance

From the activated source environment, run `python tools/benchmark_physics.py`.
It measures the full claw field without opening windows or executing robot
code, reporting fixed-step throughput and contact counts. The default wall
budget is 30 seconds after loading; it stops at a complete physics tick.

Use `--practice` for the simple field, `--config experiment.json` for a saved
scene, `--json report.json` to save results, and `--profile timing.prof` for
optional cProfile data. See [validation notes](../docs/validation.md#full-field-physics-profile)
for measured limits and interpretation. This diagnostic requires the source
Python environment.

## Import the official VEX CAD

The bundled Override visual meshes are derived from VEX Robotics' official
2026–2027 field STEP assembly. OpenCASCADE measures the BReps and tessellates
their actual surfaces. This tool does not infer shapes from pictures.

Install the optional conversion dependencies in a separate environment:

    python -m pip install -r tools/requirements-cad.txt
    New-Item -ItemType Directory -Force .sim-tools | Out-Null
    Invoke-WebRequest https://link.vex.com/docs/26-27/v5rc/field-cad -OutFile .sim-tools/override-official-cad.zip
    python tools/import_vex_cad.py --archive .sim-tools/override-official-cad.zip --selection tools/override-cad-selection.json

Run these commands from the EERSim repository root. The tools' default cache,
mesh and manifest paths are anchored to that root using their script location,
so invoking a tool from another working directory does not redirect its outputs.
Explicit relative command-line paths remain relative to the working directory.

For an already extracted STEP, supply --step followed by its path, as well as
--archive to retain the archive checksum in the manifest. The --step path takes
precedence as the actual CAD input.

The first run reads the STEP and writes an ignored binary XCAF document plus
individual BREP caches in .sim-tools/official-cad-cache. Later catalogue runs
reuse the XCAF document; --reuse-catalog skips both STEP and XCAF parsing and
uses the measured catalogue and BREP files directly. Cached catalogues are
checked against the source STEP's SHA-256.

After adding selections, --reuse-assets also verifies each existing OBJ's
SHA-256, source checksum and exact selection before reusing its tessellation.
The pinned OCP 8.0.1.0.0 distribution uses VTK 9.6.2. On machines with a large
CPU count, set OPENBLAS_NUM_THREADS=1 and OMP_NUM_THREADS=1 before launching
the converter to avoid excessive numerical-library worker creation.

To inspect a new season before selecting its parts:

    python tools/import_vex_cad.py --archive .sim-tools/override-official-cad.zip --catalog-only

The cache's catalog.json lists named CAD definitions, conservative bounding
boxes, and every assembly occurrence's exact 4×4 placement matrix. Translation
components are in millimetres. Exported selected parts additionally use precise
OpenCASCADE surface extrema and report analytical cylinder/cone radii and
depth extents, which inform the separately authored collision shapes.

override-cad-selection.json selects exact definition names and output keys.
The axis property identifies the authored CAD axis that becomes simulator +Z:

| CAD up axis | CAD coordinates mapped to simulator axes |
| --- | --- |
| z | X, Y, Z |
| y | X, −Z, Y |
| x | Y, Z, X |

Every mapping preserves handedness. Lengths convert from mm to metres.
The base origin centers X/Y and places the assembly's lower extent at Z=0.
The center origin centers all three dimensions. The transformation and exact
offset are recorded in assets/cad_manifest.json. The split_halves
option groups the assembly's complete authored solids above/below its axial
midpoint, preserving the original surfaces. Both half meshes use the same
complete-assembly origin, so a URDF can recolor the game's two ends without
changing their geometry.

An explicit origin_offset_mm overrides bounding-box centering in the oriented
CAD frame. The pin retains its real symmetry axis even though its D-shaped
flange makes the bounds asymmetric. The cup's base is 82.25 mm below its
assembly origin. Goal origins align to the foam floor; mounting screws and
plates legitimately extend below Z=0. Do not recenter these exported meshes.

The component_indices option selects whole located CAD components in original
assembly order. The toggle_fixed and toggle_rotor selections partition the
authored assembly without cutting surfaces. Their shared frame, real pivot
and axis appear in the manifest's mechanisms.toggle record. Joint angle zero
means the original CAD pose, and the rotating visual is offset by the negative
pivot relative to its joint frame. The plain CAD bar does not include printed
game-color graphics; the simulator supplies those colors separately.

OBJ meshes use a 0.25 mm chord tolerance and smooth per-face vertex normals.
Companion MTLs supply default colors; game URDFs assign alliance/end colors.
Tessellated visuals are separate from collision geometry. The manifest's
analytic_surfaces records include cylinders and cones from the exact STEP;
their coordinates remain in the original part frame in mm.

The measured_profiles records intersect the exported triangle surfaces with
axial planes, reporting outer and inner surface radii and XY bounds. These are
tessellation measurements at the stated chord tolerance, rather than exact
BRep cross-sections. Small fasteners, recesses and ribs can contribute radial
extrema, so these profiles are evidence for collision design, not ready-made
solid cylinder dimensions. goal_socket preserves the actual goal body and
tapered opening while excluding the assembly's mounting hardware.

For collision construction, distinguish a sampled plane from the part's end.
The cup's Y=80 mm sample is 2.25 mm below its rim. The exact inner lip cone
extends from Y=73.789140953 to 82.25 mm, with radius in mm equal to
34.326978255 + (Y - 73.875) * tan(30 degrees). Its rim radius is therefore
39.162286760 mm. The outer radius at that rim is 40.098507558 mm. Filling the
rim to the smaller Y=80 sample radius creates a false interference with a pin.
The outer wall's 1-degree rim cone exists only from Y=80.830177587 to
82.25 mm; its radius is 40.146508987 + (Y - 85) * tan(1 degree).
Before its rounded shoulder, the 30-degree outer flare spans
Y=72.914140953 to 79.865082500 mm. Do not extrapolate the narrow rim cone
over the entire lip: these are separate trimmed CAD surfaces.
The cup's exported base coordinate is Z=(Y+82.25)/1000 metres. These equations
come from the manifest's analytical cone records, without tessellation error.

field_placements converts every game assembly and white panel into the world
frame: (CAD_X, -CAD_Z, CAD_Y + 294.5892) / 1000. The floor offset comes from
the top face of the installed foam tiles in the source assembly. Positions
include each mesh's normalization offset; rpy uses URDF fixed-axis radians.
The asset_key and mesh fields identify the actual variant. In particular, the
source blue loader's chute is installed 7.8544 mm lower than the red loader;
loader_blue preserves that authored difference instead of only recoloring it.
The record retains all match-load stock, with in_field marking the 37 pins,
36 cups, nine goals, four toggles, four loaders and 12 white panels inside the
field boundary. The CAD tile pitch is 598.058534 mm. White-panel inner faces
are 1785.178993 mm from field center and run from 50.8 to 238.76 mm above the
foam. Nominal 12-foot dimensions must not replace these actual CAD locations.

Half colors describe local mesh ends before applying the instance rotation:
pin Red-Blue has a blue upper mesh and red lower mesh; Red-Neutral and
Blue-Neutral have their alliance color above yellow. The cup's upper mesh is
opaque gray and its lower mesh is clear. Upside-down source instances retain
their rotation, so their visible world upper color may differ.

The source ZIP, STEP and binary caches are intentionally excluded from git.
The manifest records the download URL, SHA-256 checksums, source filename,
units, conversion settings, definition names and assembly placements.
Original VEX CAD/design rights remain with their respective owners. The
repository's software license does not grant a separate license to those
third-party designs; retain VEX attribution with redistributed derivatives.
