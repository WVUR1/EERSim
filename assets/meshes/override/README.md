# Official VEX Override visual geometry

These derived meshes originate from VEX Robotics' [official 2026–2027 V5RC
Override field CAD](https://link.vex.com/docs/26-27/v5rc/field-cad), STEP assembly
276-9250-000 (2026-04-26).STEP, exported by SolidWorks on April 26, 2026.

The conversion preserves actual CAD surfaces, changes millimetres to metres,
and normalizes each part's axis and origin for the simulator. Visual
tessellation uses a 0.25 mm chord tolerance. Measured BRep dimensions,
analytical surface profiles, source checksums, exact definition names and
assembly placements are recorded in ../../cad_manifest.json.

See ../../../tools/README.md for the reproducible conversion workflow.
Physics collision shapes remain separate, with dimensions taken from the
official geometry. Meshes alone do not define material friction, compliance
or mass.

Original VEX CAD/design rights remain with their respective owners.
The software repository's license does not replace those rights. Preserve
this attribution and the provenance manifest with redistributed derivatives.
