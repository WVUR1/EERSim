"""CLI file-preservation checks; no PhysX/NumPy imports or simulation required."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from benchmark import guard_output_paths


class BenchmarkPathTests(unittest.TestCase):
    def test_cli_preserves_scene_at_generated_output_paths(self):
        script = Path(__file__).with_name("benchmark.py")
        for name in ("cpu.usda", "result.json", "final_poses.npy"):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temporary:
                work = Path(temporary)
                source = work / name
                original = b'#usda 1.0\n# unique source sentinel\n'
                source.write_bytes(original)
                probe = work / "unused-probe.so"
                probe.write_bytes(b"probe must never load")
                result = subprocess.run(
                    [sys.executable, str(script), "--backend", "cpu", "--scene", str(source),
                     "--work-dir", str(work), "--probe-library", str(probe)],
                    text=True, capture_output=True, timeout=10,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("Benchmark output would overwrite input", result.stderr)
                self.assertEqual(source.read_bytes(), original)
                self.assertEqual(set(work.iterdir()), {source, probe})

    def test_hardlink_output_cannot_overwrite_source_under_another_name(self):
        with tempfile.TemporaryDirectory() as temporary:
            work = Path(temporary)
            source = work / "source.usda"
            source.write_text("source sentinel", encoding="utf-8")
            output = work / "gpu.usda"
            try:
                os.link(source, output)
            except OSError as error:
                self.skipTest(f"Hardlinks unavailable: {error}")
            with self.assertRaisesRegex(ValueError, "would overwrite input"):
                guard_output_paths(work, "gpu", [source])
            self.assertEqual(source.read_text(encoding="utf-8"), "source sentinel")

    def test_separate_scene_can_share_work_directory_without_aliasing(self):
        with tempfile.TemporaryDirectory() as temporary:
            work = Path(temporary)
            source = work / "field.usda"
            source.write_text("source sentinel", encoding="utf-8")
            guard_output_paths(work, "gpu", [source])
            self.assertEqual(list(work.iterdir()), [source])


if __name__ == "__main__":
    unittest.main()
