"""Live import preview: real Qt worker, Bullet torque, frame delivery and shutdown."""
from __future__ import annotations

import os
from pathlib import Path
import tempfile
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
try:
    from PySide6.QtCore import QCoreApplication
    import pybullet
    from eersim.preview import UrdfPreview
except ImportError:
    QCoreApplication = None


URDF = """<robot name="preview_fixture">
<link name="base">
  <inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
  <visual><geometry><box size=".16 .16 .04"/></geometry></visual>
  <collision><geometry><box size=".16 .16 .04"/></geometry></collision>
</link>
<link name="rotor">
  <inertial><mass value=".1"/><inertia ixx=".0002" iyy=".0002" izz=".0002" ixy="0" ixz="0" iyz="0"/></inertial>
  <visual><geometry><box size=".12 .025 .025"/></geometry></visual>
  <collision><geometry><box size=".12 .025 .025"/></geometry></collision>
</link>
<joint name="spin" type="continuous">
  <parent link="base"/><child link="rotor"/><origin xyz="0 0 .12"/><axis xyz="0 0 1"/>
  <limit effort="2.1" velocity="100"/>
</joint>
</robot>
"""


@unittest.skipIf(QCoreApplication is None, "PySide6 and PyBullet are required for live preview tests")
class PreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="eersim-preview-test-")
        self.urdf = Path(self.temp.name) / "fixture.urdf"
        self.urdf.write_text(URDF)
        self.preview = UrdfPreview()
        self.states, self.frames, self.errors = [], [], []
        self.preview.state.connect(self.states.append)
        self.preview.frame.connect(self.frames.append)
        self.preview.failed.connect(self.errors.append)

    def tearDown(self):
        self.preview.close()
        self.temp.cleanup()

    def wait_for(self, predicate, timeout=6):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            self.app.processEvents()
            if predicate():
                return
            time.sleep(.005)
        self.fail(f"Preview timed out; errors={self.errors}, last_state={self.states[-1:]}")

    def configure(self, reversed=False):
        self.preview.configure(str(self.urdf), [
            {"port": 4, "joint": "spin", "rpm": 600, "gear_ratio": 1, "reversed": reversed}])
        self.wait_for(lambda: any(state.get("ready") for state in self.states))

    def test_motor_jog_moves_joint_through_real_physics_and_emits_frames(self):
        self.configure()
        self.preview.set_motor(4, 8000)
        self.wait_for(lambda: any(state.get("ready") and state["joints"][0]["position"] > .1
                                 for state in self.states))
        self.wait_for(lambda: bool(self.frames))
        state = self.states[-1]
        self.assertTrue(state["fixed_base"])
        self.assertEqual(state["pose"][:3], [0, 0, 0])
        self.assertGreater(state["motors"]["4"]["velocity"], 0)
        self.assertEqual(self.frames[-1].shape, (480, 640, 4))
        self.assertGreater(state["camera"]["distance"], 0)
        self.assertEqual(len(state["bounds"]), 2)
        self.preview.stop_all()
        count = len(self.states)
        self.wait_for(lambda: len(self.states) > count and not self.states[-1].get("running"))
        self.assertEqual(self.states[-1]["motor_commands"]["4"]["value"], 0)
        self.assertEqual(self.urdf.read_text(), URDF)
        self.assertEqual(self.errors, [])
        self.preview.close()
        self.assertFalse(self.preview._thread.is_alive())

    def test_reversal_changes_joint_direction_and_camera_preserves_target(self):
        self.configure(reversed=True)
        target = list(self.states[-1]["camera"]["target"])
        self.preview.set_camera(70, -35, 1.1)
        self.preview.set_motor(4, 8000)
        self.wait_for(lambda: any(state.get("ready") and state["joints"][0]["position"] < -.1
                                 for state in self.states))
        state = self.states[-1]
        self.assertEqual(state["camera"]["target"], target)
        self.assertEqual(state["camera"]["yaw"], 70)
        self.assertGreater(state["motors"]["4"]["velocity"], 0)  # Physical port feedback.
        self.assertEqual(self.errors, [])

    def test_invalid_binding_is_visible_and_worker_can_reconfigure(self):
        self.preview.configure(str(self.urdf), [{"port": 4, "joint": "missing"}])
        self.wait_for(lambda: bool(self.errors))
        self.assertIn("missing", self.errors[0])
        self.configure()
        self.preview.set_motor(19, 4000)
        self.wait_for(lambda: len(self.errors) > 1)
        self.assertIn("no joint binding", self.errors[-1])
        self.assertFalse(self.states[-1]["running"])


if __name__ == "__main__":
    unittest.main()
