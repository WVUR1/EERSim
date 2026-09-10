"""Native process integration: needs a host C++17 compiler, no physics dependency."""
from __future__ import annotations

import math
from pathlib import Path
import tempfile
import unittest

from eersim.bridge import BuildError, NativeRuntime, RuntimeErrorNative, _compiler, build_workspace


SOURCE = r'''
#include "main.h"
pros::Motor motor(-1, pros::MotorGearset::blue);
pros::MotorGroup group({2,-3}, pros::MotorGearset::red);
pros::Controller controller;
pros::Imu imu(6);
pros::Rotation encoder(7);
pros::Distance distance(8);
pros::adi::Pneumatics piston('C', false, true);
int ticks=0;
pros::Task background([]{while(true){++ticks;pros::delay(5);}}, "background");
void initialize(){pros::lcd::initialize();pros::delay(10);pros::lcd::print(0,"init %u",pros::millis());}
void competition_initialize(){pros::lcd::set_text(1,"competition ready");}
void disabled(){}
void autonomous(){
  motor.move_voltage(14000);
  group.move_velocity(200);
  pros::lcd::print(2,"%.1f %.1f %.1f %d %d",motor.get_position(),motor.get_actual_velocity(),imu.get_heading(),encoder.get_position(),distance.get());
  pros::delay(20);
  motor.tare_position();
  motor.move_absolute(90,40);
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  piston.extend();
}
void opcontrol(){
  if(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)==127){
    volatile unsigned counter=0;while(true){++counter;}
  }
  while(true){
    motor.move_velocity(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))piston.toggle();
    pros::lcd::print(3,"ticks %d time %u",ticks,pros::millis());
    pros::delay(10);
  }
}
'''


class NativeBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            _compiler()
        except BuildError as exc:
            raise unittest.SkipTest(str(exc))
        cls.temp = tempfile.TemporaryDirectory(prefix="eersim-native-")
        cls.workspace = Path(cls.temp.name)
        (cls.workspace / "src").mkdir()
        (cls.workspace / "include").mkdir()
        (cls.workspace / "include" / "api.h").write_text('#error "Original ARM header must not be used"\n')
        (cls.workspace / "include" / "main.h").write_text(
            '#include "api.h"\nextern "C" {void initialize();void competition_initialize();'
            'void autonomous();void opcontrol();void disabled();}\n')
        (cls.workspace / "src" / "main.cpp").write_text(SOURCE)
        cls.executable = build_workspace(cls.workspace)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def snapshot(self, time):
        return {"time": time, "motors": {"1": {"position": math.pi / 2, "velocity": 2 * math.pi}},
                "rotation": {"7": {"position": math.pi, "velocity": 0}},
                "imu": {"heading": 350, "rotation": -10}, "distance": {"8": 731}, "otos": [0, 0, 0]}

    def initialized(self, robot):
        result = robot.step(self.snapshot(0), "initialize")
        self.assertFalse(result["initialized"])
        result = robot.step(self.snapshot(.01), "initialize")
        self.assertTrue(result["initialized"])
        self.assertEqual(result["lcd"]["1"], "competition ready")
        self.assertTrue(all(m["value"] == 0 for m in result["motors"].values()))
        return result

    def test_unchanged_source_and_quoted_header_overlay(self):
        self.assertEqual((self.workspace / "src" / "main.cpp").read_text(), SOURCE)
        self.assertIn("#error", (self.workspace / "include" / "api.h").read_text())
        self.assertEqual(build_workspace(self.workspace), self.executable)

    def test_motor_units_reversal_sensors_brakes_and_pneumatics(self):
        with NativeRuntime(self.executable) as robot:
            self.initialized(robot)
            result = robot.step(self.snapshot(.011), "autonomous")
            self.assertEqual(result["motors"]["1"]["value"], -12000)
            self.assertEqual(result["motors"]["2"]["value"], 100)
            self.assertEqual(result["motors"]["3"]["value"], -100)
            self.assertEqual(result["lcd"]["2"], "-90.0 -60.0 350.0 18000 731")
            self.assertTrue(result["adi"]["3"])  # Active-low piston starts retracted.
            self.assertNotIn("7", result["motors"])  # Rotation sensor is not an actuator.
            result = robot.step(self.snapshot(.030), "autonomous")
            self.assertFalse(result["callback_done"])
            result = robot.step(self.snapshot(.031), "autonomous")
            self.assertTrue(result["callback_done"])
            self.assertEqual(result["motors"]["1"]["mode"], "position")
            self.assertAlmostEqual(result["motors"]["1"]["value"], 0)
            self.assertEqual(result["motors"]["1"]["velocity"], 40)
            self.assertEqual(result["motors"]["1"]["brake"], "hold")
            self.assertFalse(result["adi"]["3"])
            result = robot.step(self.snapshot(.032), "disabled")
            self.assertTrue(all(m["value"] == 0 for m in result["motors"].values()))

    def test_deterministic_tasks_and_controller_edge_between_physics_ticks(self):
        traces = []
        for _ in range(2):
            with NativeRuntime(self.executable) as robot:
                self.initialized(robot)
                trace = []
                for tick in range(11, 45):
                    controls = {"analog": {"left_y": 72}, "digital": {"a": tick >= 13}}
                    trace.append(robot.step(self.snapshot(tick / 1000), "opcontrol", .001, controls))
                self.assertTrue(trace[0]["adi"]["3"])
                self.assertFalse(trace[-1]["adi"]["3"])  # Press first occurred while callback slept.
                self.assertEqual(trace[-1]["motors"]["1"]["value"], -72)
                traces.append(trace)
        self.assertEqual(traces[0], traces[1])

    def test_watchdog_surfaces_non_yielding_robot_task(self):
        with NativeRuntime(self.executable) as robot:
            self.initialized(robot)
            with self.assertRaisesRegex(RuntimeErrorNative, "Watchdog.*without yielding"):
                robot.step(self.snapshot(.011), "opcontrol", controller={"analog": {"right_x": 127}})

    def test_disable_cancels_callback_before_its_next_due_instruction(self):
        with NativeRuntime(self.executable) as robot:
            self.initialized(robot)
            robot.step(self.snapshot(.011), "autonomous")
            result = robot.step(self.snapshot(.040), "disabled")
            self.assertTrue(result["adi"]["3"])  # Sleeping autonomous must not extend it.
            self.assertTrue(all(m["value"] == 0 for m in result["motors"].values()))
            result = robot.step(self.snapshot(.041), "autonomous")
            self.assertFalse(result["callback_done"])  # Re-enabling restarts the callback.
            self.assertEqual(result["motors"]["1"]["value"], -12000)

    def test_missing_sensor_binding_is_a_visible_error(self):
        with NativeRuntime(self.executable) as robot:
            snapshot = self.snapshot(0)
            snapshot.pop("distance")
            robot.step(snapshot, "initialize")
            snapshot["time"] = .01
            robot.step(snapshot, "initialize")
            snapshot["time"] = .011
            with self.assertRaisesRegex(RuntimeErrorNative, "Distance sensor port 8 has no field binding"):
                robot.step(snapshot, "autonomous")


if __name__ == "__main__":
    unittest.main()
