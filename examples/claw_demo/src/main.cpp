// Ordinary PROS source for the six-motor X-drive + lift + claw preset.
// The simulator supplies the existing EERKit OTOS serial protocol on port 7.
#include "main.h"
#include <algorithm>
#include <cmath>
#include <cstring>

pros::Motor fl(1, pros::MotorGearset::blue);
pros::Motor fr(2, pros::MotorGearset::blue);
pros::Motor bl(3, pros::MotorGearset::blue);
pros::Motor br(4, pros::MotorGearset::blue);
pros::Motor lift(5, pros::MotorGearset::red);
pros::Motor claw(6, pros::MotorGearset::red);
pros::Serial odometry(7, 115200);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

namespace {
constexpr double pi = 3.14159265358979323846;
double x = -1.45, y = -.90, yaw = 0;

void read_pose() {
    // The real club interface has the same ! + 20-byte float packet.
    if (odometry.get_read_avail() < 21) return;
    if (odometry.read_byte() != '!') return;
    float packet[5]{};
    if (odometry.read(reinterpret_cast<unsigned char*>(packet), sizeof(packet)) == sizeof(packet) && packet[0] == 1) {
        x = packet[1] * .0254;
        y = -packet[2] * .0254;
        yaw = packet[3] * pi / 180;
    }
}

void drive(double forward, double right, double clockwise) {
    // Inputs are motor RPM; geometry uses 2:1 external reductions and 2.75 in wheels.
    double demand[] = {forward + right + clockwise, forward - right - clockwise,
                       forward - right + clockwise, forward + right - clockwise};
    double peak = 200;
    for (double value : demand) peak = std::max(peak, std::abs(value));
    fl.move_velocity(static_cast<int>(demand[0] * 200 / peak));
    fr.move_velocity(static_cast<int>(demand[1] * 200 / peak));
    bl.move_velocity(static_cast<int>(demand[2] * 200 / peak));
    br.move_velocity(static_cast<int>(demand[3] * 200 / peak));
}

bool move_to(double target_x, double target_y, unsigned timeout_ms = 12000) {
    auto start = pros::millis();
    unsigned settled_since = 0;
    while (pros::millis() - start < timeout_ms) {
        read_pose();
        const double ex = target_x - x, ey = target_y - y;
        const double heading_error = std::atan2(std::sin(-yaw), std::cos(-yaw));
        const double vx = std::clamp(ex * 2.5, -.20, .20);
        const double vy = std::clamp(ey * 2.5, -.20, .20);
        const double rpm_per_ms = std::sqrt(.5) * 2 * 30 / (pi * .034925);
        const double forward = (vx * std::cos(yaw) + vy * std::sin(yaw)) * rpm_per_ms;
        const double right = (vx * std::sin(yaw) - vy * std::cos(yaw)) * rpm_per_ms;
        drive(forward, right, -heading_error * 95);
        if (std::hypot(ex, ey) < .004 && std::abs(heading_error) < .025) {
            if (!settled_since) settled_since = pros::millis();
            if (pros::millis() - settled_since > 200) {
                drive(0, 0, 0);
                return true;
            }
        } else settled_since = 0;
        pros::delay(10);
    }
    drive(0, 0, 0);
    pros::lcd::set_text(0, "Route timeout: check pose and obstacles");
    return false;
}

void set_lift(double height_m) { lift.move_absolute(height_m / .04 * 360, 100); }
void open_claw() { claw.move_absolute(720, 100); }
void close_claw() { claw.move_absolute(-144, 100); }
}

void initialize() {
    pros::lcd::initialize();
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    fl.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    fr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    bl.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    br.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::lcd::set_text(0, "Claw demo: R1 close/R2 open; L1 lift/L2 lower");
    pros::delay(100);
    read_pose();
}

void competition_initialize() {}
void disabled() {}

void autonomous() {
    pros::lcd::set_text(0, "1/4: Grip the red/yellow preload");
    close_claw();
    pros::delay(2500);
    pros::lcd::set_text(0, "2/4: Lift and navigate to red goal");
    set_lift(.20);
    pros::delay(4000);
    // Red goal centre comes from the official field CAD; the claw is 0.34 m ahead.
    if (!move_to(-1.536117, -.598059)) return;
    pros::lcd::set_text(0, "3/4: Lower above the goal socket");
    set_lift(.10);
    pros::delay(2000);
    pros::lcd::set_text(0, "4/4: Release the pin into the goal");
    open_claw();
    pros::delay(2000);
    move_to(-1.536117, -.85, 6000);
    pros::lcd::set_text(0, "Placement complete: inspect the placed-pin count");
}

void opcontrol() {
    while (true) {
        drive(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 1.5,
              controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * 1.5,
              controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) close_claw();
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) open_claw();
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) set_lift(.30);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) set_lift(0);
        read_pose();
        pros::delay(10);
    }
}
