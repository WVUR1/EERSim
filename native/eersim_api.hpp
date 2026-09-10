#pragma once
// Host-side PROS subset. This is an original simulation adapter, not VEX firmware.
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TIMEOUT_MAX UINT32_MAX
#define TASK_PRIORITY_DEFAULT 8
#define TASK_STACK_DEPTH_DEFAULT 8192
#define PROS_ERR INT32_MAX
#define PROS_ERR_F INFINITY

namespace pros {
enum motor_gearset_e_t { E_MOTOR_GEARSET_36 = 0, E_MOTOR_GEARSET_18 = 1, E_MOTOR_GEARSET_06 = 2, E_MOTOR_GEARSET_INVALID = 3 };
enum class MotorGearset { red = 0, green = 1, blue = 2, invalid = 3 };
enum motor_brake_mode_e_t { E_MOTOR_BRAKE_COAST = 0, E_MOTOR_BRAKE_BRAKE = 1, E_MOTOR_BRAKE_HOLD = 2, E_MOTOR_BRAKE_INVALID = 3 };
enum class MotorBrake { coast = 0, brake = 1, hold = 2, invalid = 3 };
enum motor_encoder_units_e_t { E_MOTOR_ENCODER_DEGREES = 0, E_MOTOR_ENCODER_ROTATIONS = 1, E_MOTOR_ENCODER_COUNTS = 2, E_MOTOR_ENCODER_INVALID = 3 };
enum class MotorUnits { degrees = 0, rotations = 1, counts = 2, invalid = 3 };
enum controller_id_e_t { E_CONTROLLER_MASTER = 0, E_CONTROLLER_PARTNER = 1 };
enum controller_analog_e_t { E_CONTROLLER_ANALOG_LEFT_X, E_CONTROLLER_ANALOG_LEFT_Y, E_CONTROLLER_ANALOG_RIGHT_X, E_CONTROLLER_ANALOG_RIGHT_Y };
enum controller_digital_e_t { E_CONTROLLER_DIGITAL_L1 = 6, E_CONTROLLER_DIGITAL_L2, E_CONTROLLER_DIGITAL_R1, E_CONTROLLER_DIGITAL_R2, E_CONTROLLER_DIGITAL_UP, E_CONTROLLER_DIGITAL_DOWN, E_CONTROLLER_DIGITAL_LEFT, E_CONTROLLER_DIGITAL_RIGHT, E_CONTROLLER_DIGITAL_X, E_CONTROLLER_DIGITAL_B, E_CONTROLLER_DIGITAL_Y, E_CONTROLLER_DIGITAL_A };

std::uint32_t millis();
std::uint64_t micros();
void delay(std::uint32_t milliseconds);
void delay_until(std::uint32_t* previous, std::uint32_t delta);
using task_fn_t = void (*)(void*);
class Task {
 public:
  explicit Task(std::function<void()> fn, const char* name = "Task");
  Task(std::function<void()> fn, std::uint32_t priority, std::uint16_t stack = TASK_STACK_DEPTH_DEFAULT, const char* name = "Task");
  Task(task_fn_t fn, void* param = nullptr, std::uint32_t priority = TASK_PRIORITY_DEFAULT, std::uint16_t stack = TASK_STACK_DEPTH_DEFAULT, const char* name = "Task");
  void remove();
  void suspend();
  void resume();
  void join();
  static void delay(std::uint32_t ms) { pros::delay(ms); }
  static void delay_until(std::uint32_t* previous, std::uint32_t delta) { pros::delay_until(previous, delta); }
 private:
  int id_;
};
class Mutex {
 public:
  Mutex();
  bool take(std::uint32_t timeout = TIMEOUT_MAX);
  bool give();
  void lock() { take(); }
  void unlock() { give(); }
 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

class Motor {
 public:
  explicit Motor(std::int8_t port, MotorGearset gearset = MotorGearset::green, MotorUnits units = MotorUnits::degrees);
  Motor(std::int8_t port, motor_gearset_e_t gearset, bool reverse = false, motor_encoder_units_e_t units = E_MOTOR_ENCODER_DEGREES);
  Motor(std::int8_t port, bool reverse, motor_encoder_units_e_t units = E_MOTOR_ENCODER_DEGREES);
  std::int32_t move(std::int32_t power) const;
  std::int32_t move_voltage(std::int32_t voltage) const;
  std::int32_t move_velocity(std::int32_t velocity) const;
  std::int32_t move_absolute(double position, std::int32_t velocity) const;
  std::int32_t move_relative(double position, std::int32_t velocity) const;
  std::int32_t modify_profiled_velocity(std::int32_t velocity) const;
  std::int32_t brake() const;
  double get_position() const;
  double get_actual_velocity() const;
  std::int32_t get_voltage() const;
  std::int32_t get_target_velocity() const;
  double get_target_position() const;
  std::int32_t tare_position() const;
  std::int32_t set_zero_position(double position) const;
  std::int32_t set_brake_mode(motor_brake_mode_e_t mode) const;
  std::int32_t set_brake_mode(MotorBrake mode) const { return set_brake_mode(static_cast<motor_brake_mode_e_t>(mode)); }
  motor_brake_mode_e_t get_brake_mode() const;
  std::int32_t set_gearing(motor_gearset_e_t gearset) const;
  std::int32_t set_gearing(MotorGearset gearset) const { return set_gearing(static_cast<motor_gearset_e_t>(gearset)); }
  motor_gearset_e_t get_gearing() const;
  std::int32_t set_encoder_units(motor_encoder_units_e_t units) const;
  std::int32_t set_encoder_units(MotorUnits units) const { return set_encoder_units(static_cast<motor_encoder_units_e_t>(units)); }
  std::int32_t set_reversed(bool reverse);
  bool is_reversed() const { return reverse_; }
  std::int8_t get_port() const { return static_cast<std::int8_t>(reverse_ ? -port_ : port_); }
  const Motor& operator=(std::int32_t value) const { move(value); return *this; }
 private:
  int port_;
  bool reverse_;
  double direction() const { return reverse_ ? -1.0 : 1.0; }
};
class MotorGroup {
 public:
  explicit MotorGroup(std::initializer_list<std::int8_t> ports, MotorGearset gearset = MotorGearset::green, MotorUnits units = MotorUnits::degrees);
  MotorGroup(std::initializer_list<std::int8_t> ports, motor_gearset_e_t gearset, motor_encoder_units_e_t units = E_MOTOR_ENCODER_DEGREES);
  explicit MotorGroup(const std::vector<std::int8_t>& ports, MotorGearset gearset = MotorGearset::green, MotorUnits units = MotorUnits::degrees);
  std::int32_t move(std::int32_t value) const;
  std::int32_t move_voltage(std::int32_t value) const;
  std::int32_t move_velocity(std::int32_t value) const;
  std::int32_t move_absolute(double position, std::int32_t velocity) const;
  std::int32_t move_relative(double position, std::int32_t velocity) const;
  std::int32_t brake() const;
  std::int32_t tare_position() const;
  std::int32_t set_brake_mode(motor_brake_mode_e_t mode) const;
  std::int32_t set_brake_mode(MotorBrake mode) const { return set_brake_mode(static_cast<motor_brake_mode_e_t>(mode)); }
  std::int32_t set_gearing(motor_gearset_e_t gearset) const;
  std::int32_t set_gearing(MotorGearset gearset) const { return set_gearing(static_cast<motor_gearset_e_t>(gearset)); }
  std::int32_t set_encoder_units(motor_encoder_units_e_t units) const;
  std::int32_t set_encoder_units(MotorUnits units) const { return set_encoder_units(static_cast<motor_encoder_units_e_t>(units)); }
  double get_position(std::uint8_t index = 0) const;
  double get_actual_velocity(std::uint8_t index = 0) const;
  std::vector<double> get_position_all() const;
  std::vector<double> get_actual_velocity_all() const;
  std::vector<std::int8_t> get_port_all() const;
  std::size_t size() const { return motors_.size(); }
  const MotorGroup& operator=(std::int32_t value) const { move(value); return *this; }
 private:
  std::vector<Motor> motors_;
};

class Controller {
 public:
  explicit Controller(controller_id_e_t id = E_CONTROLLER_MASTER) : id_(id) {}
  std::int32_t get_analog(controller_analog_e_t channel) const;
  bool get_digital(controller_digital_e_t button) const;
  bool get_digital_new_press(controller_digital_e_t button) const;
  bool is_connected() const;
  std::int32_t set_text(std::uint8_t line, std::uint8_t col, const std::string& text) const;
  std::int32_t print(std::uint8_t line, std::uint8_t col, const char* fmt, ...) const;
  std::int32_t clear() const;
  std::int32_t clear_line(std::uint8_t line) const;
  std::int32_t rumble(const char* pattern) const;
 private:
  controller_id_e_t id_;
};
class Imu {
 public:
  explicit Imu(std::uint8_t port);
  std::int32_t reset(bool blocking = false) const;
  bool is_calibrating() const;
  double get_heading() const;
  double get_rotation() const;
  std::int32_t set_heading(double heading) const;
  std::int32_t set_rotation(double rotation) const;
  std::int32_t tare_heading() const { return set_heading(0); }
  std::int32_t tare_rotation() const { return set_rotation(0); }
  std::int32_t tare() const { tare_heading(); return tare_rotation(); }
 private:
  int port_;
};
using IMU = Imu;
class Rotation {
 public:
  explicit Rotation(std::uint8_t port, bool reverse = false);
  std::int32_t get_position() const;
  std::int32_t get_velocity() const;
  std::uint32_t get_angle() const;
  std::int32_t reset_position() const { return set_position(0); }
  std::int32_t set_position(std::int32_t centidegrees) const;
  std::int32_t set_reversed(bool reverse);
 private:
  int port_; bool reverse_;
};
class Distance {
 public:
  explicit Distance(std::uint8_t port);
  std::int32_t get() const;
 private: int port_;
};
class Serial {
 public:
  explicit Serial(std::uint8_t port, std::int32_t baudrate = 115200);
  std::int32_t read_byte() const;
  std::int32_t read(std::uint8_t* buffer, std::int32_t length) const;
  std::int32_t get_read_avail() const;
  std::int32_t write_byte(std::uint8_t value) const;
  std::int32_t write(const std::uint8_t* buffer, std::int32_t length) const;
  std::int32_t flush() const;
  std::int32_t set_baudrate(std::int32_t baudrate) const;
 private: int port_;
};
namespace adi {
class DigitalOut {
 public:
  explicit DigitalOut(std::uint8_t port, bool value = false);
  std::int32_t set_value(bool value) const;
  std::int32_t get_value() const;
 protected: int port_;
};
class Pneumatics : public DigitalOut {
 public:
  Pneumatics(std::uint8_t port, bool start_extended = false, bool extended_is_low = false);
  std::int32_t extend() const;
  std::int32_t retract() const;
  std::int32_t toggle() const;
  bool is_extended() const;
 private: bool extended_is_high_;
};
}
using ADIDigitalOut = adi::DigitalOut;
namespace lcd {
bool initialize();
bool is_initialized();
bool set_text(std::int16_t line, const std::string& text);
bool print(std::int16_t line, const char* fmt, ...);
bool clear();
bool clear_line(std::int16_t line);
}
namespace competition {
bool is_disabled();
bool is_autonomous();
bool is_connected();
std::uint8_t get_status();
}
namespace c {
inline std::uint32_t millis() { return pros::millis(); }
inline void delay(std::uint32_t ms) { pros::delay(ms); }
inline void task_delay(std::uint32_t ms) { pros::delay(ms); }
inline void task_delay_until(std::uint32_t* previous, std::uint32_t delta) { pros::delay_until(previous, delta); }
std::int32_t motor_move(std::int8_t port, std::int32_t value);
std::int32_t motor_move_voltage(std::int8_t port, std::int32_t value);
std::int32_t motor_move_velocity(std::int8_t port, std::int32_t value);
double motor_get_position(std::int8_t port);
double motor_get_actual_velocity(std::int8_t port);
}
namespace literals {
inline Motor operator""_mtr(unsigned long long port) { return Motor(static_cast<std::int8_t>(port)); }
inline Motor operator""_rmtr(unsigned long long port) { return Motor(-static_cast<std::int8_t>(port)); }
}
}
#ifdef PROS_USE_SIMPLE_NAMES
#define CONTROLLER_MASTER pros::E_CONTROLLER_MASTER
#define CONTROLLER_PARTNER pros::E_CONTROLLER_PARTNER
#define MOTOR_GEARSET_06 pros::E_MOTOR_GEARSET_06
#define MOTOR_GEARSET_18 pros::E_MOTOR_GEARSET_18
#define MOTOR_GEARSET_36 pros::E_MOTOR_GEARSET_36
#endif
