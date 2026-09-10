#include "eersim_api.hpp"
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

extern "C" {
void initialize(); void competition_initialize(); void autonomous(); void opcontrol(); void disabled();
}

namespace sim {
constexpr double pi = 3.14159265358979323846;
struct Cancelled {};
struct Worker {
  int id; std::string name; std::function<void()> fn;
  bool done = false, cancelled = false, suspended = false, running = false;
  double wake = 0;
};
struct MotorState { double position=0, velocity=0, zero=0, value=0, target_velocity=0; int mode=0, brake=0, gear=1, units=0; bool configured=false; };
struct RotationState {double position=0, velocity=0, zero=0; bool bound=false;};
struct ImuState { double heading_offset=0, rotation_offset=0, calibrated_at=0; };
struct SerialState { std::deque<std::uint8_t> rx; std::vector<std::uint8_t> tx; std::array<double,3> offset{}; double last_packet=-1; };
struct State {
  std::mutex mutex; std::condition_variable cv;
  std::vector<std::shared_ptr<Worker>> workers;
  std::map<int,MotorState> motors; std::map<int,ImuState> imus;
  std::map<int,RotationState> rotations;
  std::map<int,double> distance;
  std::map<int,SerialState> serial;
  std::map<int,bool> adi;
  std::map<int,std::string> lcd;
  std::vector<std::string> diagnostics;
  std::array<int,4> analog{}; std::uint32_t digital=0, previous_digital=0, consumed_digital=0;
  std::array<double,3> otos{};
  double time=0, imu_heading=0, imu_rotation=0;
  int allowed=-1, init=-1, compinit=-1, callback=-1;
  std::string mode="initialize", callback_mode;
  std::string error; bool lcd_initialized=false;
};
State& state() { static auto* s = new State; return *s; }
thread_local int self = -1;
[[noreturn]] void unsupported(const std::string& text) { throw std::runtime_error("Unsupported PROS simulation operation: " + text); }
void port_check(int port) { if (port < 1 || port > 21) throw std::invalid_argument("Smart port must be in 1..21"); }
std::shared_ptr<Worker> worker(int id) { return state().workers.at(static_cast<std::size_t>(id)); }
int spawn(std::function<void()> fn, const std::string& name) {
  auto& s=state(); std::lock_guard<std::mutex> lock(s.mutex);
  auto w=std::make_shared<Worker>(); w->id=static_cast<int>(s.workers.size()); w->name=name; w->fn=std::move(fn); w->wake=s.time;
  s.workers.push_back(w);
  std::thread([w]() {
    auto& s=state(); self=w->id;
    std::unique_lock<std::mutex> lock(s.mutex);
    s.cv.wait(lock,[&]{return s.allowed==self || w->cancelled;});
    if (w->cancelled) {w->done=true;s.cv.notify_all();return;}
    lock.unlock();
    try { w->fn(); }
    catch (const Cancelled&) {}
    catch (const std::exception& e) {std::lock_guard<std::mutex> guard(s.mutex);s.error=w->name+": "+e.what();}
    catch (...) {std::lock_guard<std::mutex> guard(s.mutex);s.error=w->name+": unknown native exception";}
    lock.lock(); w->done=true; w->running=false; s.allowed=-1; s.cv.notify_all();
  }).detach();
  return w->id;
}
void yield(double wake) {
  auto& s=state();
  if (self<0) unsupported("pros::delay during global construction; create a Task instead");
  std::unique_lock<std::mutex> lock(s.mutex); auto w=worker(self);
  w->wake=wake; w->running=false; s.allowed=-1; s.cv.notify_all();
  s.cv.wait(lock,[&]{return s.allowed==self || w->cancelled;});
  if (w->cancelled) throw Cancelled{};
}
void drain() {
  auto& s=state(); std::unique_lock<std::mutex> lock(s.mutex);
  for (;;) {
    std::shared_ptr<Worker> next;
    for (auto& w:s.workers) if (!w->done && !w->cancelled && !w->suspended && w->wake<=s.time+1e-7) {next=w;break;}
    if (!next || !s.error.empty()) break;
    s.allowed=next->id; next->running=true; s.cv.notify_all();
    if (!s.cv.wait_for(lock,std::chrono::seconds(2),[&]{return !next->running || next->done;})) {
      s.error="Watchdog: task '"+next->name+"' ran for 2 wall seconds without yielding. Add pros::delay to long loops. Simulation time was not advanced."; break;
    }
  }
}
void cancel(int id) {
  if(id<0)return; auto& s=state(); std::unique_lock<std::mutex> lock(s.mutex); auto w=worker(id);
  w->cancelled=true;s.cv.notify_all();
  if(self==id) throw Cancelled{};
  s.cv.wait_for(lock,std::chrono::milliseconds(100),[&]{return w->done;});
}
bool done(int id) { return id>=0 && worker(id)->done; }
void zero_motors() {for(auto& entry:state().motors){entry.second.mode=0;entry.second.value=0;entry.second.target_velocity=0;}}
double wrap(double a) {a=std::fmod(a,360.0);return a<0?a+360:a;}
double units_per_degree(const MotorState& m) {
  if(m.units==1)return 1.0/360.0;
  if(m.units==2)return (m.gear==0?1800.0:m.gear==1?900.0:300.0)/360.0;
  return 1.0;
}
double max_rpm(const MotorState& m) {return m.gear==0?100:m.gear==1?200:600;}
std::string format(const char* fmt, va_list args) {char b[2048];std::vsnprintf(b,sizeof(b),fmt,args);return b;}
void fill_serial(int port) {
  auto& s=state();auto& serial=s.serial[port];
  if(!serial.rx.empty() || serial.last_packet==s.time)return;
  // EERKit SerialMessage: float type plus a 16-byte union, framed by '!'.
  float packet[5]={1,static_cast<float>(s.otos[0]+serial.offset[0]),static_cast<float>(s.otos[1]+serial.offset[1]),static_cast<float>(s.otos[2]+serial.offset[2]),0};
  serial.rx.push_back('!');auto* bytes=reinterpret_cast<const std::uint8_t*>(packet);
  serial.rx.insert(serial.rx.end(),bytes,bytes+sizeof(packet));serial.last_packet=s.time;
}
void consume_serial(int port) {
  auto& s=state();auto& serial=s.serial[port];auto& tx=serial.tx;
  if(tx.size()<21)return;
  if(tx[0]!='!')unsupported("Serial protocol: only EERKit framed 20-byte OTOS messages are modeled");
  float packet[5];std::memcpy(packet,tx.data()+1,20);
  if(packet[0]==4)for(int i=0;i<3;++i)serial.offset[i]=packet[i+1]-s.otos[i];
  else if(packet[0]==3)for(int i=0;i<3;++i)serial.offset[i]=packet[i+1];
  else unsupported("OTOS serial transmit type "+std::to_string(packet[0]));
  tx.clear();serial.rx.clear();serial.last_packet=-1;
}
}

namespace pros {
std::uint32_t millis(){return static_cast<std::uint32_t>(std::floor(sim::state().time+1e-7));}
std::uint64_t micros(){return static_cast<std::uint64_t>(std::floor(sim::state().time*1000+1e-4));}
void delay(std::uint32_t ms){sim::yield(sim::state().time+std::max<std::uint32_t>(ms,1));}
void delay_until(std::uint32_t* previous,std::uint32_t delta){auto target=*previous+delta;*previous=target;auto now=millis();delay(static_cast<std::int32_t>(target-now)>0?target-now:1);}
Task::Task(std::function<void()> fn,const char* name):id_(sim::spawn(std::move(fn),name)){}
Task::Task(std::function<void()> fn,std::uint32_t priority,std::uint16_t,const char* name):Task(std::move(fn),name){if(priority!=TASK_PRIORITY_DEFAULT)sim::state().diagnostics.push_back("Task priorities are cooperative; default creation order is used.");}
Task::Task(task_fn_t fn,void* param,std::uint32_t priority,std::uint16_t stack,const char* name):Task([fn,param]{fn(param);},priority,stack,name){}
void Task::remove(){sim::cancel(id_);}
void Task::suspend(){sim::worker(id_)->suspended=true;if(sim::self==id_)delay(1);}
void Task::resume(){sim::worker(id_)->suspended=false;}
void Task::join(){while(!sim::done(id_))delay(1);}
struct Mutex::Impl {int owner=-2;};
Mutex::Mutex():impl_(std::make_shared<Impl>()){}
bool Mutex::take(std::uint32_t timeout){auto start=millis();while(impl_->owner!=-2){if(timeout!=TIMEOUT_MAX && millis()-start>=timeout)return false;delay(1);}impl_->owner=sim::self;return true;}
bool Mutex::give(){if(impl_->owner!=sim::self)return false;impl_->owner=-2;return true;}

Motor::Motor(std::int8_t port,MotorGearset gearset,MotorUnits units):port_(std::abs(static_cast<int>(port))),reverse_(port<0){sim::port_check(port_);auto& m=sim::state().motors[port_];m.configured=true;m.gear=static_cast<int>(gearset);m.units=static_cast<int>(units);if(m.gear>2||m.units>2)throw std::invalid_argument("Invalid motor gearset or units");}
Motor::Motor(std::int8_t port,motor_gearset_e_t gearset,bool reverse,motor_encoder_units_e_t units):Motor(port,static_cast<MotorGearset>(gearset),static_cast<MotorUnits>(units)){reverse_=reverse_^(reverse);}
Motor::Motor(std::int8_t port,bool reverse,motor_encoder_units_e_t units):Motor(port,E_MOTOR_GEARSET_18,reverse,units){}
std::int32_t Motor::move(std::int32_t value)const{return move_voltage(static_cast<int>(std::clamp(value,-127,127)*12000.0/127.0));}
std::int32_t Motor::move_voltage(std::int32_t value)const{auto& m=sim::state().motors[port_];m.mode=0;m.value=direction()*std::clamp(value,-12000,12000);return 1;}
std::int32_t Motor::move_velocity(std::int32_t value)const{auto& m=sim::state().motors[port_];m.mode=1;m.value=direction()*std::clamp(static_cast<double>(value),-sim::max_rpm(m),sim::max_rpm(m));return 1;}
std::int32_t Motor::move_absolute(double position,std::int32_t velocity)const{auto& m=sim::state().motors[port_];m.mode=2;m.value=direction()*position/sim::units_per_degree(m)+m.zero;m.target_velocity=std::min(std::abs(static_cast<double>(velocity)),sim::max_rpm(m));return 1;}
std::int32_t Motor::move_relative(double position,std::int32_t velocity)const{return move_absolute(get_position()+position,velocity);}
std::int32_t Motor::modify_profiled_velocity(std::int32_t velocity)const{auto& m=sim::state().motors[port_];m.target_velocity=std::min(std::abs(static_cast<double>(velocity)),sim::max_rpm(m));return 1;}
std::int32_t Motor::brake()const{return move_velocity(0);}
double Motor::get_position()const{auto& m=sim::state().motors[port_];return direction()*(m.position*180/sim::pi-m.zero)*sim::units_per_degree(m);}
double Motor::get_actual_velocity()const{return direction()*sim::state().motors[port_].velocity*60/(2*sim::pi);}
std::int32_t Motor::get_voltage()const{auto& m=sim::state().motors[port_];if(m.mode!=0)sim::unsupported("Motor::get_voltage in closed loop mode; motor electrical telemetry is unavailable");return static_cast<int>(direction()*m.value);}
std::int32_t Motor::get_target_velocity()const{auto& m=sim::state().motors[port_];return static_cast<int>(direction()*(m.mode==1?m.value:m.target_velocity));}
double Motor::get_target_position()const{auto& m=sim::state().motors[port_];return direction()*(m.value-m.zero)*sim::units_per_degree(m);}
std::int32_t Motor::tare_position()const{return set_zero_position(0);}
std::int32_t Motor::set_zero_position(double position)const{auto& m=sim::state().motors[port_];m.zero=m.position*180/sim::pi-direction()*position/sim::units_per_degree(m);return 1;}
std::int32_t Motor::set_brake_mode(motor_brake_mode_e_t mode)const{if(mode>E_MOTOR_BRAKE_HOLD)throw std::invalid_argument("Invalid brake mode");sim::state().motors[port_].brake=mode;return 1;}
motor_brake_mode_e_t Motor::get_brake_mode()const{return static_cast<motor_brake_mode_e_t>(sim::state().motors[port_].brake);}
std::int32_t Motor::set_gearing(motor_gearset_e_t gearset)const{if(gearset>E_MOTOR_GEARSET_06)throw std::invalid_argument("Invalid gearset");sim::state().motors[port_].gear=gearset;return 1;}
motor_gearset_e_t Motor::get_gearing()const{return static_cast<motor_gearset_e_t>(sim::state().motors[port_].gear);}
std::int32_t Motor::set_encoder_units(motor_encoder_units_e_t units)const{if(units>E_MOTOR_ENCODER_COUNTS)throw std::invalid_argument("Invalid encoder units");sim::state().motors[port_].units=units;return 1;}
std::int32_t Motor::set_reversed(bool reverse){reverse_=reverse;return 1;}
MotorGroup::MotorGroup(std::initializer_list<std::int8_t> ports,MotorGearset gear,MotorUnits units):MotorGroup(std::vector<std::int8_t>(ports),gear,units){}
MotorGroup::MotorGroup(std::initializer_list<std::int8_t> ports,motor_gearset_e_t gear,motor_encoder_units_e_t units):MotorGroup(ports,static_cast<MotorGearset>(gear),static_cast<MotorUnits>(units)){}
MotorGroup::MotorGroup(const std::vector<std::int8_t>& ports,MotorGearset gear,MotorUnits units){for(auto p:ports)motors_.emplace_back(p,gear,units);if(motors_.empty())throw std::invalid_argument("MotorGroup cannot be empty");}
#define GROUP_ONE(name,type) std::int32_t MotorGroup::name(type v)const{for(auto& m:motors_)m.name(v);return 1;}
GROUP_ONE(move,std::int32_t) GROUP_ONE(move_voltage,std::int32_t) GROUP_ONE(move_velocity,std::int32_t)
GROUP_ONE(set_brake_mode,motor_brake_mode_e_t) GROUP_ONE(set_gearing,motor_gearset_e_t) GROUP_ONE(set_encoder_units,motor_encoder_units_e_t)
#undef GROUP_ONE
std::int32_t MotorGroup::move_absolute(double position,std::int32_t velocity)const{for(auto& m:motors_)m.move_absolute(position,velocity);return 1;}
std::int32_t MotorGroup::move_relative(double position,std::int32_t velocity)const{for(auto& m:motors_)m.move_relative(position,velocity);return 1;}
std::int32_t MotorGroup::brake()const{for(auto& m:motors_)m.brake();return 1;}
std::int32_t MotorGroup::tare_position()const{for(auto& m:motors_)m.tare_position();return 1;}
double MotorGroup::get_position(std::uint8_t i)const{return motors_.at(i).get_position();}
double MotorGroup::get_actual_velocity(std::uint8_t i)const{return motors_.at(i).get_actual_velocity();}
std::vector<double> MotorGroup::get_position_all()const{std::vector<double> v;for(auto& m:motors_)v.push_back(m.get_position());return v;}
std::vector<double> MotorGroup::get_actual_velocity_all()const{std::vector<double> v;for(auto& m:motors_)v.push_back(m.get_actual_velocity());return v;}
std::vector<std::int8_t> MotorGroup::get_port_all()const{std::vector<std::int8_t> v;for(auto& m:motors_)v.push_back(m.get_port());return v;}

std::int32_t Controller::get_analog(controller_analog_e_t ch)const{if(id_!=E_CONTROLLER_MASTER)sim::unsupported("partner controller input");return sim::state().analog.at(ch);}
bool Controller::get_digital(controller_digital_e_t button)const{if(id_!=E_CONTROLLER_MASTER)sim::unsupported("partner controller input");return (sim::state().digital & (1u<<button))!=0;}
bool Controller::get_digital_new_press(controller_digital_e_t button)const{auto& s=sim::state();auto bit=1u<<button;bool down=get_digital(button);bool press=down&&!(s.previous_digital&bit);if(down)s.previous_digital|=bit;else s.previous_digital&=~bit;return press;}
bool Controller::is_connected()const{return id_==E_CONTROLLER_MASTER;}
std::int32_t Controller::set_text(std::uint8_t line,std::uint8_t col,const std::string& text)const{auto& output=sim::state().lcd[8+line+id_*3];if(output.size()<col)output.resize(col,' ');output.replace(col,text.size(),text);return 1;}
std::int32_t Controller::print(std::uint8_t line,std::uint8_t col,const char* fmt,...)const{va_list args;va_start(args,fmt);auto txt=sim::format(fmt,args);va_end(args);return set_text(line,col,txt);}
std::int32_t Controller::clear()const{for(int i=0;i<3;++i)clear_line(i);return 1;}
std::int32_t Controller::clear_line(std::uint8_t line)const{sim::state().lcd[8+line+id_*3].clear();return 1;}
std::int32_t Controller::rumble(const char* pattern)const{sim::state().diagnostics.push_back(std::string("Controller rumble: ")+pattern);return 1;}
Imu::Imu(std::uint8_t port):port_(port){sim::port_check(port);sim::state().imus.emplace(port,sim::ImuState{});}
std::int32_t Imu::reset(bool blocking)const{auto& s=sim::state();auto& i=s.imus[port_];i.calibrated_at=s.time+2000;i.heading_offset=-s.imu_heading;i.rotation_offset=-s.imu_rotation;if(blocking)delay(2000);return 1;}
bool Imu::is_calibrating()const{return sim::state().time<sim::state().imus[port_].calibrated_at;}
double Imu::get_heading()const{return sim::wrap(sim::state().imu_heading+sim::state().imus[port_].heading_offset);}
double Imu::get_rotation()const{return sim::state().imu_rotation+sim::state().imus[port_].rotation_offset;}
std::int32_t Imu::set_heading(double heading)const{sim::state().imus[port_].heading_offset=heading-sim::state().imu_heading;return 1;}
std::int32_t Imu::set_rotation(double rotation)const{sim::state().imus[port_].rotation_offset=rotation-sim::state().imu_rotation;return 1;}
Rotation::Rotation(std::uint8_t port,bool reverse):port_(port),reverse_(reverse){sim::port_check(port);}
std::int32_t Rotation::get_position()const{auto& r=sim::state().rotations[port_];if(!r.bound)sim::unsupported("Rotation sensor port "+std::to_string(port_)+" has no snapshot binding");return static_cast<int>((reverse_?-1:1)*(r.position*18000/sim::pi-r.zero));}
std::int32_t Rotation::get_velocity()const{auto& r=sim::state().rotations[port_];if(!r.bound)sim::unsupported("Rotation sensor snapshot binding missing");return static_cast<int>((reverse_?-1:1)*r.velocity*18000/sim::pi);}
std::uint32_t Rotation::get_angle()const{return static_cast<unsigned>(sim::wrap(get_position()/100.0)*100);}
std::int32_t Rotation::set_position(std::int32_t position)const{auto& r=sim::state().rotations[port_];if(!r.bound)sim::unsupported("Rotation sensor snapshot binding missing");r.zero=r.position*18000/sim::pi-(reverse_?-1:1)*position;return 1;}
std::int32_t Rotation::set_reversed(bool reverse){reverse_=reverse;return 1;}
Distance::Distance(std::uint8_t port):port_(port){sim::port_check(port);}
std::int32_t Distance::get()const{auto& d=sim::state().distance;auto it=d.find(port_);if(it==d.end())sim::unsupported("Distance sensor port "+std::to_string(port_)+" has no field binding");return static_cast<int>(it->second);}
Serial::Serial(std::uint8_t port,std::int32_t):port_(port){sim::port_check(port);sim::state().serial.emplace(port,sim::SerialState{});}
std::int32_t Serial::read_byte()const{sim::fill_serial(port_);auto& rx=sim::state().serial[port_].rx;if(rx.empty()){delay(1);return -1;}int byte=rx.front();rx.pop_front();return byte;}
std::int32_t Serial::read(std::uint8_t* buffer,std::int32_t length)const{sim::fill_serial(port_);auto& rx=sim::state().serial[port_].rx;int n=std::min(length,static_cast<int>(rx.size()));for(int i=0;i<n;++i){buffer[i]=rx.front();rx.pop_front();}return n;}
std::int32_t Serial::get_read_avail()const{sim::fill_serial(port_);auto& rx=sim::state().serial[port_].rx;if(rx.empty())delay(1);return static_cast<int>(rx.size());}
std::int32_t Serial::write_byte(std::uint8_t value)const{auto& tx=sim::state().serial[port_].tx;if(tx.empty()&&value=='\n')return 1;tx.push_back(value);sim::consume_serial(port_);return 1;}
std::int32_t Serial::write(const std::uint8_t* buffer,std::int32_t length)const{if(length<0)throw std::invalid_argument("Negative serial write length");auto& tx=sim::state().serial[port_].tx;tx.insert(tx.end(),buffer,buffer+length);sim::consume_serial(port_);return length;}
std::int32_t Serial::flush()const{sim::state().serial[port_].rx.clear();return 1;}
std::int32_t Serial::set_baudrate(std::int32_t baudrate)const{if(baudrate<=0)throw std::invalid_argument("Serial baudrate must be positive");return 1;}
namespace adi {
DigitalOut::DigitalOut(std::uint8_t port,bool value):port_(port>='A'&&port<='H'?port-'A'+1:port>='a'&&port<='h'?port-'a'+1:port){if(port_<1||port_>8)throw std::invalid_argument("ADI port must be A..H or 1..8");set_value(value);}
std::int32_t DigitalOut::set_value(bool value)const{sim::state().adi[port_]=value;return 1;}
std::int32_t DigitalOut::get_value()const{return sim::state().adi[port_];}
Pneumatics::Pneumatics(std::uint8_t port,bool extended,bool low):DigitalOut(port,extended?!low:low),extended_is_high_(!low){}
std::int32_t Pneumatics::extend()const{if(is_extended())return 0;return set_value(extended_is_high_);}
std::int32_t Pneumatics::retract()const{if(!is_extended())return 0;return set_value(!extended_is_high_);}
std::int32_t Pneumatics::toggle()const{return set_value(!get_value());}
bool Pneumatics::is_extended()const{return static_cast<bool>(get_value())==extended_is_high_;}
}
namespace lcd {
bool initialize(){sim::state().lcd_initialized=true;return true;}
bool is_initialized(){return sim::state().lcd_initialized;}
bool set_text(std::int16_t line,const std::string& text){if(line<0||line>7)throw std::invalid_argument("LCD line outside 0..7");sim::state().lcd[line]=text;return true;}
bool print(std::int16_t line,const char* fmt,...){va_list args;va_start(args,fmt);auto txt=sim::format(fmt,args);va_end(args);return set_text(line,txt);}
bool clear(){for(int i=0;i<8;++i)clear_line(i);return true;}
bool clear_line(std::int16_t line){return set_text(line,"");}
}
namespace competition {
bool is_disabled(){auto& m=sim::state().mode;return m!="autonomous"&&m!="opcontrol";}
bool is_autonomous(){return sim::state().mode=="autonomous";}
bool is_connected(){return true;}
std::uint8_t get_status(){return (is_disabled()?1:0)|(is_autonomous()?2:0)|4;}
}
namespace c {
// Preserve previously configured gearing when creating transient C API handles.
Motor handle(std::int8_t port){auto& m=sim::state().motors[std::abs(static_cast<int>(port))];return Motor(port,static_cast<MotorGearset>(m.gear),static_cast<MotorUnits>(m.units));}
std::int32_t motor_move(std::int8_t port,std::int32_t value){return handle(port).move(value);}
std::int32_t motor_move_voltage(std::int8_t port,std::int32_t value){return handle(port).move_voltage(value);}
std::int32_t motor_move_velocity(std::int8_t port,std::int32_t value){return handle(port).move_velocity(value);}
double motor_get_position(std::int8_t port){return handle(port).get_position();}
double motor_get_actual_velocity(std::int8_t port){return handle(port).get_actual_velocity();}
}
}

namespace {
std::string hex(const std::string& text){static const char* digits="0123456789abcdef";std::string out;for(unsigned char c:text){out+=digits[c>>4];out+=digits[c&15];}return out.empty()?"-":out;}
FILE* protocol_stream(){
#ifdef _WIN32
  int fd=_dup(_fileno(stdout));_dup2(_fileno(stderr),_fileno(stdout));return _fdopen(fd,"w");
#else
  int fd=dup(fileno(stdout));dup2(fileno(stderr),fileno(stdout));return fdopen(fd,"w");
#endif
}
void lifecycle(const std::string& requested){
  auto& s=sim::state();
  if(s.callback>=0&&requested!=s.callback_mode){sim::cancel(s.callback);s.callback=-1;s.callback_mode.clear();sim::zero_motors();}
  if(s.init<0)s.init=sim::spawn(initialize,"initialize");
  sim::drain();if(!sim::done(s.init))return;
  if(s.compinit<0)s.compinit=sim::spawn(competition_initialize,"competition_initialize");
  sim::drain();if(!sim::done(s.compinit))return;
  if(requested=="initialize"||requested=="competition_initialize")return;
  if(requested!=s.callback_mode){
    sim::cancel(s.callback);sim::zero_motors();s.callback_mode=requested;
    auto fn=requested=="autonomous"?autonomous:requested=="opcontrol"?opcontrol:disabled;
    s.callback=sim::spawn(fn,requested);
  }
  sim::drain();
}
}

int main(){
  FILE* out=protocol_stream();auto& s=sim::state();std::string line;
  while(std::getline(std::cin,line)){
    if(line=="QUIT")break;
    std::istringstream header(line);std::string tag,requested;double seconds,dt;
    if(!(header>>tag>>seconds>>dt>>requested)||tag!="STEP"){std::fprintf(out,"ERROR %s\nEND\n",hex("Invalid STEP protocol header").c_str());std::fflush(out);return 2;}
    if(!std::isfinite(seconds)||seconds*1000+1e-6<s.time){std::fprintf(out,"ERROR %s\nEND\n",hex("Simulation time must be finite and monotonic").c_str());std::fflush(out);return 2;}
    s.time=seconds*1000;s.mode=requested;
    while(std::getline(std::cin,line)&&line!="END"){
      std::istringstream input(line);input>>tag;
      if(tag=="M"){int port;double p,v;input>>port>>p>>v;auto& m=s.motors[port];m.position=p;m.velocity=v;auto& r=s.rotations[port];r.position=p;r.velocity=v;r.bound=true;}
      else if(tag=="R"){int port;double p,v;input>>port>>p>>v;auto& r=s.rotations[port];r.position=p;r.velocity=v;r.bound=true;}
      else if(tag=="I")input>>s.imu_heading>>s.imu_rotation;
      else if(tag=="D"){int port;double value;input>>port>>value;s.distance[port]=value;}
      else if(tag=="O")input>>s.otos[0]>>s.otos[1]>>s.otos[2];
      else if(tag=="A")for(auto& axis:s.analog)input>>axis;
      else if(tag=="B")input>>s.digital;
    }
    try{lifecycle(requested);}catch(const std::exception& e){s.error=e.what();}
    bool initialized=sim::done(s.init)&&sim::done(s.compinit);
    bool callback_done=(requested=="initialize"||requested=="competition_initialize")?initialized:sim::done(s.callback);
    std::fprintf(out,"STATE %d %d %.12g\n",initialized,callback_done,s.time/1000);
    bool enabled=requested=="initialize"||requested=="competition_initialize"||(initialized&&(requested=="autonomous"||requested=="opcontrol"));
    for(auto& entry:s.motors){auto& m=entry.second;if(!m.configured)continue;std::fprintf(out,"M %d %d %.12g %.12g %d %d\n",entry.first,enabled?m.mode:0,enabled?m.value:0,m.target_velocity,m.brake,m.gear);}
    for(auto& entry:s.adi)std::fprintf(out,"P %d %d\n",entry.first,entry.second);
    for(auto& entry:s.lcd)std::fprintf(out,"L %d %s\n",entry.first,hex(entry.second).c_str());
    for(auto& diagnostic:s.diagnostics)std::fprintf(out,"LOG %s\n",hex(diagnostic).c_str());s.diagnostics.clear();
    if(!s.error.empty())std::fprintf(out,"ERROR %s\n",hex(s.error).c_str());
    std::fprintf(out,"END\n");std::fflush(out);
    if(!s.error.empty())std::_Exit(2);
  }
  // Detached PROS tasks live until process shutdown, matching the V5 lifetime.
  std::_Exit(0);
}
