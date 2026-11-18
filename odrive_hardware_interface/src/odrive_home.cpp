// Copyright 2026 Hornet
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Home ODrive axes on their min endstops, as the board does at power-up with
// config.startup_homing, without power-cycling it.
//
//   odrive_home [--timeout S] --urdf FILE|- JOINT [JOINT ...]
//
// Each joint's ODrive (serial_number) and axis are read from its <ros2_control> block in the
// URDF, as ODriveHardwareInterface reads them. Only an axis whose board homes it at power-up
// (config.startup_homing) is homed; any other joint is refused before anything moves. All the
// joints home at once, as at power-up: the
// axis drives at controller.config.homing_speed onto its min endstop, which sets its position to
// min_endstop.config.offset, then goes to 0 on its trapezoidal trajectory. Each homed axis is then
// put in closed loop, holding 0, as config.startup_closed_loop_control does after a startup
// homing; ODriveHardwareInterface takes it from there.
//
// Errors standing on the board (a min endstop pressed outside homing drops the axis to idle with
// MIN_ENDSTOP_PRESSED) are printed and cleared first. On a failure, the --timeout (30 s), SIGINT or
// SIGTERM, every axis still homing is put in idle. Exits 0 once every joint is homed and holding.
//
// Needs the boards to itself: run it before the stack (ros2_control_node holds them).

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "odrive_hardware_interface/odrive_usb.hpp"

using odrive::ODriveUSB;

namespace
{
// Firmware 0.5.6's AxisState.
constexpr int32_t AXIS_STATE_IDLE = 1;
constexpr int32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
constexpr int32_t AXIS_STATE_HOMING = 11;

constexpr char PLUGIN[] = "odrive_hardware_interface/ODriveHardwareInterface";
constexpr auto POLL = std::chrono::milliseconds(20);
// From the request to the axis reporting HOMING (the board's state machine picks it up at its
// next loop), and from a homing's end to it reporting CLOSED_LOOP_CONTROL.
constexpr double STATE_CHANGE_TIMEOUT = 2.0;

std::atomic<bool> interrupted{false};

void on_signal(int) {interrupted = true;}

std::string hex(uint64_t value)
{
  std::ostringstream out;
  out << "0x" << std::hex << value;
  return out.str();
}

std::string serial_text(int64_t serial)
{
  std::ostringstream out;
  out << std::uppercase << std::hex << serial;
  return out.str();
}

// Firmware 0.5.6's Axis.Error.
std::string axis_error_text(uint32_t error)
{
  static const std::vector<std::pair<uint32_t, const char *>> names = {
    {0x00000001, "INVALID_STATE"}, {0x00000800, "WATCHDOG_TIMER_EXPIRED"},
    {0x00001000, "MIN_ENDSTOP_PRESSED"}, {0x00002000, "MAX_ENDSTOP_PRESSED"},
    {0x00004000, "ESTOP_REQUESTED"}, {0x00020000, "HOMING_WITHOUT_ENDSTOP"},
    {0x00040000, "OVER_TEMP"}, {0x00080000, "UNKNOWN_POSITION"},
  };
  std::string text = hex(error);
  std::string known;
  uint32_t rest = error;
  for (const auto & [bit, name] : names) {
    if (error & bit) {
      known += (known.empty() ? "" : "|") + std::string(name);
      rest &= ~bit;
    }
  }
  if (!known.empty()) {
    text += " (" + known + (rest ? "|" + hex(rest) : "") + ")";
  }
  return text;
}

class UsbError : public std::runtime_error
{
public:
  UsbError(const std::string & what, int code)
  : std::runtime_error(what + ": " + libusb_error_name(code))
  {
  }
};

struct Axis
{
  std::string joint;
  int64_t serial;
  int axis;

  // Homing progress.
  bool homing_seen = false;
  bool done = false;
  bool failed = false;
  std::string result;

  int16_t ep(int16_t endpoint) const {return endpoint + odrive::per_axis_offset * axis;}
};

class Board
{
public:
  explicit Board(ODriveUSB & usb)
  : usb_(usb) {}

  template<typename T>
  T read(Axis & a, int16_t endpoint)
  {
    T value{};
    int ret = usb_.read(a.serial, a.ep(endpoint), value);
    if (ret != 0) {
      throw UsbError(a.joint + ": reading endpoint " + std::to_string(endpoint), ret);
    }
    return value;
  }

  void request(Axis & a, int32_t state)
  {
    // int32, as ODriveHardwareInterface writes it.
    int ret = usb_.write(a.serial, a.ep(odrive::AXIS__REQUESTED_STATE), state);
    if (ret != 0) {
      throw UsbError(a.joint + ": requesting state " + std::to_string(state), ret);
    }
  }

  void clear_errors(int64_t serial)
  {
    int ret = usb_.call(serial, odrive::CLEAR_ERRORS);
    if (ret != 0) {
      throw UsbError("ODrive " + serial_text(serial) + ": clear_errors()", ret);
    }
  }

  // Every error the axis holds, as text; empty when there is none.
  std::string errors(Axis & a)
  {
    std::string text;
    auto add = [&text](const std::string & what, uint64_t value, const std::string & shown) {
        if (value) {
          text += (text.empty() ? "" : ", ") + what + " " + shown;
        }
      };
    auto axis_error = read<uint32_t>(a, odrive::AXIS__ERROR);
    auto motor_error = read<uint64_t>(a, odrive::AXIS__MOTOR__ERROR);
    auto encoder_error = read<uint16_t>(a, odrive::AXIS__ENCODER__ERROR);
    auto controller_error = read<uint8_t>(a, odrive::AXIS__CONTROLLER__ERROR);
    add("axis error", axis_error, axis_error_text(axis_error));
    add("motor error", motor_error, hex(motor_error));
    add("encoder error", encoder_error, hex(encoder_error));
    add("controller error", controller_error, hex(controller_error));
    return text;
  }

private:
  ODriveUSB & usb_;
};

std::string read_urdf(const std::string & path)
{
  std::stringstream text;
  if (path == "-") {
    text << std::cin.rdbuf();
  } else {
    std::ifstream file(path);
    if (!file) {
      throw std::runtime_error("cannot read " + path);
    }
    text << file.rdbuf();
  }
  return text.str();
}

std::vector<Axis> axes_from_urdf(const std::string & urdf, const std::vector<std::string> & joints)
{
  std::vector<Axis> axes;
  auto systems = hardware_interface::parse_control_resources_from_urdf(urdf);
  for (const auto & joint : joints) {
    bool found = false;
    for (const auto & system : systems) {
      if (system.hardware_plugin_name != PLUGIN) {
        continue;
      }
      for (const auto & component : system.joints) {
        if (component.name != joint) {
          continue;
        }
        Axis a;
        a.joint = joint;
        a.serial = std::stoll(component.parameters.at("serial_number"), nullptr, 16);
        a.axis = std::stoi(component.parameters.at("axis"));
        axes.push_back(a);
        found = true;
      }
    }
    if (!found) {
      throw std::runtime_error(
        joint + ": no such joint on " + std::string(PLUGIN) + " in the URDF");
    }
  }
  return axes;
}

double seconds_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

int usage(const char * argv0)
{
  std::cerr << "usage: " << argv0 << " [--timeout S] --urdf FILE|- JOINT [JOINT ...]\n";
  return 2;
}

int run(int argc, char ** argv)
{
  double timeout = 30.0;
  std::string urdf_path;
  std::vector<std::string> joints;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--timeout" && i + 1 < argc) {
      timeout = std::stod(argv[++i]);
    } else if (arg == "--urdf" && i + 1 < argc) {
      urdf_path = argv[++i];
    } else if (arg == "-h" || arg == "--help") {
      usage(argv[0]);
      return 0;
    } else if (!arg.empty() && arg[0] == '-') {
      return usage(argv[0]);
    } else {
      joints.push_back(arg);
    }
  }
  if (urdf_path.empty() || joints.empty()) {
    return usage(argv[0]);
  }

  std::vector<Axis> axes = axes_from_urdf(read_urdf(urdf_path), joints);
  std::set<int64_t> serials;
  for (const auto & a : axes) {
    serials.insert(a.serial);
  }

  // init() wants [sensors' serials, joints' serials].
  std::vector<std::vector<int64_t>> wanted(2);
  wanted[1].assign(serials.begin(), serials.end());
  ODriveUSB usb;
  // init() prints "Connected to ODrive ..." for each board: not this tool's output.
  std::streambuf * cout_buffer = std::cout.rdbuf(nullptr);
  int ret = usb.init(wanted);
  std::cout.rdbuf(cout_buffer);
  if (ret != 0) {
    std::string which;
    for (auto serial : serials) {
      which += " " + serial_text(serial);
    }
    throw UsbError("opening ODrive" + which + " (is the stack running?)", ret);
  }
  for (auto serial : serials) {
    if (!usb.connected(serial)) {
      throw std::runtime_error("ODrive " + serial_text(serial) + " is not connected");
    }
  }
  Board board(usb);

  // What each axis holds before homing, and whether it can home at all.
  std::set<int64_t> to_clear;
  for (auto & a : axes) {
    std::string where = "ODrive " + serial_text(a.serial) + " axis" + std::to_string(a.axis);
    // The board's own word that this axis homes: an endstop left enabled is not (a wheel axis
    // with one on GPIO8 from an old setup drove for as long as it was let).
    if (!board.read<bool>(a, odrive::AXIS__CONFIG__STARTUP_HOMING)) {
      throw std::runtime_error(
        a.joint + ": " + where + " is not set to home (config.startup_homing is false): refused");
    }
    if (!board.read<bool>(a, odrive::AXIS__MIN_ENDSTOP__CONFIG__ENABLED)) {
      throw std::runtime_error(
        a.joint + ": " + where + " has no min endstop enabled: it cannot home");
    }
    if (!board.read<bool>(a, odrive::AXIS__MOTOR__IS_CALIBRATED) ||
      !board.read<bool>(a, odrive::AXIS__ENCODER__IS_READY))
    {
      throw std::runtime_error(
        a.joint + ": motor not calibrated or encoder not ready (the ODrive's startup sequence did "
        "not finish: power-cycle it)");
    }
    std::string errors = board.errors(a);
    if (!errors.empty()) {
      std::cout << a.joint << ": clearing " << errors << std::endl;
      to_clear.insert(a.serial);
    }
  }
  for (auto serial : to_clear) {
    board.clear_errors(serial);
  }

  // From here on, whatever ends the run leaves no axis homing.
  auto stop_homing = [&board, &axes]() {
      for (auto & a : axes) {
        if (!a.done) {
          try {
            board.request(a, AXIS_STATE_IDLE);
          } catch (const std::exception & e) {
            std::cout << e.what() << std::endl;
          }
        }
      }
    };

  auto start = std::chrono::steady_clock::now();
  try {
    for (auto & a : axes) {
      board.request(a, AXIS_STATE_HOMING);
    }

    size_t left = axes.size();
    while (left > 0) {
      if (interrupted) {
        stop_homing();
        std::cout << "Homing interrupted; axes put in idle" << std::endl;
        return 130;
      }
      if (seconds_since(start) > timeout) {
        stop_homing();
        for (auto & a : axes) {
          if (!a.done) {
            std::cout << a.joint << ": FAILED: not homed after " << timeout
                      << " s (endstop never reached?); put in idle" << std::endl;
          }
        }
        return 1;
      }
      std::this_thread::sleep_for(POLL);
      for (auto & a : axes) {
        if (a.done) {
          continue;
        }
        auto state = board.read<uint8_t>(a, odrive::AXIS__CURRENT_STATE);
        if (state == AXIS_STATE_HOMING) {
          a.homing_seen = true;
          continue;
        }
        if (!a.homing_seen) {
          // Not picked up yet, or refused at once.
          std::string errors = board.errors(a);
          if (errors.empty() && seconds_since(start) < STATE_CHANGE_TIMEOUT) {
            continue;
          }
          a.failed = true;
          a.result = errors.empty() ? "the axis never entered homing (state " +
            std::to_string(state) + ")" :
            errors;
        } else {
          // Homing over: it ends in idle, homed or not.
          std::string errors = board.errors(a);
          bool homed = board.read<bool>(a, odrive::AXIS__IS_HOMED);
          if (!errors.empty() || !homed) {
            a.failed = true;
            a.result = errors.empty() ? "homing ended without is_homed" : errors;
          } else {
            char took[48];
            std::snprintf(took, sizeof(took), "homed in %.1f s", seconds_since(start));
            a.result = took;
          }
        }
        a.done = true;
        --left;
      }
    }
  } catch (...) {
    stop_homing();
    throw;
  }

  // Hold where homing left each axis (0), as the startup sequence's closed loop does.
  bool ok = true;
  for (auto & a : axes) {
    if (a.failed) {
      std::cout << a.joint << ": FAILED: " << a.result << std::endl;
      ok = false;
      continue;
    }
    board.request(a, AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  auto hold_start = std::chrono::steady_clock::now();
  for (auto & a : axes) {
    if (a.failed) {
      continue;
    }
    uint8_t state = 0;
    while (seconds_since(hold_start) < STATE_CHANGE_TIMEOUT) {
      state = board.read<uint8_t>(a, odrive::AXIS__CURRENT_STATE);
      if (state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
        break;
      }
      std::this_thread::sleep_for(POLL);
    }
    std::string errors = board.errors(a);
    auto position = board.read<float>(a, odrive::AXIS__ENCODER__POS_ESTIMATE);
    char at[32];
    std::snprintf(at, sizeof(at), "%.3f", position);
    if (state != AXIS_STATE_CLOSED_LOOP_CONTROL || !errors.empty()) {
      std::cout << a.joint << ": FAILED: " << a.result << " at " << at
                << " turns, but closed loop refused (state " << std::to_string(state)
                << (errors.empty() ? "" : ", " + errors) << ")" << std::endl;
      ok = false;
    } else {
      std::cout << a.joint << ": " << a.result << ", holding " << at << " turns" << std::endl;
    }
  }
  return ok ? 0 : 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::cout << "Homing: FAILED: " << e.what() << std::endl;
    return 1;
  }
}
