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

// Print the battery's voltage, the DC bus voltage (vbus_voltage) of one ODrive.
//
//   odrive_vbus --urdf FILE|-
//
// The boards are the ones ODriveHardwareInterface claims in the URDF's <ros2_control> block,
// tried in the order they appear there; the first that answers is read (they all sit on the
// one battery). Prints the voltage in volts with one decimal, e.g. "23.1", and exits 0; on a
// failure prints why on stderr and exits 1.
//
// Needs the boards to itself: run it before the stack (ros2_control_node holds them).

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "odrive_hardware_interface/odrive_usb.hpp"

using odrive::ODriveUSB;

namespace
{
constexpr char PLUGIN[] = "odrive_hardware_interface/ODriveHardwareInterface";

std::string serial_text(int64_t serial)
{
  std::ostringstream out;
  out << std::uppercase << std::hex << serial;
  return out.str();
}

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

// Every board's serial in the URDF, in order of appearance, each once.
std::vector<int64_t> serials_from_urdf(const std::string & urdf)
{
  std::vector<int64_t> serials;
  auto add = [&serials](const hardware_interface::ComponentInfo & component) {
      auto it = component.parameters.find("serial_number");
      if (it == component.parameters.end()) {
        return;
      }
      int64_t serial = std::stoll(it->second, nullptr, 16);
      for (auto known : serials) {
        if (known == serial) {
          return;
        }
      }
      serials.push_back(serial);
    };
  for (const auto & system : hardware_interface::parse_control_resources_from_urdf(urdf)) {
    if (system.hardware_plugin_name != PLUGIN) {
      continue;
    }
    for (const auto & component : system.sensors) {
      add(component);
    }
    for (const auto & component : system.joints) {
      add(component);
    }
  }
  if (serials.empty()) {
    throw std::runtime_error(std::string("no ODrive on ") + PLUGIN + " in the URDF");
  }
  return serials;
}

// The board's vbus_voltage, or why it could not be read.
bool read_vbus(int64_t serial, float & volts, std::string & why)
{
  ODriveUSB usb;
  std::vector<std::vector<int64_t>> wanted(2);
  wanted[1].push_back(serial);
  // init() prints "Connected to ODrive ..." for each board: not this tool's output.
  std::streambuf * cout_buffer = std::cout.rdbuf(nullptr);
  int ret = usb.init(wanted);
  std::cout.rdbuf(cout_buffer);
  if (ret != 0) {
    why = std::string("opening it: ") + libusb_error_name(ret);
    return false;
  }
  if (!usb.connected(serial)) {
    why = "not connected";
    return false;
  }
  ret = usb.read(serial, odrive::VBUS_VOLTAGE, volts);
  if (ret != 0) {
    why = std::string("reading vbus_voltage: ") + libusb_error_name(ret);
    return false;
  }
  return true;
}

int usage(const char * argv0)
{
  std::cerr << "usage: " << argv0 << " --urdf FILE|-\n";
  return 2;
}

int run(int argc, char ** argv)
{
  std::string urdf_path;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--urdf" && i + 1 < argc) {
      urdf_path = argv[++i];
    } else if (arg == "-h" || arg == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      return usage(argv[0]);
    }
  }
  if (urdf_path.empty()) {
    return usage(argv[0]);
  }

  std::string failures;
  for (auto serial : serials_from_urdf(read_urdf(urdf_path))) {
    float volts = 0.0f;
    std::string why;
    if (read_vbus(serial, volts, why)) {
      std::printf("%.1f\n", volts);
      return 0;
    }
    failures += (failures.empty() ? "" : "; ") + ("ODrive " + serial_text(serial) + ": " + why);
  }
  std::cerr << "no ODrive answered (is the stack running?): " << failures << std::endl;
  return 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
