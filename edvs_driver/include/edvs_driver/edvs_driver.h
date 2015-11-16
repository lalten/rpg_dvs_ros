// This file is part of DVS-ROS - the RPG DVS ROS Package
//
// DVS-ROS is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DVS-ROS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DVS-ROS.  If not, see <http://www.gnu.org/licenses/>.

#ifndef EDVS_DRIVER_H_
#define EDVS_DRIVER_H_

#include "Edvs.h"

#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <stdio.h>

namespace dvs {

struct Event {
  uint16_t x, y;
  bool polarity;
  uint64_t timestamp;
};

class EDVS_Driver {
public:
  EDVS_Driver(std::string dvs_serial_number = "", bool master = true);
  ~EDVS_Driver();

  std::vector<Event> get_events();

  bool change_parameters(uint32_t cas, uint32_t injGnd, uint32_t reqPd, uint32_t puX,
                         uint32_t diffOff, uint32_t req, uint32_t refr, uint32_t puY,
                         uint32_t diffOn, uint32_t diff, uint32_t foll, uint32_t Pr);

  void callback(const std::vector<Edvs::Event>& events);

  void resetTimestamps();

  inline std::string get_camera_id() {
    return camera_id;
  }

  inline bool isDeviceRunning() {
	  return device != nullptr;
  }

private:
  bool change_parameter(std::string parameter, uint32_t value);
  bool send_parameters();

  boost::mutex event_buffer_mutex;
  boost::mutex device_mutex;

  // Device handle and capture object
  Edvs::Device *device = nullptr;
  Edvs::EventCapture *capture = nullptr;

  // event buffer
  std::vector<dvs::Event> event_buffer;

  // buffers
  static const uint32_t bufferNumber = 8;
  static const uint32_t bufferSize = 4096;

  uint64_t integratedTimeSinceReset; // sum of time_deltas in us

  class Parameter {
  public:
    Parameter(uint32_t min = 0, uint32_t max = 0, uint32_t value = 0) :
      _min(min), _max(max), _value(value) {}

    uint32_t get_value() { return _value; }

    bool set_value(uint32_t value) {
      if (value >= _min && value <= _max) {
        _value = value;
        return true;
      }
      else {
        return false;
      }
    }
  private:
    uint32_t _min;
    uint32_t _max;
    uint32_t _value;

  };

  // parameters
  std::map<std::string, Parameter> parameters;

  // camera name
  std::string camera_id;
};

} // namespace
#endif
