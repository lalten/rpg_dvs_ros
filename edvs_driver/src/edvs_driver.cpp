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

#include "edvs_driver/edvs_driver.h"

#include <stdexcept>
#include <boost/algorithm/string/replace.hpp>

#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <ros/ros.h>

namespace dvs {

EDVS_Driver::EDVS_Driver(std::string edvs_port, bool master) {
  // initialize parameters (min, max, value)
  parameters.insert(std::pair<std::string, Parameter>("cas", Parameter(0, 16777215, 54)));
  parameters.insert(std::pair<std::string, Parameter>("injGnd", Parameter(0, 16777215, 1108364)));
  parameters.insert(std::pair<std::string, Parameter>("reqPd", Parameter(0, 16777215, 16777215)));
  parameters.insert(std::pair<std::string, Parameter>("puX", Parameter(0, 16777215, 8159221)));
  parameters.insert(std::pair<std::string, Parameter>("diffOff", Parameter(0, 16777215, 132)));
  parameters.insert(std::pair<std::string, Parameter>("req", Parameter(0, 16777215, 159147)));
  parameters.insert(std::pair<std::string, Parameter>("refr", Parameter(0, 16777215, 6)));
  parameters.insert(std::pair<std::string, Parameter>("puY", Parameter(0, 16777215, 16777215)));
  parameters.insert(std::pair<std::string, Parameter>("diffOn", Parameter(0, 16777215, 482443)));
  parameters.insert(std::pair<std::string, Parameter>("diff", Parameter(0, 16777215, 30153)));
  parameters.insert(std::pair<std::string, Parameter>("foll", Parameter(0, 16777215, 51)));
  parameters.insert(std::pair<std::string, Parameter>("Pr", Parameter(0, 16777215, 3)));

  integratedTimeSinceReset = 0;

  last_timestamp = 0;

  Edvs::EventCallbackType cbf = boost::bind(&EDVS_Driver::callback, this, _1);

  device_mutex.lock();
  try {
    if(edvs_port!="") {
      device = new Edvs::Device(Edvs::B4000k, edvs_port);
    } else {
      device = new Edvs::Device(Edvs::B4000k); // single camera mode, using default port
    }
    // reset device to defaults before starting event capture with specific protocol
    device->WriteCommand("R\n");

	  capture = new Edvs::EventCapture(*device, cbf);
  } catch (std::runtime_error &ex) {
    ROS_ERROR("edvs_driver error %s",ex.what());
    device_mutex.unlock();
    device = nullptr; // Not sure this is necessary. Also, do we have a memory leak if the exception comes from eventcapture?
    return;
  }
  device_mutex.unlock();

  // eDVS camera don't have proper serial numbers, so for now we construct one from its port
  camera_id = boost::replace_all_copy(edvs_port, "/", "_");


  // put into master or slave mode?
  if (master) {
    ROS_INFO("Setting camera (%s) as master.", camera_id.c_str());
    device_mutex.lock();
    device->WriteCommand("!ETM+");
    device_mutex.unlock();
  } else {
    ROS_INFO("Setting camera (%s) as slave.", camera_id.c_str());
    device_mutex.lock();
    device->WriteCommand("!ETS");
    device_mutex.unlock();
  }

}

EDVS_Driver::~EDVS_Driver() {
  device_mutex.lock();
  delete capture;
  delete device;
  device_mutex.unlock();
}

void EDVS_Driver::callback(const std::vector<Edvs::Event>& events) {
  // Handle data.
  event_buffer_mutex.lock();

  for (std::vector<Edvs::Event>::const_iterator i=events.begin(); i<events.end(); ++i) {
	if(i->time_delta < last_timestamp) {
		integratedTimeSinceReset += last_timestamp;
	}
    Event e {i->x, i->y, i->polarity, integratedTimeSinceReset + i->time_delta};
  	event_buffer.push_back(e);

  	last_timestamp = i->time_delta;
  	//std::cout << "Event from "<<camera_id<<": <x, y, t, p> = <" << e.x << ",\t" << e.y << ",\t" << e.timestamp << ",\t" << e.polarity << ">" << std::endl;
  }

  event_buffer_mutex.unlock();
}

std::vector<Event> EDVS_Driver::get_events() {
  event_buffer_mutex.lock();
  std::vector<Event> buffer_copy = event_buffer;
  event_buffer.clear();
  event_buffer_mutex.unlock();
  return buffer_copy;
}

void EDVS_Driver::resetTimestamps() {
  event_buffer_mutex.lock();
  integratedTimeSinceReset = 0;
  event_buffer_mutex.unlock();

  device_mutex.lock();
  device->WriteCommand("!ET0\n");
  device_mutex.unlock();
}

bool EDVS_Driver::change_parameter(std::string parameter, uint32_t value) {
  // does parameter exist?
  if (parameters.find(parameter) != parameters.end()) {
    // did it change? (only if within range)
    if (parameters[parameter].set_value(value)) {
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

bool EDVS_Driver::change_parameters(uint32_t cas, uint32_t injGnd, uint32_t reqPd, uint32_t puX,
                                   uint32_t diffOff, uint32_t req, uint32_t refr, uint32_t puY,
                                   uint32_t diffOn, uint32_t diff, uint32_t foll, uint32_t Pr) {

  change_parameter("cas", cas);
  change_parameter("injGnd", injGnd);
  change_parameter("reqPd", reqPd);
  change_parameter("puX", puX);
  change_parameter("diffOff", diffOff);
  change_parameter("req", req);
  change_parameter("refr", refr);
  change_parameter("puY", puY);
  change_parameter("diffOn", diffOn);
  change_parameter("diff", diff);
  change_parameter("foll", foll);
  change_parameter("Pr", Pr);

  return send_parameters();
}

bool EDVS_Driver::send_parameters() {
  // see http://inilabs.com/support/edvs/#h.bctg2sgwitln
  // Warning: Biases must only be set during Event mode (E+)! Otherwise the camera will freeze and require a power cycle.

  std::stringstream cmdstr;
  boost::format f("!B%d=%d\n");

  cmdstr << "E+\n";

  int i=0;

  cmdstr << f % i++ % parameters["cas"].get_value();
  cmdstr << f % i++ % parameters["injGnd"].get_value();
  cmdstr << f % i++ % parameters["reqPd"].get_value();
  cmdstr << f % i++ % parameters["puX"].get_value();
  cmdstr << f % i++ % parameters["diffOff"].get_value();
  cmdstr << f % i++ % parameters["req"].get_value();
  cmdstr << f % i++ % parameters["refr"].get_value();
  cmdstr << f % i++ % parameters["puY"].get_value();
  cmdstr << f % i++ % parameters["diffOn"].get_value();
  cmdstr << f % i++ % parameters["diff"].get_value();
  cmdstr << f % i++ % parameters["foll"].get_value();
  cmdstr << f % i++ % parameters["Pr"].get_value();

  cmdstr << "!BF\n";

  device_mutex.lock();
  ROS_INFO("Sending to (%s): \"%s\"", camera_id.c_str(), cmdstr.str().c_str());
  device->WriteCommand(cmdstr.str());
  device_mutex.unlock();

  return true;
}

} // namespace
