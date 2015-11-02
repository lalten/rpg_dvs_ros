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

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace dvs {

EDVS_Driver::EDVS_Driver(std::string dvs_serial_number, bool master) {
  // initialize parameters (min, max, value)
  parameters.insert(std::pair<std::string, Parameter>("cas", Parameter(0, 16777215, 1992)));
  parameters.insert(std::pair<std::string, Parameter>("injGnd", Parameter(0, 16777215, 1108364)));
  parameters.insert(std::pair<std::string, Parameter>("reqPd", Parameter(0, 16777215, 16777215)));
  parameters.insert(std::pair<std::string, Parameter>("puX", Parameter(0, 16777215, 8159221)));
  parameters.insert(std::pair<std::string, Parameter>("diffOff", Parameter(0, 16777215, 132)));
  parameters.insert(std::pair<std::string, Parameter>("req", Parameter(0, 16777215, 309590)));
  parameters.insert(std::pair<std::string, Parameter>("refr", Parameter(0, 16777215, 969)));
  parameters.insert(std::pair<std::string, Parameter>("puY", Parameter(0, 16777215, 16777215)));
  parameters.insert(std::pair<std::string, Parameter>("diffOn", Parameter(0, 16777215, 209996)));
  parameters.insert(std::pair<std::string, Parameter>("diff", Parameter(0, 16777215, 13125)));
  parameters.insert(std::pair<std::string, Parameter>("foll", Parameter(0, 16777215, 271)));
  parameters.insert(std::pair<std::string, Parameter>("Pr", Parameter(0, 16777215, 217)));

  wrapAdd = 0;
  lastTimestamp = 0;

  Edvs::EventCallbackType cbf = boost::bind(&EDVS_Driver::callback, this, _1);

  device_mutex.lock();
  try {
	  device = new Edvs::Device(Edvs::B1000k);
	  capture = new Edvs::EventCapture(*device, cbf);
  } catch (std::runtime_error &ex) {
	  std::cerr << ex.what() <<std::endl;
  }
  device_mutex.unlock();

  camera_id = dvs_serial_number;


  // put into slave mode?
  if (!master) {
    std::cout << "Setting camera (" << camera_id << ") as slave!" << std::endl;
    device_mutex.lock();
    device->WriteCommand("!ETS");
    device_mutex.unlock();
  }
  // TODO: do we need to set master mode as well? are settings volatile?

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
  	// TODO: where do we get the timestamp from?
  	Event e;
  	e.x=i->x;
  	e.y=i->y;
  	e.polarity=i->parity;
  	e.timestamp=0;
  	event_buffer.push_back(e);
  	std::cout << "Event: <x, y, t, p> = <" << e.x << ", " << e.y << ", " << e.timestamp << ", " << e.polarity << ">" << std::endl;
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
  uint8_t biases[12 * 3];

  // see http://inilabs.com/support/edvs/#h.bctg2sgwitln
  std::stringstream cmdstr;
  boost::format f("!B%d=%d\n");

  int i=0;

  cmdstr << f % i++ %  parameters["cas"].get_value();
  cmdstr << f % i++ %  parameters["injGnd"].get_value();
  cmdstr << f % i++ %  parameters["reqPd"].get_value();
  cmdstr << f % i++ %  parameters["puX"].get_value();
  cmdstr << f % i++ %  parameters["diffOff"].get_value();
  cmdstr << f % i++ %  parameters["req"].get_value();
  cmdstr << f % i++ %  parameters["refr"].get_value();
  cmdstr << f % i++ %  parameters["puY"].get_value();
  cmdstr << f % i++ %  parameters["diffOn"].get_value();
  cmdstr << f % i++ %  parameters["diff"].get_value();
  cmdstr << f % i++ %  parameters["foll"].get_value();
  cmdstr << f % i++ %  parameters["Pr"].get_value();

  cmdstr << "!BF\n";

  device_mutex.lock();
  device->WriteCommand(cmdstr.str());
  device_mutex.unlock();

  return true;
}

} // namespace
