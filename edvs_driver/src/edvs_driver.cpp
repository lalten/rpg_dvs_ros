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
	  device = new Edvs::Device();
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
  	// std::cout << "Event: <x, y, t, p> = <" << x << ", " << y << ", " << timestamp << ", " << polarity << ">" << std::endl;
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

	// TODO: does the eDVS support that many biases?

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

  uint32_t cas = parameters["cas"].get_value();
  biases[0] = (uint8_t) (cas >> 16);
  biases[1] = (uint8_t) (cas >> 8);
  biases[2] = (uint8_t) (cas >> 0);

  uint32_t injGnd = parameters["injGnd"].get_value();
  biases[3] = (uint8_t) (injGnd >> 16);
  biases[4] = (uint8_t) (injGnd >> 8);
  biases[5] = (uint8_t) (injGnd >> 0);

  uint32_t reqPd = parameters["reqPd"].get_value();
  biases[6] = (uint8_t) (reqPd >> 16);
  biases[7] = (uint8_t) (reqPd >> 8);
  biases[8] = (uint8_t) (reqPd >> 0);

  uint32_t puX = parameters["puX"].get_value();
  biases[9] = (uint8_t) (puX >> 16);
  biases[10] = (uint8_t) (puX >> 8);
  biases[11] = (uint8_t) (puX >> 0);

  uint32_t diffOff = parameters["diffOff"].get_value();
  biases[12] = (uint8_t) (diffOff >> 16);
  biases[13] = (uint8_t) (diffOff >> 8);
  biases[14] = (uint8_t) (diffOff >> 0);

  uint32_t req = parameters["req"].get_value();
  biases[15] = (uint8_t) (req >> 16);
  biases[16] = (uint8_t) (req >> 8);
  biases[17] = (uint8_t) (req >> 0);

  uint32_t refr = parameters["refr"].get_value();
  biases[18] = (uint8_t) (refr >> 16);
  biases[19] = (uint8_t) (refr >> 8);
  biases[20] = (uint8_t) (refr >> 0);

  uint32_t puY = parameters["puY"].get_value();
  biases[21] = (uint8_t) (puY >> 16);
  biases[22] = (uint8_t) (puY >> 8);
  biases[23] = (uint8_t) (puY >> 0);

  uint32_t diffOn = parameters["diffOn"].get_value();
  biases[24] = (uint8_t) (diffOn >> 16);
  biases[25] = (uint8_t) (diffOn >> 8);
  biases[26] = (uint8_t) (diffOn >> 0);

  uint32_t diff = parameters["diff"].get_value();
  biases[27] = (uint8_t) (diff >> 16);
  biases[28] = (uint8_t) (diff >> 8);
  biases[29] = (uint8_t) (diff >> 0);

  uint32_t foll = parameters["foll"].get_value();
  biases[30] = (uint8_t) (foll >> 16);
  biases[31] = (uint8_t) (foll >> 8);
  biases[32] = (uint8_t) (foll >> 0);

  uint32_t Pr = parameters["Pr"].get_value();
  biases[33] = (uint8_t) (Pr >> 16);
  biases[34] = (uint8_t) (Pr >> 8);
  biases[35] = (uint8_t) (Pr >> 0);

  // see http://inilabs.com/support/edvs/#h.bctg2sgwitln
  std::stringstream cmdstr;
  boost::format f("!B%d=%d\n");
  for (int i=0; i<12*3; ++i) {
	  cmdstr << f % i % biases[i];
  }
  cmdstr << "!BF\n";

  device_mutex.lock();
  device->WriteCommand(cmdstr.str());
  device_mutex.unlock();

  return true;
}

} // namespace
