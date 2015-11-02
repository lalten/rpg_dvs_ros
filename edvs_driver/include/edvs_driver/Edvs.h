/*
 * Edvs.h
 *
 *  Created on: Mar 23, 2011
 *  Changed on: Mar 25, 2011
 *      Author: David Weikersdorfer
 *
 * C++ classes and functions to access the eDVS128 sensor
 *
 * The code has been tested under Ubuntu 10.04 with gcc 4.4.3
 *
 * Requirements:
 *   -- boost --
 *   All headers are required, but you only need to compile
 *   the library boost_thread library.
 *   Please add the boost include directory to your include
 *   path and the boost lib directory to your library path.
 *   Example: -I$BOOST_INCLUDE -L$BOOST_LOB -lboost_thread
 *
 * Code example:
 *

// header with Edvs functions
#include "Edvs.h"
// to write/read to/from the console
#include <iostream>

// C style event callback function
void OnEvent(const std::vector<Edvs::Event>& events)
{
	std::cout << "Got " << events.size() << " events: ";
	// iterating over the events and writing them to the console
	for(std::vector<Edvs::Event>::const_iterator it=events.begin(); it!=events.end(); it++) {
		// int x = it->x; int y = it->y; bool parity = it->parity
		std::cout << *it << ", ";
	}
	std::cout << std::endl;
}

// the main program
int main(int argc, char* argv[])
{
	// open device and specify connection speed
	Edvs::Device device(Edvs::B4000k);
	// start event capture
	Edvs::EventCapture capture(device, OnEvent);
	// read in string from the console and post them as command strings
	// try e.g. 0/1/2 to change the LED state (off/on/blicking)
	// press q to quit the program
	std::string str;
	while(str != "q") {
		std::cin >> str;
		device.WriteCommand(str + "\n");
	}
	return 0;
}

 *
 * Example how to change the device speed:
 * This code changes the device baud rate.
 * It's inside brackets so that the device connection
 * is closed immediately. We cannot use this connection
 * to access data because its connection speed is not
 * changed automatically.
 *
	{ // set rate to 4M, keep the brackets!
		Edvs::Device m;
		m.WriteCommand("!S=4000000\n");
	}

 */

#ifndef EDVS_H_
#define EDVS_H_

#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <iostream>
#include <string>
#include <stdexcept>
#include <vector>
#include <fcntl.h>
#include <termios.h>

//#define VERBOSE

namespace Edvs
{
	enum Baudrate {
		B921k, B1000k, B2000k, B4000k
	};

	const int cDeviceDisplaySize = 128;

	const bool cEnableFillRead = false;

	namespace Impl
	{
		/** Actual reading and writing to the port */
		struct SerialPort
		{
			struct IOException
			: public std::runtime_error {
				IOException(const std::string& msg)
				: std::runtime_error(msg) {}
			};
			/** Opens the serial port */
			SerialPort(Baudrate br=B921k) {
				const char* port = "/dev/ttyUSB0";
			#ifdef VERBOSE
				std::cout << "Edvs: Openening port " << port << " with baud rate " << br << std::endl;
			#endif
				port_ = open(port, O_RDWR /*| O_NOCTTY/ * | O_NDELAY*/);
				if (port_ == -1) {
					throw IOException("Unable to open port " + std::string(port));
				}
			//	fcntl(fd, F_SETFL, 0);
				struct termios settings;
				tcgetattr(port_, &settings);
				int rate = 0;
				switch(br) {
				case B921k: rate = B921600; break;
				case B1000k: rate = B1000000; break;
				case B2000k: rate = B2000000; break;
				case B4000k: rate = B4000000; break;
				default:
					throw IOException("Unkown baud rate");
				}
				cfsetispeed(&settings, rate); // set baud rates
				cfsetospeed(&settings, rate);
				settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8; // 8 bits
				settings.c_cflag |= CLOCAL | CREAD;
				settings.c_cflag |= CRTSCTS; // use hardware handshaking
				settings.c_iflag = IGNBRK;
				settings.c_oflag = 0;
				settings.c_lflag = 0;
				settings.c_cc[VMIN] = 1; // minimum number of characters to receive before satisfying the read.
				settings.c_cc[VTIME] = 5; // time between characters before satisfying the read.
				// write modified record of parameters to port
				tcsetattr(port_, TCSANOW, &settings);
			#ifdef VERBOSE
				std::cout << "Edvs: Device opened successfully!" << std::endl;
			#endif
			}
			/** Closes the serial port */
			~SerialPort() {
				close(port_);
			#ifdef VERBOSE
				std::cout << "Edvs: Device closed successfully!" << std::endl;
			#endif
			}
			/** Writes a command string to the serial port
			 * Warning: Command string must end with a '\n'!
			 */
			void WriteCommand(const std::string& str) {
				int n = ::write(port_, str.c_str(), str.length());
				if(n != (int)str.length()) {
					throw IOException("Could not write correct number of bytes!");
				}
			}
			/** Reads some data from the serial port */
			size_t ReadBinaryData(size_t n, char* data) {
				ssize_t actual = ::read(port_, data, n);
				if(actual < 0) {
					throw IOException("Could not read from port!");
				}
				return size_t(actual);
			}
		private:
			int port_;
		};
	}

	/** Wrapper for device implementation */
	struct Device
	{
		Device() {}
		Device(Baudrate br)
		: device_(new Impl::SerialPort(br)) {
		}
		void WriteCommand(const std::string& str) {
			device_->WriteCommand(str);
		}
		size_t ReadBinaryData(size_t n, char* data) {
			return device_->ReadBinaryData(n, data);
		}
	private:
		boost::shared_ptr<Impl::SerialPort> device_;
	};

	/** An eDVS event */
	struct Event {
		unsigned int x, y;
		bool parity;
	};

	/** Type of event callback function */
	typedef boost::function<void(const std::vector<Event>&)> EventCallbackType;

	namespace Impl {
		/** Event capturing */
		struct EventCapture
		{
			static const size_t cDefaultBufferSize = 8192;
			EventCapture() {
			}
			EventCapture(Device device, EventCallbackType f, size_t buffer_size=cDefaultBufferSize)
			: device_(device), event_callback_(f), buffer_size_(buffer_size), running_(false) {
				StartEventCapture();
			}
			~EventCapture() {
				StopEventCapture();
			}
		private:
			void StartEventCapture() {
				if(running_) {
					throw "Already running!";
				}
				device_.WriteCommand("!E0\n");
				device_.WriteCommand("E+\n");
				running_ = true;
				thread_ = boost::thread(&Edvs::Impl::EventCapture::Run, this);
			}
			void Run() {
				const unsigned char cHighBitMask = 0x80;
				const unsigned char cLowerBitsMask = 0x7F;
				unsigned char* buffer = new unsigned char[buffer_size_];
				std::vector<Event>* buff1 = new std::vector<Event>();
				buff1->reserve(buffer_size_ / 2 + 1);
				std::vector<Event>* buff2 = new std::vector<Event>();
				buff2->reserve(buffer_size_ / 2 + 1);
				std::vector<Event>* buffA = buff1;
				std::vector<Event>* buffB = buff2;
				while(running_) {
					size_t bytes_read = 0;
					if(cEnableFillRead) {
						while(bytes_read < buffer_size_) {
							size_t n = device_.ReadBinaryData(buffer_size_ - bytes_read, (char*)buffer + bytes_read);
							bytes_read += n;
							if(n == 0) {
								break;
							}
						}
					} else {
						bytes_read = device_.ReadBinaryData(buffer_size_, (char*)buffer);
					}
					buffA->clear();
					for(size_t i=0; i<bytes_read; i+=2) {
						// get to bytes
						unsigned char a = buffer[i];
						unsigned char b = buffer[i + 1];
						// check for and parse 0yyyyyyy pxxxxxxx
						if(!(a & cHighBitMask)) { // check that the high bit o first byte is 1
							// the serial port missed a byte somewhere ...
							// skip one byte to jump to the next event
							i ++;
							continue;
						}
						// create event
						Event e;
						e.x = (unsigned int)(b & cLowerBitsMask);
						e.y = (unsigned int)(a & cLowerBitsMask);
						e.parity = (b & cHighBitMask); // converts to bool
						buffA->push_back(e);
					}
#ifdef VERBOSE
					std::cout << events.size();
#endif
					event_callback_(*buffA);
					std::swap(buffA, buffB);
				}
				delete[] buffer;
				delete buff1;
				delete buff2;
			}
			void StopEventCapture() {
				running_ = false;
				thread_.join();
				device_.WriteCommand("E-\n");
			}
		private:
			Device device_;
			EventCallbackType event_callback_;
			size_t buffer_size_;
			bool running_;
			boost::thread thread_;
		};
	}

	/** Wrapper for device implementation */
	struct EventCapture {
		EventCapture() {
		}
		EventCapture(Device device, EventCallbackType f, size_t buffer_size=Impl::EventCapture::cDefaultBufferSize) {
			capture_.reset(new Impl::EventCapture(device, f, buffer_size));
		}
	private:
		boost::shared_ptr<Impl::EventCapture> capture_;
	};

}

inline std::ostream& operator<<(std::ostream& os, const Edvs::Event& e) {
	os << "(" << e.parity << ": " << e.x << ", " << e.y << ")";
	return os;
}

#endif /* EDVS_H_ */
