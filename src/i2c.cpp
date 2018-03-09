#include "ros/ros.h"
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <unistd.h>
//#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <syslog.h>		/* Syslog functionality */
#include <tr1cpp/i2c.h>

namespace tr1cpp
{
	I2C::I2C(int bus, int address) {
		_i2cbus = bus;
		_i2caddr = address;
		snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
		openfd();
	}

	I2C::~I2C() {
		close(fd);
	}
	//! Read a single byte from I2C Bus
	/*!
	 \param address register address to read from
	 */
	uint8_t I2C::read_byte(uint8_t address) {
		if (fd != -1) {
			uint8_t buff[BUFFER_SIZE];
			buff[0] = address;
			if (write(fd, buff, BUFFER_SIZE) != BUFFER_SIZE) {
				ROS_ERROR("I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]", _i2caddr, address, errno);
				return (-1);
			} else {
				if (read(fd, dataBuffer, BUFFER_SIZE) != BUFFER_SIZE) {
					ROS_ERROR("Could not read from I2C slave 0x%x, register 0x%x [read_byte():read %d]", _i2caddr, address, errno);
					return (-1);
				} else {
					return dataBuffer[0];
				}
			}
		} else {
			ROS_ERROR("Device File not available. Aborting read");
			return (-1);
		}

	}
	//! Write a single byte from a I2C Device
	/*!
	 \param address register address to write to
	 \param data 8 bit data to write
	 */
	uint8_t I2C::write_byte(uint8_t registerNumber, uint8_t data[4]) {
		if (fd != -1) {
			uint8_t buff[5];
			buff[0] = registerNumber;
			buff[1] = data[0];
			buff[2] = data[1];
			buff[3] = data[2];
			buff[4] = data[3];

			int result = write(fd, buff, sizeof(buff));
			if (result != sizeof(buff)) {
				ROS_ERROR("%s. Failed to write to I2C Slave 0x%x @ register 0x%x [write_byte():write %d]", strerror(errno), _i2caddr, registerNumber, errno);
				return (-1);
			} else {
				//ROS_INFO("Wrote to I2C Slave 0x%x @ register 0x%x", _i2caddr, address);
				return (-1);
			}
		} else {
			ROS_ERROR("Device File not available. Aborting write");
			return (-1);
		}
		return 0;
	}
	//! Open device file for I2C Device
	void I2C::openfd() {
		if ((fd = open(busfile, O_RDWR)) < 0) {
			ROS_ERROR("Couldn't open I2C Bus %d [openfd():open %d]", _i2cbus, errno);
		}
		if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0) {
			ROS_ERROR("I2C slave %d failed [openfd():ioctl %d]", _i2caddr, errno);
		}
	}
}
