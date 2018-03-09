#include <stdlib.h>
#include <math.h>
#include "ros/ros.h"
#include <tr1cpp/joint.h>
#include <tr1cpp/i2c.h>

namespace tr1cpp
{
	Joint::Joint()
	{
		
	}

	Joint::Joint(uint8_t motorId)
	{
		setMotorId(motorId);
	}

	Joint::~Joint()
	{

	}

	void Joint::setMotorId(uint8_t motorId)
	{
		this->_motorId = motorId;
	}

	void Joint::step(double effort)
	{
		if (abs(effort * 100.0) > 100)
		{
			ROS_ERROR("Joint %i effort magnitude is greater than 1. Must keep values between -1 and 1.", _motorId);
		}

		if (abs(effort * 100.0) > 25)
		{
			uint8_t data[4];
			_prepareI2C(data, effort);
			I2C i2cSlave = I2C(1, _getSlaveAddress());
			uint8_t result = i2cSlave.write_byte(0x00, data);
			ROS_INFO("Result: [%i]; effort: [%f]; bytes: %i, %i, %i, %i", result, effort, data[0], data[1], data[2], data[3]);
		}
	}

	uint8_t Joint::_getSlaveAddress()
	{
		if (_motorId > 0 && _motorId <= 4)
		{
			return _armSlave1Address;
		}
		else if (_motorId > 0 && _motorId <= 7)
		{
			return _armSlave2Address;
		}
		else
		{
			ROS_ERROR("Invalid MotorID: %i", _motorId);
			return -1;
		}
	}

	void Joint::_prepareI2C(uint8_t result[4], double effort)
	{
		uint8_t speed = floor(abs(effort * 100));
		uint8_t direction = (effort > 0);
		uint8_t duration = 10;

		result[0] = _motorId;
		result[1] = speed;
		result[2] = direction;
		result[3] = duration;
	}
}
