#include <stdlib.h>
#include <math.h>
#include "ros/ros.h"
#include <tr1cpp/joint.h>
#include <tr1cpp/i2c.h>

namespace tr1cpp
{
	double constrainAngle(double x){
		  x = fmod(x + 180,360);
		  if (x < 0)
		      x += 360;
		  return x - 180;
	}

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

	void Joint::setActuatorType(uint8_t actuatorType)
	{
		this->_actuatorType = actuatorType;
	}

	void Joint::setMotorId(uint8_t motorId)
	{
		this->_motorId = motorId;
	}

	double Joint::readAngle()
	{
		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
		{
			I2C i2cSlave = I2C(1, 0x74);
			uint8_t position = i2cSlave.readByte(_motorId);
			double angle = (position / 128.0 * 360.0) + _angleOffset;
			angle = constrainAngle(angle);
			//ROS_INFO("MotorId: %i, Position: %i, Angle: %f", _motorId, position, angle);
			return angle;
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			ROS_ERROR("Cannot read joint value from servo actuator");
			return 0;
		}
		
	}

	void Joint::actuate(double effort)
	{
		if (abs(effort * 100.0) > 100)
		{
			ROS_ERROR("Joint %i effort magnitude is greater than 1. Must keep values between -1 and 1.", _motorId);
		}

		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
		{
			uint8_t data[4];
			_prepareI2CWrite(data, effort);
			I2C i2cSlave = I2C(1, _getSlaveAddress());
			uint8_t result = i2cSlave.writeData(0x00, data);
			//ROS_INFO("Result: [%i]; effort: [%f]; bytes: %i, %i, %i, %i", result, effort, data[0], data[1], data[2], data[3]);
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			if (effort != _previousEffort)
			{
				uint8_t data[4];
				_prepareI2CWrite(data, effort);
				I2C i2cSlave = I2C(1, _getSlaveAddress());
				uint8_t result = i2cSlave.writeData(0x00, data);
				//ROS_INFO("Result: [%i]; effort: [%f]; bytes: %i, %i, %i, %i", result, effort, data[0], data[1], data[2], data[3]);
			}
		}

		_previousEffort = effort;
	}

	uint8_t Joint::_getSlaveAddress()
	{
		if (_motorId > 0 && _motorId <= 4)
		{
			return _armSlave1Address;
		}
		else if (_motorId > 0 && _motorId <= 8)
		{
			return _armSlave2Address;
		}
		else
		{
			ROS_ERROR("Invalid MotorID: %i", _motorId);
			return -1;
		}
	}

	void Joint::setServoLimits(uint8_t minValue, uint8_t maxValue)
	{
		this->_minServoValue = minValue;
		this->_maxServoValue = maxValue;
	}

	void Joint::_prepareI2CWrite(uint8_t result[4], double effort)
	{
		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
		{
			uint8_t speed = floor(abs(effort * 100));
			uint8_t direction = (effort > 0);
			uint8_t duration = 15;

			result[0] = _motorId;
			result[1] = speed;
			result[2] = direction;
			result[3] = duration;
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			double magnitude = (((effort * -1) + 1.0) / 2) * 100.0;
			uint8_t servoValue = floor(_minServoValue + ((_maxServoValue - _minServoValue) * (magnitude / 100.0)));

			result[0] = _motorId;
			result[1] = servoValue;
			result[2] = 0;
			result[3] = 0;
		}
	}

	void Joint::setAngleOffset(double angleOffset)
	{
		_angleOffset = angleOffset;
	}
	
	int Joint::getActuatorType()
	{
		return _actuatorType;
	}
}
