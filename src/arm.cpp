#include "ros/ros.h"
#include <tr1cpp/arm.h>
#include <tr1cpp/joint.h>

namespace tr1cpp
{
	Arm::Arm()
	{
		for (int i = 0; i < 8; i++)
		{
			this->joints[i].setMotorId(i + 1);
		}

		this->joints[6].setActuatorType(ACTUATOR_TYPE_SERVO);
		this->joints[7].setActuatorType(ACTUATOR_TYPE_SERVO);

		this->joints[6].setServoLimits(0, 180);
		this->joints[7].setServoLimits(0, 75);

		this->joints[0].setAngleOffset(-213.75);
		this->joints[1].setAngleOffset(-87.1875);
		this->joints[2].setAngleOffset(-202.5);
		this->joints[3].setAngleOffset(-357.1875);
		this->joints[4].setAngleOffset(-75.9375);
		this->joints[5].setAngleOffset(-151.875);
	}

	Arm::~Arm()
	{

	}
}
