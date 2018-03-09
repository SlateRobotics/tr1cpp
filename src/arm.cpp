#include <tr1cpp/arm.h>
#include <tr1cpp/joint.h>

namespace tr1cpp
{
	Arm::Arm()
	{
		for (int i = 0; i < 6; i++)
		{
			this->joints[i].setMotorId(i + 1);
		}
	}

	Arm::~Arm()
	{

	}
}
