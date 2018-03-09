#include <tr1cpp/tr1.h>
#include <tr1cpp/arm.h>

namespace tr1cpp
{
	TR1::TR1()
	{
		this->armLeft = Arm();
		this->armRight = Arm();
	}

	TR1::~TR1()
	{

	}
}
