#ifndef TR1CPP__JOINT_H
#define TR1CPP__JOINT_H

#include <tr1cpp/i2c.h>

namespace tr1cpp
{
	class Joint
	{
		private:
			uint8_t _motorId = 0;
			uint8_t _armSlave1Address = 0x72;
			uint8_t _armSlave2Address = 0x73;
			uint8_t _getSlaveAddress();
			void _prepareI2C(uint8_t result[4], double effort);
		public:
			Joint();
			Joint(uint8_t motorId);
			~Joint();
			void setMotorId(uint8_t motorId);
			void step(double effort);
	};
}

#endif
