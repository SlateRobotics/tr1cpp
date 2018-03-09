#ifndef I2C_H_
#define I2C_H_
#include <inttypes.h>

#define BUFFER_SIZE 0x04  //1 byte buffer

namespace tr1cpp
{
	class I2C
	{
		public:
			I2C(int, int);
			virtual ~I2C();
			uint8_t dataBuffer[BUFFER_SIZE];
			uint8_t read_byte(uint8_t);
			uint8_t write_byte(uint8_t registerNumber, uint8_t data[4]);
		private:
			int _i2caddr;
			int _i2cbus;
			void openfd();
			char busfile[64];
			int fd;
	};
}

#endif /* I2C_H_ */
