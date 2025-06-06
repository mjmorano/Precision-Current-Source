#pragma once

#include <Arduino.h>

/*****************************************************************************/
/************************** ADC Address Definitions **************************/
/*****************************************************************************/

//ADC Write Commands
#define MODE_WRITE			0x10					// Write to the Mode Register
#define COMM_WRITE			0x00					// Write to the Comm Register
#define COMM_WRITE_VDD		0x03					// Write to the Comm Register for Vdd
#define MODE_WRITE_VDD		0x13					// Write to the Mode Register for Vdd

//ADC Read Commands
#define STATUS_READ			0x08					// Read from the Status Register
#define STATUS_READ_VDD		0x0B					// Read from the Status Register for Vdd
#define MODE_READ			0x18					// Read from the Mode Register
#define FILTER_READ			0x28					// Read from the Filter Register
#define DATA_READ			0x38					// Read from the Data Register
#define COMM_READ			0x08					// Read fomr the Comm Register

#define RESET				0xFF					// Resets the chip to default

class AD7789class
{
	public:
		void writeAD7789 (uint8_t ui8address, uint8_t ui8value);
		uint32_t readAD7789 (uint8_t ui8address);
	private:

};

extern AD7789class AD7789;
