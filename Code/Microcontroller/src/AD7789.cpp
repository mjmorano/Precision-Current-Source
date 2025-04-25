/*
  AD7789.cpp - 
*/

#include <Arduino.h>
#include <SPI.h>
#include "AD7789.h"

AD7789class AD7789;

uint32_t AD7789class::readAD7789 (uint8_t ui8address){
	uint8_t ui8AdcUpperCodes = 0;			// Data register bits 23-16
	uint8_t ui8AdcMiddleCodes = 0;			// Data register bits 15-8
	uint8_t ui8AdcLowerCodes = 0;			// Data register bits 7-0
	uint32_t ui32AdcCodes = 0;

  SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE3) );

	if (ui8address == DATA_READ){	
		digitalWrite(SS,LOW);
		SPI.transfer(ui8address); 
		ui8AdcUpperCodes = SPI.transfer(0x00);			// Data register bits 23-16
		ui8AdcMiddleCodes = SPI.transfer(0x00);			// Data register bits 15-8
		ui8AdcLowerCodes = SPI.transfer(0x00);			// Data register bits 7-0
		digitalWrite(SS,HIGH);
		ui32AdcCodes = ((long)ui8AdcUpperCodes << 16) | ((long)ui8AdcMiddleCodes << 8) | ui8AdcLowerCodes;
	}
	else {
		digitalWrite(SS, LOW);
		SPI.transfer(ui8address); 
		ui8AdcLowerCodes = SPI.transfer(0x00);			// register read
		digitalWrite(SS, HIGH);	
		ui32AdcCodes = ((long)ui8AdcUpperCodes << 16) | ((long)ui8AdcMiddleCodes << 8) | ui8AdcLowerCodes;
	}

  	SPI.endTransaction();
	return ui32AdcCodes;

}

void AD7789class::writeAD7789 (uint8_t ui8address, uint8_t ui8value)
{
	SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE3) );
    	
	if (ui8address != RESET && ui8address != COMM_WRITE && ui8address != COMM_WRITE_VDD){
		digitalWrite(SS,LOW);
		SPI.transfer(ui8address); 
		SPI.transfer(ui8value);
		digitalWrite(SS,HIGH);
	}
	else if (ui8address == COMM_WRITE){
		digitalWrite(SS,LOW);
		SPI.transfer(COMM_WRITE); 
		digitalWrite(SS,HIGH);		
	}
	else if (ui8address == COMM_WRITE_VDD){
		digitalWrite(SS,LOW);
		SPI.transfer(COMM_WRITE_VDD); 
		digitalWrite(SS,HIGH);		
	}
	else {
		digitalWrite(SS,LOW);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		digitalWrite(SS,HIGH);
	}

	SPI.endTransaction();

}
