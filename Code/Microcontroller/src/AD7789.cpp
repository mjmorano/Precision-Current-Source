/*
  AD7791.cpp - 
*/

#include <Arduino.h>
#include <SPI.h>
#include "AD7789.h"

AD7789class AD7789;

uint32_t AD7789class::readAD7789 (uint8_t ui8address)
{
	uint8_t ui8AdcUpperCodes = 0;			// Data register bits 23-16
	uint8_t ui8AdcMiddleCodes = 0;			// Data register bits 15-8
	uint8_t ui8AdcLowerCodes = 0;			// Data register bits 7-0
	uint32_t ui32AdcCodes = 0;

  SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE3) );

	if (ui8address == DATA_READ)
	{	
		digitalWrite(AD7789_SS,LOW);
		SPI.transfer(ui8address); 
		ui8AdcUpperCodes = SPI.transfer(0x00);			// Data register bits 23-16
		ui8AdcMiddleCodes = SPI.transfer(0x00);			// Data register bits 15-8
		ui8AdcLowerCodes = SPI.transfer(0x00);			// Data register bits 7-0
		digitalWrite(AD7789_SS,HIGH);
		ui32AdcCodes = ((long)ui8AdcUpperCodes << 16) | ((long)ui8AdcMiddleCodes << 8) | ui8AdcLowerCodes;
		
		// Serial.print("ADC Data Register Read : ");  //Debug serial prints
		// Serial.println(ui32AdcCodes,2);			
		// Serial.print("ADC Bits 23-16 : ");		
		// Serial.println(ui8AdcUpperCodes,2);
		// Serial.print("ADC Bits 15-8 : ");  
		// Serial.println(ui8AdcMiddleCodes,2);
		// Serial.print("ADC Bits 7-0 : ");  
		// Serial.println(ui8AdcLowerCodes,2);
	}
	else
	{
		digitalWrite(AD7789_SS, LOW);
		SPI.transfer(ui8address); 
		ui8AdcLowerCodes = SPI.transfer(0x00);			// register read
		digitalWrite(AD7789_SS, HIGH);	
		ui32AdcCodes = ((long)ui8AdcUpperCodes << 16) | ((long)ui8AdcMiddleCodes << 8) | ui8AdcLowerCodes;
		
		// Serial.print("ADC Register: ");		//Debug serial prints
		// Serial.print(ui8address);
		// Serial.print(", contains: ");
		// Serial.println(ui32AdcCodes);
	}

  SPI.endTransaction();
	return ui32AdcCodes;
}

void AD7789class::writeAD7789 (uint8_t ui8address, uint8_t ui8value)
{
	SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE3) );
    	
	if (ui8address != RESET && ui8address != COMM_WRITE && ui8address != COMM_WRITE_VDD)
	{
		digitalWrite(AD7789_SS,LOW);
		SPI.transfer(ui8address); 
		SPI.transfer(ui8value);
		digitalWrite(AD7789_SS,HIGH);
		// Serial.print("Wrote to address: ");	//Debug serial prints
		// Serial.println(ui8address, BIN);		//Debug serial prints
		// Serial.print("Wrote value: ");		//Debug serial prints
		// Serial.println(ui8value, BIN);		//Debug serial prints
	}
	else if (ui8address == COMM_WRITE){
		digitalWrite(AD7789_SS,LOW);
		SPI.transfer(COMM_WRITE); 
		digitalWrite(AD7789_SS,HIGH);		
	}
	else if (ui8address == COMM_WRITE_VDD){
		digitalWrite(AD7789_SS,LOW);
		SPI.transfer(COMM_WRITE_VDD); 
		digitalWrite(AD7789_SS,HIGH);		
	}
	else
	{
		digitalWrite(AD7789_SS,LOW);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		digitalWrite(AD7789_SS,HIGH);
		//Serial.println("Reset Command");			//Debug serial prints
	}

	SPI.endTransaction();
		/*Serial.print("ADC Register ");		//Debug serial prints
		Serial.print(ui8address);
		Serial.println("Written");
		Serial.print("With Register Value ");
		Serial.println(ui8value);*/
}
