/*
 * Code for the B0 and shim/gradient powIer supply. Uses an AD7789 ADC for current readback
 * and an LTC1595 based circuit for setting the output current. Uses a modified version of
 * the Analog Devices CN0216 circuit from the lab example for reading the ADC.
 * 
 * Author: M. Morano
 * Date: 4/25/2025
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include "AD7789.h"

// Do all the setup stuff
#define STAT 2      // Stats LED pin
#define OC 3        // Overcurrent LED pin
#define OT 4        // Overtemp LED pin
#define LD 9        // LD pin for the LTC1595
#define TSR A0      // Resistor NTC on ADC channel 0
#define TSM A1      // MOSFET NTC on ADC channel 1

const float maxAdcCode = pow(2, 24);        // Max code for 24 bit ADC
const float maxDacCode = pow(2, 16) - 1;    // Max code for 16 bit DAC
float R1 = 10.0f;                           // Current shunt resistor value. Change this if using different value
float vref = 2.5f;                          // ADC reference voltage
uint16_t code = 0;                          // Code for DAC current setpoint
float I = 0.0;                              // Current set value
uint32_t myTime = 0;                        // Used for polling the temperature sensors
uint8_t nAvg = 1;                           // Number of ADC averages
bool adcGood = false;

// Temperature sensor stuff
Adafruit_TMP117  tmp117;
sensors_event_t temp;
bool tmpFound = true;
float* Ta = &temp.temperature;
float Tr = 0.0f;                // NTC1 temperature reading
float Tm = 0.0f;                // NTC2 temperature reading
const float beta = 3892.0f;     // Beta value for NTC
float TaMax = 80.f;             // Maximum temperature for the enclosure before shutdown
float TrMax = 90.f;             // Maximum temperature for the feedback resistor before shutdown
float TmMax = 90.f;             // Maximum temperature for the MOSFET before shutdown

const char term = '\n';         // Terminator for serial interface

float read_ADC(){
  
  uint32_t reading = 0;
  uint32_t ui32status = 0;
  
  for(int i=0; i<nAvg; i++){
    
    ui32status = 0;
    
    do {
      ui32status = AD7789.readAD7789(STATUS_READ);
    } while (ui32status & 0x80);
    
    reading += AD7789.readAD7789(DATA_READ);
  }
  
  return ((float)reading / (float)nAvg) * vref;
}

float read_ADC_VDD(){
  
  uint32_t reading = 0;
  uint32_t ui32status = 0;
    
  do {
    ui32status = AD7789.readAD7789(STATUS_READ_VDD);
  } while (ui32status & 0x80);
    
  reading += AD7789.readAD7789(DATA_READ);
  
  return 1.17f * 5.f * (float)reading / maxAdcCode;
}

void readTemps(){
  int sensorValue = analogRead(TSR);
  float RNTC = 1e3 / (1023.0 / sensorValue - 1);
  Tr = 1.f / ( 1.f / 298.15f + (1.f / beta) * (float)log(RNTC / 1e3f) ) - 273.25f; // Gives the temperature in celsius
  delay(1);
  sensorValue = analogRead(TSM);
  RNTC = 1e3 / (1023.0 / sensorValue - 1);
  Tm = 1.f / ( 1.f / 298.15f + (1.f / beta) * (float)log(RNTC / 1e3f) ) - 273.25f; // Gives the temperature in celsius
}

void writeDAC(uint16_t code){
  SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE0) );
  SPI.transfer16(code);
  SPI.endTransaction();
  digitalWrite(LD, LOW);
  digitalWrite(LD, HIGH);
}

bool AD7789INIT(){
	AD7789.writeAD7789(RESET, 0xFF);		      // Resets the part for initial use
	delay(1000);
  AD7789.writeAD7789(COMM_WRITE_VDD, 0x00);
  AD7789.writeAD7789(MODE_WRITE_VDD, 0x06);	    // Mode register value (continuous conversion, unipolar input)
  float vdd = read_ADC_VDD();
  AD7789.writeAD7789(COMM_WRITE, 0x00);
  if(vdd >= 4.75f && vdd <= 5.25f)
    return true;
  else
    return false;
}

void parseCmd(String inString){

  int8_t index = inString.indexOf('?');

  if (index == -1){
    index = inString.indexOf('=');

    if (index == -1)
      Serial.println("Invalid syntax");

    else {
      char cmd[10];
      char value[10];
      char* end;
      sscanf(inString.c_str(), "%[^=]=%s", cmd, value);

      if (!strcmp(cmd, "code")){
        int32_t inInt = strtol(value, &end, 10);

        if (isControl(*end)){
          if (inInt >= 0 && inInt <= 65535){
            code = (uint16_t)inInt;
            writeDAC(code);
            Serial.println("Ok");

          } else
            Serial.println("Code must be between 0 and 65535");

        } else
          Serial.println("Invalid Code");

      } else if (!strcmp(cmd, "nAvg")){        
        int32_t inInt = strtol(value, &end, 10);       

        if (isControl(*end)){
          if (inInt >= 1 && inInt <= 255){
            nAvg = (uint8_t)inInt;
            Serial.println("Ok");

          } else 
            Serial.println("nAvg must be between 1 and 255");

        } else
            Serial.println("Invalid number");

      } else if (!strcmp(cmd, "curr")){
        float I = (float)strtod(value, &end);

        if (isControl(*end)){          
          if (I >= 0.f && I <= 0.25f){
            code = (uint16_t)(maxDacCode * R1 * I / vref);
            writeDAC(code);
            Serial.println("Ok");

          } else 
            Serial.println("Current must be between 0 and 250mA");

        } else 
          Serial.println("Invalid number");

      } else if (!strcmp(cmd, "TrMax")){        
        float limit = (float)strtod(value, &end);       

        if (isControl(*end)){
          if (limit >= 0.f && limit <= 100.f){
            TrMax = limit;
            Serial.println("Ok");

          } else 
            Serial.println("TrMax must be between 0 and 100 C");

        } else
            Serial.println("Invalid temperature");

      } else if (!strcmp(cmd, "TaMax")){        
        float limit = (float)strtod(value, &end);       

        if (isControl(*end)){
          if (limit >= 0.f && limit <= 100.f){
            TaMax = limit;
            Serial.println("Ok");

          } else 
            Serial.println("TaMax must be between 0 and 100 C");

        } else
            Serial.println("Invalid temperature");

      } else if (!strcmp(cmd, "TmMax")){        
        float limit = (float)strtod(value, &end);       

        if (isControl(*end)){
          if (limit >= 0.f && limit <= 100.f){
            TmMax = limit;
            Serial.println("Ok");

          } else 
            Serial.println("TmMax must be between 0 and 100 C");

        } else
            Serial.println("Invalid temperature");

      } else
        Serial.println("Invalid command");

    }

  } else {

    char cmd[10];
    sscanf(inString.c_str(), "%[^?]?", cmd);

    if (!strcmp(cmd, "max")){      
      Serial.println("250 mA");

    } else if (!strcmp(cmd, "curr")){
      Serial.println(read_ADC(), 7);

    } else if (!strcmp(cmd, "TaMax")){
      Serial.println(TaMax, 2);

    } else if (!strcmp(cmd, "TrMax")){
      Serial.println(TrMax, 2);

    } else if (!strcmp(cmd, "TmMax")){
      Serial.println(TmMax, 2);

    } else if (!strcmp(cmd, "Tr")){
      Serial.println(Tr, 2);

    } else if (!strcmp(cmd, "Tm")){
      Serial.println(Tm, 2);

    } else if (!strcmp(cmd, "Ta")){
      Serial.println(*Ta, 2);

    } else if (!strcmp(cmd, "code")){
      Serial.println(code);

    } else if (!strcmp(cmd, "nAvg")){
      Serial.println(nAvg);

    } else if(!strcmp(cmd, "ID")){
      Serial.println("Precision current source");

    } else if(!strcmp(cmd, "temp")){
      if (tmpFound){
        tmp117.getEvent(&temp);
        Serial.print(*Ta, 2);
        Serial.print(", ");
      }
      Serial.print(Tr, 2);
      Serial.print(", ");
      Serial.println(Tm, 2);

    } else if(!strcmp(cmd, "Vdd")){
      AD7789.writeAD7789(COMM_WRITE_VDD, 0x00);
      AD7789.writeAD7789(MODE_WRITE_VDD, 0x06);	    // Mode register value (continuous conversion, unipolar input)
      float vdd = read_ADC_VDD();
      Serial.println(vdd, 7);
      AD7789.writeAD7789(COMM_WRITE, 0x00);

    } else if(!strcmp(cmd, "adc")){
      adcGood = AD7789INIT();
      if (adcGood)
        Serial.println("Good");
      else
        Serial.println("Bad");

    } else 
      Serial.println("Invalid query");

  }

}

void errorState(){
  while (1)
  {
    digitalWrite(STAT, HIGH);
    delay(250);
    digitalWrite(STAT, LOW);
    delay(250);
  }
}

void setup(){
	
  // open digital communication protocols
  Serial.begin(115200);
  Serial.setTimeout(50);
  SPI.begin();

  // initialize pins
  pinMode(OT, OUTPUT);
  digitalWrite(OT, LOW);
  pinMode(OC, OUTPUT);
  digitalWrite(OC, LOW);
  pinMode(STAT, OUTPUT);
  digitalWrite(STAT, LOW);       
  digitalWrite(SS, HIGH);		  
  pinMode(LD, OUTPUT);                
  digitalWrite(LD, HIGH);
  
  writeDAC(0);

  if (!tmp117.begin()) {
    Serial.println("Failed to find TMP117 chip");
    tmpFound = false;
    temp.temperature = 0.f;
  } else {
    Serial.println("TMP117 Found");
    tmp117.getEvent(&temp);
  }

  // initialize AD7791
  adcGood = AD7789INIT();

  // Read NTC temperature sensors
  readTemps();

  if (adcGood && Tr < TrMax && Tm < TmMax && *Ta < TaMax){
    Serial.println("Ready.");
    digitalWrite(STAT, HIGH);

  } else if (!adcGood){
    Serial.println("Error: Check ADC or power supply.");
    digitalWrite(OT, HIGH);
    digitalWrite(OC, HIGH);
    errorState();

  } else if (Tr >= TrMax){
    Serial.println("Error: R1 overtemp at startup.");
    digitalWrite(OT, HIGH);
    errorState();

  } else if (Tm >= TmMax){
    Serial.println("Error: MOSFET overtemp at startup.");
    digitalWrite(OT, HIGH);
    errorState();

  } else if (*Ta > TaMax){
    Serial.println("Error: Ambient overtemp at startup.");
    digitalWrite(OT, HIGH);
    errorState();

  } else {
    Serial.println("Error: ");
    errorState();

  }

  myTime = millis();

}

void loop(){

  if (millis() >= myTime + 500UL){
    readTemps();
    if (tmpFound)
      tmp117.getEvent(&temp);
    else
      temp.temperature = 0.f;

    if (Tr >= TrMax || Tm >= TmMax || *Ta >= TaMax){
      writeDAC(0);
      digitalWrite(OT, HIGH);
      errorState();
    }
    myTime = millis();

  }

  if (Serial.available()){
    String cmd = Serial.readStringUntil(term);

    if (cmd == "")
      Serial.println("Did you forget the \\n?");
    else
      parseCmd(cmd);
    
  }
}