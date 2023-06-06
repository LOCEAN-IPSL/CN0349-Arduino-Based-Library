/*
  CN0349.cpp
  MIT License
Copyright (c) 2017 
Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE 
USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Wire.h>// used for I2C communication
#include <Arduino.h>

#ifdef avr
#include "avr/pgmspace.h" 
#include <util/delay.h>
#endif
#if defined(ESP32)
#include <pgmspace.h>
#endif

#include "CN0349.h"
#include <stdlib.h>
#include <inttypes.h>


void CN0349::configureAD5934(uint8_t settlingTimes, float startFreq, float freqIncr, uint8_t numIncr) {
  setNumberOfSettlingTimes(settlingTimes);
  setStartFrequency(startFreq);
  setFrequencyIncrement(freqIncr);
  setNumberOfIncrements(numIncr);
}
///////////////////////////////////////////////////////////////////////////////////////////
/////////AD5934
///////////////////////////////////////////////////////////////////////////////////////////
bool CN0349::AD5934byteWrite(uint8_t address, uint8_t data) {
  Wire.beginTransmission(AD5934_ADDR);
  Wire.write(address); // address specifier
  Wire.write(data); // value specifier
  uint8_t i2cStatus = Wire.endTransmission();
  _delay_ms(1);
  if (i2cStatus)
    return false;
  else
    return true;
}

uint16_t CN0349::AD5934byteRead(uint8_t address) {
  uint8_t rxByte;
  Wire.beginTransmission(AD5934_ADDR);
  Wire.write(address); // address specifier
  int i2cStatus = Wire.endTransmission();
  _delay_ms(1);
  Wire.requestFrom(AD5934_ADDR, 1);
  if (1 <= Wire.available()) {
    rxByte = Wire.read();
  }
  else {
    rxByte = -1;
  }
  return rxByte;
}

uint16_t CN0349::checkStatus() {
  return (AD5934byteRead(STATUS_REGISTER[0]) & 7);
}

// start frequency and frequency increment formula:
uint8_t CN0349::frequencyCode(float freqInHz, uint8_t byteNum) {
  long value = long((freqInHz / (CLOCK_SPEED / 16)) * pow(2, 27));
  uint8_t code[3];
  code[0] = (value & 0xFF0000) >> 0x10;
  code[1] = (value & 0x00FF00) >> 0x08;
  code[2] = (value & 0x0000FF);
  return code[byteNum];
}

bool CN0349::setStartFrequency(float freqInHz) {
  bool statusValue;
  for (uint8_t n = 0; n < 3; n++) {
    statusValue = AD5934byteWrite(START_FREQUENCY_REGISTER[n], frequencyCode(freqInHz, n));
  }
  return statusValue;
}

bool CN0349::setFrequencyIncrement(float freqInHz) {
  bool statusValue;
  for (uint8_t n = 0; n < 3; n++) {
    statusValue = AD5934byteWrite(FREQ_INCREMENT_REGISTER[n], frequencyCode(freqInHz, n));
  }
  return statusValue;
}

bool CN0349::setNumberOfIncrements(uint8_t n) {
  bool i2cStatus;
  uint8_t numIncrements = (n)<(511)?(n):(511);
  i2cStatus = AD5934byteWrite(NUM_INCREMENTS_REGISTER[0], numIncrements >> 8);
  i2cStatus = AD5934byteWrite(NUM_INCREMENTS_REGISTER[1], numIncrements & 255);
  return i2cStatus;
}

bool CN0349::setNumberOfSettlingTimes(uint8_t n) {
  int decode;
  int numSettlingTimes = (n)<(2044)?(n):(2044);
  if (n > 1023) { // put into 9 bit
    decode = 3; // times 4
    numSettlingTimes /= 4;
  }
  else if (n > 511) { // put into 9 bit!
    decode = 1; //times 2
    numSettlingTimes /= 2;
  }
  else {  // within 9 bit range
    decode = 0; //default
    numSettlingTimes = n;
  }
  bool i2cStatus;
   //get MSB add decode value with a 0 at the end to include the MSB
  // ignore first 8 bits
  i2cStatus = AD5934byteWrite(NUM_SETTLING_CYCLES_REGISTER[0], (numSettlingTimes >> 8) + (decode << 1));
  //put 8 bit number in the second address.
  i2cStatus = AD5934byteWrite(NUM_SETTLING_CYCLES_REGISTER[1], numSettlingTimes & 255);
  return i2cStatus;
}

bool CN0349::setControlRegister(uint8_t code) {
  uint8_t rxByte = AD5934byteRead(CONTROL_REGISTER[0]);
  rxByte &= 0x0F; // clear upper four bits
  rxByte |= code << 4; // set to 1011
  bool s = AD5934byteWrite(CONTROL_REGISTER[0], rxByte);
  _delay_ms(1);
  return s;
}

bool CN0349::setControlRegister2() { //initalize D11 D10 D9 D8 @0x80 Excitation Voltage 2.0Vp-p, Internal PGA=1
  uint8_t rxByte = AD5934byteRead(CONTROL_REGISTER[0]);
  rxByte &= 0xF0; // clear lower four bits (11110000)
  rxByte |= REG_CONTROL0 | REG_CONTROL0_1V | REG_CONTROL0_PGA_X1;
  bool s = AD5934byteWrite(CONTROL_REGISTER[0], rxByte);
  _delay_ms(10);
  return s;
}

void CN0349::sweep_init() {
	//setControlRegister2();
	
	/*
	ADG715reset();            //clear out switches
	ADG715writeChannel(switch1, 1); //turn on switchs
	ADG715writeChannel(switch2, 1);
	*/

	setControlRegister2(); //0.Inizialize bit D11,D10,D9,D8
	setControlRegister(STANDBY); //1. place AD5934 in standby mode
	setControlRegister(INITIALIZE); //2. initialize with start frequency
	_delay_ms(100);
	
	setControlRegister(START_SWEEP);
}



uint8_t CN0349::sweep_read_data(float* magnitude, float* phase) {
	uint16_t status = checkStatus();
	int16_t real = 0;
	int16_t imag = 0;
	
	
	if (status & STATUS_REGISTER_VALIDDATA) {
		
		real = AD5934byteRead(REAL_DATA_REGISTER[0]) << 8;
		real |= AD5934byteRead(REAL_DATA_REGISTER[1]);

		imag = AD5934byteRead(IMAG_DATA_REGISTER[0]) << 8;
		imag |= AD5934byteRead(IMAG_DATA_REGISTER[1]);

		if(magnitude)
			*magnitude = sqrt(pow(double(real), 2) + pow(double(imag), 2));
		
		if(phase)
			*phase = atan(double(imag) / double(real)); // if you ever need it
		
		return (status & STATUS_REGISTER_SWEEPDONE) ? 2 : 0;
	}
	
	return (status & STATUS_REGISTER_SWEEPDONE) ? 2 : 1;
}

void CN0349::sweep_step() {
	setControlRegister(INCREMENT);
}

void CN0349::sweep_close() {
	setControlRegister(POWER_DOWN);
}

///////////////////////////////////////////////////////////////////////////////////////////
/////////ADG715
///////////////////////////////////////////////////////////////////////////////////////////
void CN0349::ADG715set(uint8_t reg) {
	Wire.beginTransmission(ADG715_ADDR);
	Wire.write(reg);
	Wire.endTransmission();
}

void CN0349::ADG715reset() { //clear out register
	ADG715set(0x0);
};
