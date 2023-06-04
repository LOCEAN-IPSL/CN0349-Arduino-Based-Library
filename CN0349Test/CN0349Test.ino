#include <EEPROM.h>
#include "CN0349.h"

//#define EEPROM_SIZE 48

CN0349 CT;  //initialize CT sensor

//For saving calibration gain factors and offsets:
template <class T> int EEPROM_writeAnything(int ee, const T& value) { //saves value into memory
  const uint8_t* p = (const uint8_t*)(const void*)&value;
  uint16_t i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value) {  //retrieves value from memory
  uint8_t* p = (uint8_t*)(void*)&value;
  uint16_t i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}

void calibrateCN0349(char state) { //find, and save gain factors, and offsets. See AD5934 and CN-0349 datasheets for process
  //CN-0349 reference:
  //      Rcal(ohms) | Rfb(ohms)  |MUXChannels
  //RTD:   R3(100)     R9(100)      4,1
  //High1: R3(100)     R9(100)      4,1
  //High2: R4(1000)    R9(100)      5,1
  //Low1:  R4(1000)    R8(1000)     5,2
  //Low2:  R7(10000)   R8(1000)     6,2
  float H1Tmag, H2mag, L1mag, L2mag = 0;
  float YL, YH, NH, NL, GF_low, NOS_low, GF_high, NOS_high = 0;
  //reference:                      //three point calibration
  //Rcal = {R3,R4,R7}={100,1000,10000}
  if (state == 'L') {
    //LOW MODE:
    L1mag =  CT.calibrate(R4, R4);    //Low range 1 //was 1000 1000
    Serial.println(L1mag, 8);
    Serial.println(F("Low Magnitude 1 Found"));
    L2mag =  CT.calibrate(R7, R4);   //Low range 2
    Serial.println(L2mag, 8);
    Serial.println(F("Low Magnitude 2 Found"));

    YL = 1 / R7;   //low admittance
    YH = 1 / R4;   //high admittance
    NL = L2mag;    //Magnitudes
    NH = L1mag;
    GF_low = (YH - YL) / (NH - NL); //interpolation
    NOS_low = NH - (YH) / GF_low;   //calculate offset
    EEPROM_writeAnything(GF_low_addr, GF_low); //save values
    EEPROM_writeAnything(NOS_low_addr, NOS_low);
    EEPROM_writeAnything(calibState_addr, 2);
  }
  //HIGH MODE:
  if (state == 'H') {
    H1Tmag = CT.calibrate(R3, R9);     //High range 1
    Serial.println(H1Tmag, 8);
    Serial.println(F("High Magnitude 1 Found"));
    H2mag = CT.calibrate(R4, R9);     //High range 2
    Serial.println(H2mag, 8);
    Serial.println(F("High Magnitude 2 Found"));

    YL = 1 / R4;   //low admittance
    YH = 1 / R3;     //high admittance
    NL = H2mag;    //Magnitudes
    NH = H1Tmag;

    GF_high = (YH - YL) / (NH - NL);   //interpolation
    NOS_high = NH - YH / GF_high;      //calculate offset
    EEPROM_writeAnything(GF_high_addr, GF_high);  //save values in memory
    EEPROM_writeAnything(NOS_high_addr, NOS_high);
    EEPROM_writeAnything(calibState_addr, 1);
  }
  Serial.println(GF_high, 8);
  Serial.println(NOS_high, 8);

  Serial.println(F("Calibration Done"));
}

float sample() {

  double sum = 0;
  float magnitude;
  uint8_t count = 0;
  uint8_t err = 0;

  do {
    err = CT.sweep_read_data(&magnitude, NULL);

    if (!err) {
      //Serial.printf("magnitude: %f\n", magnitude);
      sum += magnitude;
      count++;
      CT.sweep_step();
    }
  } while(err != 2);

  return (float) (sum/count);
  //Serial.printf("magnitude: %f\n", sum/count);
}

void setup() {
  Serial.begin(115200);

  //float startFreq = 8 * pow(10, 3);
  float startFreq = 1000;
  CT.configureAD5934(15, startFreq, 0, 5);     // number of settling times ,start frequency (Hz),frequency increment (Hz), number of increments
  delay(5);
  //calibrateCN0349('L');


  Serial.printf("0x82: %02x / %02x\n", CT.AD5934byteRead(0x82), CT.frequencyCode(startFreq, 0));
  Serial.printf("0x83: %02x / %02x\n", CT.AD5934byteRead(0x83), CT.frequencyCode(startFreq, 1));
  Serial.printf("0x84: %02x / %02x\n", CT.AD5934byteRead(0x84), CT.frequencyCode(startFreq, 2));

  uint8_t modes[] = {
	  R9|R3,
	  R8|R3,
	  R9|R4,
	  R8|R4,
	  R9|R7,
	  R8|R7,
	  R9|RTD,
	  R8|RTD,
	  R9|YCELL,
	  R8|YCELL,
  };
  
  for(uint8_t i=0; sizeof(modes)/sizeof(uint8_t); i++) {
	  
	  uint8_t mode = modes[i];
	  
	  CT.setSwitches(mode);
	  CT.sweep_init();
	  Serial.printf("mode %d,\tmagnitude: %f\n", mode, sample());
  }
  CT.sweep_close();
  
}


void loop() {
  return;
  float YL, YH, NH, NL, GF_low, NOS_low, GF_high, NOS_high = 0;
  float Y_cell, T_cell, YT_cell, T_imp, imp = -1;
  //Just please make sure calibrateCN0349 is called at least once in the programs lifetime before the following line!
  EEPROM_readAnything(GF_high_addr, GF_high);  //save values //GF_high = same as GF_rtd
  EEPROM_readAnything(NOS_high_addr, NOS_high);
  uint8_t CT_error = CT.measure(GF_high, GF_high, NOS_high, 'H', &T_imp, &imp, &Y_cell, &T_cell);
  Serial.print(F("Admittance:\t\t"));
  Serial.println(Y_cell, 3);
  Serial.print("Temperature : ");
  Serial.println(T_cell, 3);
  
}
