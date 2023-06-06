#include <EEPROM.h>
#include <CN0349.h>

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

float sample() {

  double sum = 0;
  float magnitude;
  uint8_t count = 0;
  uint8_t err = 0;

  do {
    err = CT.sweep_read_data(&magnitude, NULL);

    if ( err != 1 ) {
      sum += magnitude;
      count++;
      CT.sweep_step();
    }
  } while(err != 2);

  return (float) (sum/count);
}

typedef struct {
  char* name;
  uint8_t reg;
} MODE;

#define MODE_INIT(X) { .name = #X, .reg = X }

MODE modes[] = {
  MODE_INIT(R9|R4),
  MODE_INIT(R8|R3),
  MODE_INIT(R9|R4|R3),
  MODE_INIT(R9|R3),
  MODE_INIT(R8|R4),
  MODE_INIT(R9|R8|R3),
//  MODE_INIT(R8|R3),
//  MODE_INIT(R8|R4),
//  MODE_INIT(R9|R7),
//  MODE_INIT(R8|R7),
  MODE_INIT(R9|R6),
  MODE_INIT(R9|RTD),
//  MODE_INIT(R8|RTD),
  MODE_INIT(R9|YCELL),
//  MODE_INIT(R8|YCELL),
  MODE_INIT(R8|YCELL),
};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  CT.AD5934byteWrite(0x81, REG_CONTROL1_RESET); // reset
  _delay_ms(100);
  
  Serial.printf("0x80: %02x\n", CT.AD5934byteRead(0x80));
  Serial.printf("0x81: %02x\n", CT.AD5934byteRead(0x81));

  float startFreq = 1000;
  CT.configureAD5934(50, startFreq, 0, 5);     // number of settling times ,start frequency (Hz),frequency increment (Hz), number of increments
  
  delay(1);
}


void loop() {
  for(uint8_t i=0; i<sizeof(modes)/sizeof(MODE); i++) {
    
    MODE mode = modes[i];
    
    CT.ADG715set(mode.reg);
    CT.sweep_init();
    Serial.printf("mode %s,\tmagnitude: %d\n", mode.name, (uint16_t)sample());
  }
  CT.sweep_close();

  Serial.printf("\n");
  
  delay(1);
  
  return;
}
