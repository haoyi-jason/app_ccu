#ifndef _THERMOCOUPLE_
#define _THERMOCOUPLE_

enum sensor_type_e{
  TC_K,
  TC_J,
  PT_100,
  PT_1000
};

void updateColdT(double t);
void updateHumidity(double t);
double getBoardTemp(void);
double getBoardHumidity(void);
double getTempFromV(uint8_t type, double v);
void thermocoupleInit(void);
void tempUseComp(uint8_t state);
#endif
