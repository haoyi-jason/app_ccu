#ifndef _VMU_H
#define _VMU_H

#include "ch.h"
#include "hal.h"

#define EVT_TEST_SATRT  EVENT_MASK(31)

enum vmu_dig_in_name{
  DI0,
  DI1,
  DI2,
  DI3,
  DI4,
  DI5,
  DI6,
  DI7,
  EXT_WKUP,
  ACC_SENS,  
};

enum vmu_dig_out_name{
  DO0,
  DO1,
  DO2,
  DO3,
  DO4,
  DO5,
  DO6,
  DO7,
  AD1_RST,
  AD2_RST,
  AUX_PWR_EN
};

typedef enum {
  ACC_INACTIVE,
  ACC_ACTIVE
}acc_status_t;

typedef enum{
  AUX_POWER_ON,
  AUX_POWER_OFF
}aux_power_state_t;

typedef struct{
  uint32_t pgn;
  uint8_t buffer[8];
  uint8_t len;
  uint8_t dst;
  uint8_t src;
  uint8_t prio;
}j1939_msg_t;

typedef struct{
  uint8_t dump_adc0;
  uint8_t dump_adc1;
  uint8_t echo_rs485;
  uint8_t echo_canbus;  
}test_state_t;

typedef struct{
  test_state_t testState;
  uint8_t digital_input[8];
  uint8_t digital_output[8];
  struct tm tim_now;
}appParam_t;

extern appParam_t appParam;

void ccuInit(void);

#endif