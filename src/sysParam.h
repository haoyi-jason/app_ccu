#ifndef _SYSPARAM_
#define _SYSPARAM_

#define EEP_HEADING              0xaa
#define EEP_STORE_OFFSET        0x100

// common configuration
typedef enum{
  SPAR_NONE,
  SPAR_EVEN,
  SPAR_ODD,
}com_parity_t;

typedef enum{
  SSTOP_1,
  SSTOP_1_5,
}com_stopbit_t;

typedef enum{
  DATA_7 = 7,
  DATA_8
}com_databit_t;

typedef struct{
  uint8_t baudrate_id;
  uint32_t baudrate_val;
  com_parity_t parity;
  com_stopbit_t stop;
  com_databit_t data;
}serial_setting_t;

typedef struct{
  uint8_t ip[6];
  uint8_t mask[6];
  uint8_t gateway[6];
  uint8_t macaddr[6];
}lan_setting_t;

typedef struct{
  uint8_t flag;
  uint32_t verNum;
  uint32_t serialNum;
  uint8_t vender[32];
  uint8_t user[32];
}module_setting_t;
// end common configuration

typedef struct module_param_s{
  module_setting_t param;
  serial_setting_t serial;
  lan_setting_t lan;
}module_params_t;


extern module_params_t moduleParam;

void sysSaveParams(void);
void sysParamInit(void);

#endif
