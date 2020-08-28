#include "ch.h"
#include "hal.h"
#include "sysParam.h"
#include "i2c_eeprom.h"
#include "vmu.h"

module_params_t moduleParam;
const serial_setting_t serial_default = {
  0,
  9600,
  SPAR_NONE,
  SSTOP_1,
  DATA_8
};

const lan_setting_t lan_default = {
  {192,168,0,240,0,0},
  {255,255,255,0,0,0},
  {192,168,0,1,0,0},
  {0x70,0xb1,0xff,0xff,0xff,0xff}
};

const module_setting_t module_default = {
  EEP_HEADING,
  0x01000000,
  0x00000001,
  "Grididea.com",
  "USER"
};

void defaultParams(void)
{
  app_loadDefault();
  memcpy((uint8_t*)&moduleParam.param,(uint8_t*)&module_default,sizeof(module_setting_t));
  memcpy((uint8_t*)&moduleParam.serial,(uint8_t*)&serial_default,sizeof(serial_setting_t));
  memcpy((uint8_t*)&moduleParam.lan,(uint8_t*)&lan_default,sizeof(lan_setting_t)); 
}
void sysSaveParams(void)
{
  eepromWrite(EEP_STORE_OFFSET,sizeof(module_params_t),(uint8_t*)&moduleParam);
}


void sysParamInit()
{
  eepromRead(EEP_STORE_OFFSET,sizeof(module_params_t),(uint8_t*)&moduleParam);
  
  if(moduleParam.param.flag != EEP_HEADING){
    defaultParams();
    sysSaveParams();
  }
}
