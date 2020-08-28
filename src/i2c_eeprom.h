#ifndef _I2C_EEPROM_H
#define _I2C_EEPROM_H

#include "ch.h"
#include "hal.h"

void eepromInit(void);
int eepromRead(uint16_t addr, uint16_t len, uint8_t *buf);
int eepromWrite(uint16_t addr, uint16_t len, uint8_t *buf);
void eepTest(void);
void eepRead32(uint16_t regAddr,uint16_t len, uint8_t *dptr);
void eepWrite32(uint16_t regAddr,uint16_t len, uint8_t *dptr);



#endif