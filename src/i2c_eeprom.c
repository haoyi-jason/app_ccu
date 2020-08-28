#include "ch.h"
#include "hal.h"

#include "i2c_eeprom.h"
#include "string.h"


#define PAGE_SIZE 16
#define EEP_ADDR        0x50
#define SLA 0x00

static uint8_t txBuf[PAGE_SIZE+2];
static uint8_t rxBuf[PAGE_SIZE];

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE,
};

void eepromInit(void)
{
  i2cInit();
  i2cStart(&I2CD1,&i2ccfg);
}

int eepromRead2(uint16_t addr, uint8_t len, uint8_t *buf)
{
  uint8_t addr_h = (addr >> 8);
  uint8_t addr_l = addr & 0xff;
  static msg_t ii;

  txBuf[0] = addr_l;
  ii = i2cMasterTransmitTimeout(&I2CD1,
                             0xd0 | addr_h,
                             txBuf,
                             1,
                             buf,
                             len,
                             100);

  
}

int eepromRead(uint16_t addr, uint16_t len, uint8_t *buf)
{
  uint16_t bufRead=0;
  uint16_t bufToRead;
  uint16_t address = addr;
  uint8_t addr_h = (addr >> 8);
  uint8_t addr_l = addr & 0xff;
  uint8_t *bufptr = buf;
  static msg_t ii;
  
  i2cAcquireBus(&I2CD1);
  while(bufRead < len-1){
    addr_h = (address >> 8);
    addr_l = address & 0xff;
    
    bufToRead = ((len - bufRead)>PAGE_SIZE)?(PAGE_SIZE):(len-bufRead);
    txBuf[0] = addr_l;
    ii = i2cMasterTransmitTimeout(&I2CD1,
                             EEP_ADDR | addr_h,
                             txBuf,
                             1,
                             rxBuf,
                             bufToRead,
                             MS2ST(10));
    address += bufToRead;
    bufRead += bufToRead;
    memcpy(bufptr,rxBuf,bufToRead);
    bufptr+=bufToRead;
    chThdSleepMilliseconds(10);
  }
  i2cReleaseBus(&I2CD1);
  
}

int eepromWrite(uint16_t addr, uint16_t len, uint8_t *buf)
{
  uint16_t bufWrote = 0;
  uint16_t bufToWrite;
  uint16_t address = addr;
  uint8_t *bufptr = buf;
  uint8_t addr_h = (addr >> 8);
  uint8_t addr_l = addr & 0xff;
  static msg_t ii;
  
  i2cAcquireBus(&I2CD1);
  while(bufWrote < len-1){
    addr_h = (address >> 8);
    addr_l = address & 0xff;
    if((address & (PAGE_SIZE-1))){
      bufToWrite = PAGE_SIZE-(address & (PAGE_SIZE-1));
    }else{
      bufToWrite = PAGE_SIZE;
    }
    txBuf[0] = addr_l;
    memcpy(&txBuf[1],bufptr,bufToWrite);
    ii = i2cMasterTransmitTimeout(&I2CD1,
                             EEP_ADDR | addr_h,
                             txBuf,
                             bufToWrite+1,
                             rxBuf,
                             0,
                             TIME_INFINITE);
    address += bufToWrite;
    bufWrote += bufToWrite;
    bufptr += bufToWrite;
        chThdSleepMilliseconds(10);
  }
  i2cReleaseBus(&I2CD1);
}

void eepWrite32(uint16_t regAddr,uint16_t len, uint8_t *dptr)
{
  uint8_t bufWrote = 0;
  uint8_t bufToWrite;
  uint16_t address = regAddr;
  uint8_t *bufptr = dptr;
  static msg_t ret;
  uint8_t done = 0;
  
  while(done == 0){
    bufToWrite = ((len - bufWrote) > PAGE_SIZE)?(PAGE_SIZE):(len-bufWrote);
    txBuf[0] = (address >> 8);
    txBuf[1] = (address >> 0);
    memcpy(&txBuf[2],bufptr,bufToWrite);
    ret = i2cMasterTransmitTimeout(&I2CD1,
                             EEP_ADDR | SLA,
                             txBuf,
                             bufToWrite+2,
                             rxBuf,
                             0,
                             TIME_INFINITE);
    address += bufToWrite;
    bufWrote += bufToWrite;
    bufptr += bufToWrite;
    if(bufWrote == len) done = 1;
    chThdSleepMilliseconds(5);
  }
  
}

void eepRead32(uint16_t regAddr,uint16_t len, uint8_t *dptr)
{
  uint8_t bufRead=0;
  uint8_t bufToRead;
  uint16_t address = regAddr;
  uint8_t *bufptr = dptr;
  uint8_t done = 0;
  static msg_t ii;
  /*
    txBuf[0] = (address >> 8);
    txBuf[1] = (address >> 0);
    ii = i2cMasterTransmitTimeout(&I2CD1,
                             AT24C64_BASE | SLA,
                             txBuf,
                             2,
                             rxBuf,
                             0,
                             TIME_INFINITE);
  */
  while(done == 0){
    bufToRead = ((len - bufRead)>PAGE_SIZE)?(PAGE_SIZE):(len-bufRead);
    txBuf[0] = (address >> 8);
    txBuf[1] = (address >> 0);
    ii = i2cMasterTransmitTimeout(&I2CD1,
                             EEP_ADDR | SLA,
                             txBuf,
                             2,
                             rxBuf,
                             bufToRead,
                             TIME_INFINITE);
    address += bufToRead;
    bufRead += bufToRead;
    memcpy(bufptr,rxBuf,bufToRead);
    bufptr+=bufToRead;
    if(bufRead == len) done = 1;
    chThdSleepMilliseconds(5);
  }
  
}

void eepTest(void)
{
  int8_t data[64];
  static int8_t ret[64];
  int8_t i;
  uint16_t addr = 0x00;
  
  for(i=0;i<64;i++) data[i] = i*8;
  
  eepWrite32(0x0,64,data);
  eepRead32(0x0,64,ret);
  /*
  data[0] = 0;
  data[1] = 0;
  i = i2cMasterTransmitTimeout(&I2CD1,
                         AT24C64_BASE | SLA,
                         data,
                         32,
                         ret,
                         0,
                         TIME_INFINITE);

  i = i2cMasterTransmitTimeout(&I2CD1,
                         AT24C64_BASE | SLA,
                         data,
                         2,
                         ret,
                         30,
                         TIME_INFINITE);
  */
//  eepromWrite(0xa0,32,data);
  
//  eepromRead(0xa0,32,ret);
  
}
