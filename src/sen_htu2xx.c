/*
 * sen_htu2xx.c
 *
 *  Created on: 2017¦~8¤ë2¤é
 *      Author: Jason
 */

#include "ch.h"
#include "hal.h"
#include "sen_htu2xx.h"
#include "sysParam.h"

#define SEN_HTU2XX_ADDR (0x40)

#define TRG_TEMP_MEAS   0xe3
#define TRG_HUMI_MEAS   0xe5
#define NO_HOLD_MASTER_BIT 0x10
#define WR_USER_REG     0xe6
#define RD_USER_REG     0xe7
#define SW_RST_REG      0xfe


double sen_htu2xx_read_temp()
{
  double retv;
  uint8_t txBuf[4],rxBuf[8];
  txBuf[0] = TRG_TEMP_MEAS;
  txBuf[1] = 0x81;
  txBuf[2] = 0x81;
  rxBuf[0] = rxBuf[1] = rxBuf[2] = 0x0;
  msg_t ii;

  i2cAcquireBus(&I2CD1);
  ii = i2cMasterTransmitTimeout(&I2CD1,
                                SEN_HTU2XX_ADDR,
                                txBuf,
                                1,
                                rxBuf,
                                3,
                                MS2ST(100));
  i2cReleaseBus(&I2CD1);

  uint16_t s = rxBuf[0]<<8 | rxBuf[1];
  retv = -46.85 + 175.72*s/(1 << 16);

  return retv;
}

double sen_htu2xx_read_humidity()
{
  double retv;
  uint8_t txBuf[4],rxBuf[4];
  txBuf[0] = TRG_HUMI_MEAS;

  msg_t ii;
  i2cAcquireBus(&I2CD1);
  ii = i2cMasterTransmitTimeout(&I2CD1,
                                SEN_HTU2XX_ADDR,
                                txBuf,
                                1,
                                rxBuf,
                                3,
                                MS2ST(100));

  i2cReleaseBus(&I2CD1);
  uint16_t s = rxBuf[0]<<8 | rxBuf[1];
  retv = -6. + 125.*s/(1 << 16);

  return retv;
}
