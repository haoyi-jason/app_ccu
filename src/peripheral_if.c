#include "ch.h"
#include "hal.h"
#include "peripheral_if.h"

int8_t mrxx_spi_select(uint8_t v)
{
  if(v)
    palSetPad(GPIOB,12);
  else
    palClearPad(GPIOB,12);
  return 0;
}

int8_t spiRead(uint8_t *b,uint16_t n)
{
  spiReceive(&SPID2,n,b);
  return 0;
}

int8_t spiWrite(uint8_t *b,uint16_t n)
{
  spiSend(&SPID2,n,b);
  return 0;
}