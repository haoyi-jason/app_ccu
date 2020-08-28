#ifndef _PERIPHERAL_IF_
#define _PERIPHERAL_IF_



int8_t mrxx_spi_select(uint8_t v);
int8_t spiRead(uint8_t *b,uint16_t n);
int8_t spiWrite(uint8_t *b,uint16_t n);


#endif