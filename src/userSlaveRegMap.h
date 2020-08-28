#ifndef _USERSLAVEREGMAP_
#define _USERSLAVEREGMAP_
#include "modbusregmap.h"


int8_t mbread_0_10(uint16_t,uint8_t*);
int8_t mbwrite_0_10(uint16_t,uint8_t*);
int8_t doorCtrl(uint16_t,uint8_t*);

#endif