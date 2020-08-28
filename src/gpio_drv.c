#include "ch.h"
#include "hal.h"
#include "gpio_drv.h"
#include "sysParam.h"

void setPad(struct dio_map_s *p)
{
  palSetPad(p->port,p->pad);
}

void clrPad(struct dio_map_s *p)
{
  palClearPad(p->port,p->pad);
}

uint8_t readPad(struct dio_map_s *p)
{
  return palReadPad(p->port, p->pad)==PAL_HIGH?1:0;
}

uint8_t readLatch(struct dio_map_s *p)
{
  return ((palReadLatch(p->port) >> (p->pad)) && 1U);
}

uint8_t readLatch_inv(struct dio_map_s *p)
{
  return ((palReadLatch(p->port) >> (p->pad)) && 1U)==1?0:1;
}

dio_map_t do_map[]={
  {GPIOB,11,setPad,clrPad,readLatch},
  {GPIOE,15,setPad,clrPad,readLatch},
  {GPIOE,13,setPad,clrPad,readLatch},
  {GPIOE,11,setPad,clrPad,readLatch},
  {GPIOA,7,setPad,clrPad,readLatch},
  {GPIOE,8,setPad,clrPad,readLatch},
  {GPIOB,2,setPad,clrPad,readLatch},
  {GPIOD,7,setPad,clrPad,readLatch},
  {GPIOD,14,clrPad,setPad,readLatch},
  {GPIOD,12,clrPad,setPad,readLatch},
};

dio_map_t di_map[] = {
  {GPIOD,8,setPad,clrPad,readPad},
  {GPIOB,10,setPad,clrPad,readPad},
  {GPIOE,14,setPad,clrPad,readPad},
  {GPIOE,12,setPad,clrPad,readPad},
  {GPIOB,0,setPad,clrPad,readPad},
  {GPIOC,5,setPad,clrPad,readPad},
  {GPIOC,4,setPad,clrPad,readPad},
  {GPIOB,1,setPad,clrPad,readPad},
  {GPIOE,10,setPad,clrPad,readPad},
  {GPIOE,7,setPad,clrPad,readPad},
  {GPIOA,6,setPad,clrPad,readPad},
  {GPIOA,5,setPad,clrPad,readPad},
  {GPIOD,11,setPad,clrPad,readPad},
  {GPIOD,10,setPad,clrPad,readPad},
  {GPIOD,9,setPad,clrPad,readPad},
  {GPIOD,13,setPad,clrPad,readPad},
  {GPIOD,15,setPad,clrPad,readPad},
  {GPIOA,4,setPad,clrPad,readPad},
};

#define NOF_DO_CH       sizeof(do_map)/sizeof(dio_map_t)
#define NOF_DI_CH       sizeof(di_map)/sizeof(dio_map_t)

int32_t digital_output(uint8_t ch, uint8_t val)
{
  if(ch < NOF_DO_CH){
    if(val) do_map[ch].set(&do_map[ch]);
    else do_map[ch].clear(&do_map[ch]);
    return do_map[ch].read(&do_map[ch]);
  }
  return -1;
}

int32_t digital_output_read(uint8_t ch)
{
  if(ch < NOF_DO_CH)
    return do_map[ch].read(&do_map[ch]);
  return -1;
}

int32_t digital_input_read(uint8_t ch)
{
  if(ch < NOF_DI_CH)
    return di_map[ch].read(&di_map[ch]);
  return -1;
}
