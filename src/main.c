#include "ch.h"
#include "hal.h"
#include "stdlib.h"
#include "sysParam.h"
#include "ccu.h"



uint8_t reset_flags = 0;
extern thread_t *thdAD7124;

static THD_WORKING_AREA(waThread1,128);
static msg_t Thread1(void *arg)
{
  (void)arg;
  chRegSetThreadName("Blinker");
  while(TRUE){
    chThdSleepMilliseconds(10);
  }

  return 0;
}


/*
 * Application entry point.
 */
int main(void) {


   // initial chibios 
  halInit();
  chSysInit();
  
  ccuInit();

  while(1){
    chThdSleepMilliseconds(100);
  }
   
}
