#include "ch.h"
#include "hal.h"
#include "digital_io.h"
#include "sysParam.h"

static digital_io_type_t *dig_out;
static digital_io_type_t *dig_in;
static uint8_t nof_dig_out;
static uint8_t nof_dig_in;

int8_t dio_read(uint8_t ch)
{
  uint8_t ret;
  if(ch >= nof_dig_in) return -1;
  ret = palReadPad(dig_in[ch].port,dig_in[ch].pad);
  return ret==0?1:0;
}

void dio_write(uint8_t ch, uint8_t val)
{
  if(ch >= nof_dig_out)
    return;
  
  if(val)
    palSetPad(dig_out[ch].port, dig_out[ch].pad);
  else
    palClearPad(dig_out[ch].port, dig_out[ch].pad);
}

void digital_io_init()
{
  uint8_t i;
  
  // do nothing when only gpio supported so far

}

void dio_set_in_def(digital_io_type_t *d, uint8_t nof_pins)
{
  if(d){
    dig_in = d;
    nof_dig_in = nof_pins;
  }
}
void dio_set_out_def(digital_io_type_t *d, uint8_t nof_pins)
{
  if(d){
    dig_out = d;
    nof_dig_out = nof_pins;
  }
}

