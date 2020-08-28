#include "hal.h"
#include "mrxx.h"
#include "peripheral_if.h"

#define MRAM_CAP_4M     524288
#define MRAM_CAP_1M     128096

static MR25XX_dev_t mr25 = {
  MRAM_CAP_4M,      // bytes
  0,
  mrxx_spi_select,
  spiRead,
  spiWrite
};