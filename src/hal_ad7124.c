/**
 * @file    hal_ad7124.c
 * @brief   ADI AD7124-8 function code for ChibiOS
 *
 * @addtogroup  adda
 * @details
 *
 *
 * @note
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "hal_ad7124.h"
#include "sysParam.h"
#include "thermocouple.h"


//#define REG_READBACK

static SPIDriver *spiDev;
thread_t *thdAD7124;
thread_reference_t ad7124_trp = NULL;
static uint8_t nofChannel;
static uint8_t chCount;
static uint8_t chrrentCh;
static uint16_t vBiasEn = 0x0;
#define fVolts0 (2.5/268435456)

#define NOF_EN_CHANNELS 8
#define NOF_SAMPLE_NUMBER   8
#define NOF_ANACHANNEL      9

#define SPI_SELECT() palClearPad(GPIOD,6)
#define SPI_DESELECT() palSetPad(GPIOD,6);
//#define SPI_SELECT() palClearPad(GPIOB,9)
//#define SPI_DESELECT() palSetPad(GPIOB,9);

static const SPIConfig hs_spicfg = {
  NULL,
  NULL,//GPIOB,
  NULL,//GPIOB_SPI1_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0  | SPI_CR1_CPOL | SPI_CR1_CPHA
//  SPI_CR1_BR_2 |SPI_CR1_BR_1  | SPI_CR1_CPOL  
};
static const SPIConfig ls_spicfg = {
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_2 | SPI_CR1_BR_1
};

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL}, 
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOB, ad7124RdyHandler},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
  }
};

/**
 *  @brief      Read register from AD7124
 *  @details    This function read registers from AD7124 vis SPI bus
 *
 *  @param[in]  regAddr     the address of the register to read
 *  @param[in]  reg         the pointer to the data to written
 *  @param[in]  len         the length of the data
 *  @return     always 1
 */

int ad7124RegRead(unsigned char regAddr, unsigned char *reg, unsigned char len)
{
  //unsigned char ucTx[]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  //unsigned char ucRx[]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  //unsigned char i;
  
  spiAcquireBus(spiDev);
  SPI_SELECT();
  uint8_t r = regAddr | REG_READ_MASK;
  //ucTx[0] = REG_READ_MASK | regAddr;
  spiSend(spiDev,1,&r);
  spiReceive(spiDev,len,reg);
  //spiExchange(spiDev,len+1,ucTx,ucRx);
  SPI_DESELECT();
  spiReleaseBus(spiDev);
//  memcpy(reg,&ucRx[1],len);

  return 1;
}

/**
 * @brief       Write register to AD7124
 * @details     This function write registers to AD7124 via SPI bus
 *
 * @param[in]   regAddr     the address of the register
 * @param[out]  reg         the pointer to the data
 * @param[in]   len         the length in bytes to read
 * @return      always 1
 */

int ad7124RegWrite(unsigned char regAddr, unsigned char *reg, unsigned char len)
{
  unsigned char ucTx[16]={0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
  unsigned char ucRx[8];
  unsigned char i;
  
  spiAcquireBus(spiDev);
  SPI_SELECT();
  ucTx[0] = regAddr;
  memcpy(&ucTx[1],reg,len);
  spiSend(spiDev,1,&regAddr);
  spiSend(spiDev,len,reg);
//  spiExchange(spiDev,len+1,ucTx,ucRx);
  SPI_DESELECT();
  spiReleaseBus(spiDev);
  
#ifdef REG_READBACK
  ad7124RegRead(regAddr,reg,len);
#endif
  
  return 1;
}

int ad7124SetVoltageBias(uint8_t pin)
{
  if(pin < 16){
    vBiasEn |= (1 << pin);
  }
  uint8_t tx[2];
  tx[0] = (vBiasEn >> 8);
  tx[1] = vBiasEn;
  ad7124RegWrite(REG_VBIAS_ADDR,tx,2);
  return 0;
}



int ad7124ClearVoltageBias(uint8_t channel)
{
  if(channel < 16){
    vBiasEn &= ~(1 << channel);
  }
  uint8_t tx[2];
  //ad7124RegRead(REG_VBIAS_ADDR,tx,2);

  tx[0] = vBiasEn >> 8;
  tx[1] = vBiasEn;
  ad7124RegWrite(REG_VBIAS_ADDR,tx,2);
  return 0;
}

int ad7124ClearVBias()
{
  uint8_t tx[2] = {0,0};
  ad7124RegWrite(REG_VBIAS_ADDR,tx,2);
  return 0;
}

/**
 * @brief       Setup the voltage bias to certain pin
 * @details     This function setup the bias voltage to present to pin
 * @param[in] channel   The pin to enable bias voltage
 */


int a7124EnableVBias(uint8_t channel)
{
  uint16_t biasEn = (1 << channel);
  uint8_t tx[2];
  tx[0] = vBiasEn >> 8;
  tx[1] = vBiasEn;
  ad7124RegWrite(REG_VBIAS_ADDR,tx,2);
  return 0;
}

/**
 * @brief       Clear Iout on any channel
 * @return  none
 */

int ad7124ClearIout(void)
{
  uint8_t tx[3] = {0,0,0};
  // clear output channel
  ad7124RegWrite(0x3,tx,3);
  return 0;
}

/**
 * @brief       Set current output on one channel for RTD application
 * @note        The minimum REF is 1V.
 * @param[in] ch0   The pin number for IEXEC0
 * @param[in] ch1   The pin number for IEXEC1
 * @param[in] cur0  The excitation current setting of IEXEC0
 * @param[in] cur1  The excitation current setting of IEXEC1
 */

int ad7124SetIout0(uint8_t ch0, uint8_t ch1, uint8_t cur0, uint8_t cur1)
{
  uint8_t tx[3] = {0,0,0};
  uint8_t rx[3];


  // clear output channel
  ad7124ClearIout();
  ad7124ClearVBias();

  if(cur0 < 0x8){
    tx[1] = IO_IOUT0_RANG(cur0);
    tx[2] = IO_IOUT0_CH(ch0);
  }
  
  if(cur1 < 0x8){
    tx[1] |= IO_IOUT1_RANG(cur1);
    tx[2] |= IO_IOUT1_CH(ch1);
  }
  
  ad7124RegWrite(0x3,tx,3);
  ad7124RegRead(0x3,rx,3);
  return 0;
}

/**
 * @brief       Setup channel register according to anaInData[]
 * @details     This function sets up the channel  register (0x09 ~ 0x18 map to ch0~ch15)
 * @param[in] ch    The channel to configure
 * @return none
 */

void ad7124ChannelSetup(uint8_t ch)
{
  uint8_t reg[2] = {0,0};
  uint8_t inp, inn;
  uint8_t inType;
  
  inp = anaInData[ch].settings->inp;
  inn = anaInData[ch].settings->inn;
  inType = anaInData[ch].settings->inputType >> 8;

  // todo: fix below code for general application
  if(ch == 8){
    inp = 0x10;
    inn = 0x11;
    inType = 4;
  }
  
  reg[0] |= CHANNEL_SETUP(inType);
  reg[0] |= (inp & 0x1f) >> 3;
  reg[1] = (inp & 0x1f) << 5 | inn;
  
  ad7124RegWrite(ADC_REG_CHANNEL(ch),reg,2);
  ad7124RegRead(ADC_REG_CHANNEL(ch),reg,2);
}

/**
 * @brief       Setup the configuration register of AD7124
 * @detail      This function configuration by analog_input_type_t defined by user
 * @param[in] cfg   The configuration index
 */

void ad7124InputConfig(uint8_t cfg)
{
  uint8_t reg[2] = {0,0};
  sysGetSetParam(cfg, reg);
  ad7124RegWrite(ADC_REG_CONFIG(cfg),reg,2);  
}

/**
 * @brief       Setup the filter register of AD7124
 * @detail      This function configuration by analog_filter_type_t defined by user
 * @param[in] cfg   The configuration index
 */
void ad7124FilterConfig(uint8_t flt)
{
  uint8_t reg[3] = {0,0,0};
  sysGetFilterParam(flt, reg);
  ad7124RegWrite(ADC_REG_FILTER(flt),reg,3);  
  //ad7124RegRead(ADC_REG_FILTER(flt),reg,3);
}

int halAd7124StartContConv()
{
  uint8_t reg[2];
  reg[0] = ADC_CTRL_DOUTRDY(1) | ADC_CTRL_CONT(1) | ADC_CTRL_DSTA(1) | ADC_CTRL_CSBEN(1) | ADC_CTRL_REFEN(1);
  reg[1] = ADC_CTRL_POWER(ADC_LOW_POWER) | ADC_CTRL_MODE(ADC_MODE_CONT) | ADC_CTRL_CLKSEL(ADC_CLK_INT_OUT_DIS);
  ad7124RegWrite(ADC_REG_CTRL,reg,2);

  return 0;
}

int halAd7124SetOpMode(uint8_t mode)
{
  uint8_t reg[2];
  // make sure spi bus not activated
  SPI_DESELECT();
  if((mode == ADC_MODE_SINGLE) || (mode == ADC_MODE_CONT))
    reg[0] = ADC_CTRL_DOUTRDY(1) | ADC_CTRL_CONT(1) | ADC_CTRL_DSTA(1) | ADC_CTRL_CSBEN(1) | ADC_CTRL_REFEN(1);
  else
    reg[0] = ADC_CTRL_DOUTRDY(1) | ADC_CTRL_CONT(0) | ADC_CTRL_DSTA(1) | ADC_CTRL_CSBEN(1) | ADC_CTRL_REFEN(1);
  reg[1] = ADC_CTRL_POWER(ADC_FULL_POWER) | ADC_CTRL_MODE(mode) | ADC_CTRL_CLKSEL(ADC_CLK_INT_OUT_DIS);
  ad7124RegWrite(ADC_REG_CTRL,reg,2);
  ad7124RegRead(ADC_REG_CTRL,reg,2);
}

int halAd7124StartContConversion(void)
{
  uint8_t reg[2]; 
  // make sure spi bus not activated
  SPI_DESELECT();
  ad7124RegRead(ADC_REG_CTRL,reg,2);

  reg[0] = ADC_CTRL_DOUTRDY(1) | ADC_CTRL_CONT(1) | ADC_CTRL_DSTA(1) | ADC_CTRL_CSBEN(1) | ADC_CTRL_REFEN(1);
  reg[1] = ADC_CTRL_POWER(ADC_FULL_POWER) | ADC_CTRL_MODE(ADC_MODE_CONT) | ADC_CTRL_CLKSEL(ADC_CLK_INT_OUT_DIS);

  uint8_t rega = ADC_REG_CTRL;
  spiAcquireBus(spiDev);
  SPI_SELECT();
  spiSend(spiDev,1,&rega);
  spiSend(spiDev,2,reg);
  spiReleaseBus(spiDev);
}

int halAd7124StartSingleConv(void)
{
  uint8_t reg[3];
  reg[0] = ADC_REG_CTRL;
  reg[1] = ADC_CTRL_DOUTRDY(1) | ADC_CTRL_CONT(1) | ADC_CTRL_DSTA(1) | ADC_CTRL_CSBEN(1) | ADC_CTRL_REFEN(1);
  reg[2] = ADC_CTRL_POWER(ADC_LOW_POWER) | ADC_CTRL_MODE(ADC_MODE_SINGLE) | ADC_CTRL_CLKSEL(ADC_CLK_INT_OUT_DIS);
  
  SPI_SELECT();
  
  return 1;
}

int halAd7124StartConversionSingle(void)
{
  uint8_t tx[8];
  uint8_t rx[8];

  
  chSysLock();
  spiAcquireBus(spiDev);
  SPI_SELECT();
  spiSend(spiDev,3,tx);
  spiReleaseBus(spiDev);
  chSysUnlock();

  return 1;
}

int halAd7124StopConversion()
{
  uint8_t reg[2];
  reg[0] = 0;
  reg[1] = ADC_CTRL_POWER(ADC_LOW_POWER) | ADC_CTRL_MODE(ADC_MODE_STBY) | ADC_CTRL_CLKSEL(ADC_CLK_INT_OUT_DIS);
  ad7124RegWrite(ADC_REG_CTRL, reg,2);
  ad7124RegRead(ADC_REG_CTRL, reg,2);
  return 1;  
}

/**
 * @brief       Perform dummy read
 */

void halAd7124DummyRead()
{
  unsigned char tx[8],rx[8];
  tx[0] = 0x42;
  spiAcquireBus(spiDev);
  spiExchange(spiDev,5,tx,rx);
  spiReleaseBus(spiDev);
}

long halAd7124DataRead()
{
  unsigned char tx[8],rx[8];

  spiAcquireBus(spiDev);
  spiExchange(spiDev,4,tx,rx);
  spiReleaseBus(spiDev);
  long v = 0;
  v = (rx[0] << 16) | (rx[1] << 8) | (rx[2]);
  return v;
}



// todo: implement ext. interrupt for drdy signal
// & thread to read data

void ad7124RdyHandler(EXTDriver *extp, expchannel_t channel)
{

  chSysLockFromISR();
  extChannelDisableI(&EXTD1,4);
  chEvtSignalI(thdAD7124,EV_ADC_DATA_READY);
  chSysUnlockFromISR();
}

void ad7124EnableChannel(uint8_t ch, uint8_t en)
{
  uint8_t reg[2] = {0,0};
  ad7124RegRead(ADC_REG_CHANNEL(ch),reg,2);

  if(en){
    reg[0] |= 0x80;
  }else{
    reg[0] &= 0x7f;
  }

  ad7124RegWrite(ADC_REG_CHANNEL(ch),reg,2);
  ad7124RegRead(ADC_REG_CHANNEL(ch),reg,2);
}

void ad7124_change_input(uint8_t ch)
{
  uint8_t reg[2] = {0,0};
  uint16_t inputType;
  uint8_t inn,inp;
  
  if(ch == 0xff){
    inp = 0x10;
    inn = 0x11;
    reg[0] |= CHANNEL_SETUP(CFG_VOLTAGE);
    reg[0] |= (inp & 0x1f) >> 3;
    reg[1] = (inp & 0x1f) << 5 | inn;
  }
  else{
    inp = anaInData[ch].settings->inp;
    inn = anaInData[ch].settings->inn;
    inputType = anaInData[ch].settings->inputType;  
    if((inputType >=AIO_TCK) && (inputType <= AIO_TCD)){
      reg[0] |= CHANNEL_SETUP(CFG_THERMOCOUPLE);
    }
    else if((inputType >=AIO_PT100) && (inputType <= AIO_PT1000)){
      reg[0] |= CHANNEL_SETUP(CFG_RTD);
    }
    else if((inputType >=AIO_MILLIAMP) && (inputType <= AIO_VOLT)){
      reg[0] |= CHANNEL_SETUP(CFG_VOLTAGE);    
    }
    else if(inputType == AIO_MILLIOVOLT){
      reg[0] |= CHANNEL_SETUP(CFG_VOLTAGE);    
    }
    reg[0] |= (inp & 0x1f) >> 3;
    reg[1] = (inp & 0x1f) << 5 | inn;
    
  }  
  reg[0] |= 0x80;
  ad7124RegWrite(0x9,reg,2);
  ad7124RegRead(0x9,reg,2);
}

int halAD7124Reset(void)
{
  uint8_t reg[] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
  ad7124RegWrite(0xff,reg,8);
  return 0;
}
uint8_t halAD7124ReadStatus(void)
{
  uint8_t sta;
  ad7124RegRead(0x0,&sta,1);
  return sta;
}
uint8_t halAD7124ReadMode(void)
{
  uint8_t sta;
  ad7124RegRead(0x1,&sta,1);
  return sta;
}

void hal_ad7124_init_config(void)
{
  uint8_t i;
  halAD7124Reset();
  chThdSleepMilliseconds(100);

  // channel setup
//  for(i=0;i<NOF_ANACHANNEL;i++){
//    ad7124ChannelSetup(i);
//    chThdSleepMilliseconds(1);
//  }
  
  // config setup
  for(i=0;i<8;i++){
    ad7124InputConfig(i);
    chThdSleepMilliseconds(1);
  }
  // filter setuup
  for(i=0;i<8;i++){
    ad7124FilterConfig(i);
    chThdSleepMilliseconds(1);
  }
}
  
virtual_timer_t vt;

static void timeout_cb(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(thdAD7124,EV_ADC_RESTART);
  chVTSetI(&vt,MS2ST(1000),timeout_cb,NULL);
  chSysUnlockFromISR();
}

typedef enum{
  CALI_IDLE,
  CALI_INT_OFFSET,
  CALI_INT_GAIN
}cali_state_t;

static THD_WORKING_AREA(waAD7124,1024);
static THD_FUNCTION(thAD7124 ,p)
{
  uint8_t i;
  static uint8_t nofSamplesAcquired;
  uint8_t nofSamplesIgnored;
  long adcResult;
  long adcHistory[8];
  static uint8_t currentChannel;
  uint8_t nextChannel;
  bool bContinueRead = false;
  bool bSampling = false;
  uint8_t releaseCounter = 0;
  int currChannel;
  chRegSetThreadName("AD7124_Thread");
  static float boardTemp;
  cali_state_t caliState = CALI_IDLE;

//  EXTChannelConfig *oldcp;
  EXTConfig *cfg;
  spiDev = (SPIDriver*)p;
  spiStart(spiDev,&hs_spicfg);
  SPI_DESELECT();
  //spiUnselect(spiDev);

  static uint8_t tx[8],rx[8];
  //chThdSleepMilliseconds(10);
  //halAd7124StopConversion();
  hal_ad7124_init_config();
  
  nofChannel = NOF_ANACHANNEL;

  chThdSleepMilliseconds(100);
  //SPI_SELECT();

  currentChannel = 0;

  extStart(&EXTD1,&extcfg);
  
  /* Resumes the caller and goes to the final priority.*/
  chThdResume(&ad7124_trp, MSG_OK);
  
  chVTObjectInit(&vt);
  chVTSet(&vt,MS2ST(1000),timeout_cb,NULL);

  bool done = false;
  while(!done){
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,MS2ST(5));
    switch(caliState){
    case CALI_IDLE:
      ad7124EnableChannel(0,1);
      halAd7124SetOpMode(ADC_MODE_ZERO_CALI);
      palSetGroupMode(GPIOB,PAL_PORT_BIT(4),0,PAL_STM32_ALTERNATE(0));
      extChannelEnable(&EXTD1,4);
      SPI_SELECT();
      caliState = CALI_INT_OFFSET;
      break;
    case CALI_INT_OFFSET:
      if(evt & EV_ADC_DATA_READY){
        // disable interrupt function
        palSetGroupMode(GPIOB,PAL_PORT_BIT(4),0,PAL_STM32_ALTERNATE(5) | PAL_STM32_MODE_ALTERNATE);
        SPI_DESELECT();
        ad7124RegRead(ADC_REG_OFFSET(0),(unsigned char*)&anaInData[0].offsetReg,4);

        halAd7124SetOpMode(ADC_MODE_FS_CALI);
        palSetGroupMode(GPIOB,PAL_PORT_BIT(4),0,PAL_STM32_ALTERNATE(0));
        extChannelEnable(&EXTD1,4);
        SPI_SELECT();
        caliState = CALI_INT_GAIN;
      }
      break;
    case CALI_INT_GAIN:
      if(evt & EV_ADC_DATA_READY){
        // disable interrupt function
        palSetGroupMode(GPIOB,PAL_PORT_BIT(4),0,PAL_STM32_ALTERNATE(5) | PAL_STM32_MODE_ALTERNATE);
        SPI_DESELECT();
        ad7124RegRead(ADC_REG_GAIN(0),(unsigned char*)&anaInData[0].gainReg,4);
        done = true;
      }
      break;
    }
    
  }
  
  
  
  while(true){
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,MS2ST(5));

    if(!bSampling){
      bSampling = true;
      bContinueRead = true;
      adcResult = 0;
      nofSamplesAcquired = 0;
      nofSamplesIgnored = 2;
      if(currentChannel < 8){
        switch(anaInData[currentChannel].settings->inputType){
        case AIO_PT100:
        case AIO_PT1000:
          ad7124SetIout0(anaInData[currentChannel].settings->inp, anaInData[currentChannel].settings->inn,IOUT_250,IOUT_250);
          chThdSleepMilliseconds(1);
          break;
        }
      }
      ad7124_change_input(currentChannel);
      // start conversion
      halAd7124StartContConversion();
      palSetGroupMode(GPIOB,PAL_PORT_BIT(4),0,PAL_STM32_ALTERNATE(0));
      extChannelEnable(&EXTD1,4);
    }else{
//      releaseCounter++;
//      if(releaseCounter > 200){
//        releaseCounter = 0;
//        SPI_DESELECT();
//        bSampling = false;
//        extChannelDisable(&EXTD1,4);
//      }

    }

    if(evt & EV_ADC_DATA_READY){
      // disable interrupt function
      palSetGroupMode(GPIOB,PAL_PORT_BIT(4),0,PAL_STM32_ALTERNATE(5) | PAL_STM32_MODE_ALTERNATE);
      if(caliState == CALI_INT_OFFSET){
        // read gain offset register
        ad7124RegRead(ADC_REG_OFFSET(currentChannel),(unsigned char*)anaInData[currentChannel].offsetReg,4);
      }
      else{
        if(nofSamplesAcquired == NOF_SAMPLE_NUMBER){
          // stop adc
          halAd7124DummyRead();
          SPI_DESELECT();
          adcResult = 0;
          for(i=0;i<NOF_SAMPLE_NUMBER;i++){
            adcResult += adcHistory[i];
          }
          adcResult >>= 3;
          if(currentChannel == 0xff){
            boardTemp = ((adcResult) - 8388608)/13584. - 272.5;
            updateColdT(boardTemp);
          }
          else{
            sysSetAnaInData(currentChannel,adcResult);
          }
            nextChannel = currentChannel+1;
            if(currentChannel == 0xff)
              nextChannel = 0;
            else if(nextChannel == NOF_ANACHANNEL-1)
              nextChannel = 0xff;
            currentChannel = nextChannel;
            bSampling = false;
        }else{
          if(nofSamplesIgnored > 0){
            nofSamplesIgnored--;
            halAd7124DataRead();
          }else{
            //adcResult += halAd7124DataRead();
            adcHistory[nofSamplesAcquired] =halAd7124DataRead();
            nofSamplesAcquired++;
          }
          // enable interrupt
          palSetGroupMode(GPIOB,PAL_PORT_BIT(4),0,PAL_STM32_ALTERNATE(0));
          extChannelEnable(&EXTD1,4);
        }
      }
    }
    if(evt & EV_ADC_DRDY){
      //halAd7124DataRead();
    }
    if(evt & EV_ADC_START_SINGLE){
      if(!bSampling){
        bSampling = true;
        releaseCounter = 0;
        bContinueRead = false;
        chCount = 0;
        halAd7124StartSingleConv();
        palSetGroupMode(GPIOB,PAL_PORT_BIT(4),0,PAL_STM32_ALTERNATE(0));
        extChannelEnable(&EXTD1,4);
      }
    }
    if(evt & EV_ADC_START_CONT){
      bContinueRead = true;
      adcResult = 0;
      nofSamplesAcquired = 0;
      halAd7124StartContConv();
    }
    if(evt & EV_ADC_STOP){
      halAd7124StopConversion();
    }
    if(evt & EV_ADC_START_CALI){
    }
    if(evt & EV_ADC_DONE_CALI){
    }
    if(evt & EV_ADC_CONFIG){
      // stop sampling
      SPI_DESELECT();
      // reconfig ADC
      hal_ad7124_init_config();
    }
    
    if(evt & EV_ADC_RESTART){
      SPI_DESELECT();
      bSampling = false;
    }
    
    if(evt & EV_ADC_INT_OFF_CALI){
      
    }
    if(evt & EV_ADC_INT_GAIN_CALI){
      
    }
    
    
  }
}


int halAD7124ScanRegs()
{
  uint8_t rx[8];
  uint8_t i;
    // read ID register
  ad7124RegRead(0x0,rx,2);
  ad7124RegRead(0x1,rx,2);
  ad7124RegRead(0x2,rx,2);
  ad7124RegRead(0x5,rx,1);
  ad7124RegRead(0x6,rx,3);
  ad7124RegRead(0x7,rx,3);

  for(i=0;i<8;i++){
    ad7124RegRead(i+0x9,rx,2);
    ad7124RegRead(i+0x19,rx,2);
  }
  return 0;
}



int halAd7124Init(SPIDriver *spi)
{
  int i;
  uint8_t sta;
  uint8_t rx[8],tx[8];
  
  spiDev = spi;
  // create working thread
  thdAD7124 = chThdCreateStatic(waAD7124,sizeof(waAD7124),NORMALPRIO,thAD7124,(SPIDriver*)spi);
  chSysLock();
  chThdSuspendS(&ad7124_trp);
  chSysUnlock();
  return 1;
}
