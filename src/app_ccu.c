#include "ch.h"
#include "hal.h"
#include "ccu.h"
#include "gpio_drv.h"
#include "shell.h"
#include "ads1015.h"

appParam_t appParam;

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE
};
static ads1x15_config_t ads1105_config = {
 &I2CD1,
 &i2ccfg,
 DEV_ADS1015
};
ADS1x15Driver ads1015 = {
  &ads1105_config,
  0x0480,
  0x0000,
  0x8000,
  ADS1015_ADDR_GND,
};


// CAN config for 500K baud

static CANConfig canCfg500K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(6) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_SJW(0)
};

// can config for 250K baud
static CANConfig canCfg250K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(20) | CAN_BTR_TS1(5) | CAN_BTR_TS2(0) | CAN_BTR_SJW(0)
};


// shell command on UART1
static THD_WORKING_AREA(waShell,2048);
static void cmdReadId(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmddc5v(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmdlamp(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmdwakeout(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmdrstout(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmdvcpout(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmdmc1out(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmdmc2out(BaseSequentialStream *chp, int argc, char *argv[]);
//
static const ShellCommand commands[] = {
  {"readid",cmdReadId},
  {"dc5v",cmddc5v},
  {"lamp",cmdlamp},
  {"wakeout",cmdwakeout},
  {"rst",cmdrstout},
  {"vcp",cmdvcpout},
  {"mc1",cmdmc1out},
  {"mc2",cmdmc2out},
  {NULL,NULL}
};
//
static const ShellConfig shell_cfg = {
  (BaseSequentialStream*)&SD1,
  commands
};
// end shell command


//AD5593RDriver ad_dev0,ad_isens;
thread_t *thdCCU;
thread_t *can1Recv, *can2Recv;
thread_reference_t ccu_ref;

static msg_t canSend(CANDriver *p, j1939_msg_t *m);

static THD_WORKING_AREA(waCANRX1,256);
static THD_WORKING_AREA(waCANRX2,256);
static THD_FUNCTION(thCanRx,p)
{
  (void)p;
  CANDriver *ip = p;
  CANRxFrame rxmsg;
  event_listener_t el;
  j1939_msg_t m;
  uint8_t i;
  //chRegSetThreadName("CANRx_01");
  
  chEvtRegister(&ip->rxfull_event,&el,1);
  while(true){
    if(chEvtWaitAnyTimeout(ALL_EVENTS,TIME_MS2I(100)) == 0){
      continue;
    }
    
    while(canReceive(ip,CAN_ANY_MAILBOX,&rxmsg,TIME_IMMEDIATE) == MSG_OK){
      m.pgn = (rxmsg.EID >> 8) & 0x1ffff;
      m.src = (rxmsg.EID & 0xff);
      m.len = rxmsg.DLC;
      memcpy(m.buffer,rxmsg.data8,rxmsg.DLC);
      if(ip == &CAND1){// && (appParam.testState.echo_canbus & 0x1)){
        char msg[64];
        char *ptr = msg;
        char *q = (char*)&m;
        for(uint8_t i=0;i<16;i++){
          ptr += chsnprintf(ptr,64,"%02x-",*q++);
        }
        chprintf(&SD1,"CAN 1 Received:%s\n",msg);
      }
      else if(ip == &CAND2){// && (appParam.testState.echo_canbus & 0x2)){
        char msg[64];
        char *ptr = msg;
        char *q = (char*)&m;
        for(uint8_t i=0;i<16;i++){
          ptr += chsnprintf(ptr,64,"%02x-",*q++);
        }
        chprintf(&SD1,"CAN 2 Received:%s\n",msg);
      }
      
    }
  }
  chEvtUnregister(&ip->rxfull_event,&el);
}


static SerialConfig serialcfg = {
  115200,
};


I2CDriver *pca_dev = &I2CD1;
#define PCA9536_ADR     0x41
void pca9536_init(void)
{
  uint8_t tx[2];
  uint8_t rx[2];
  i2cAcquireBus(pca_dev);
  i2cStart(pca_dev,&i2ccfg);
  // set io direction(byte 2), 0:output, 1:input
  tx[0] = 0x3;
  tx[1] = 0xF;
  msg_t ret = i2cMasterTransmitTimeout(pca_dev,PCA9536_ADR,tx,2,rx,0,TIME_MS2I(100));
  ret = i2cMasterTransmitTimeout(pca_dev,PCA9536_ADR,tx,1,rx,2,TIME_MS2I(100));
  
  i2cStop(pca_dev);
  i2cReleaseBus(pca_dev);
}

uint8_t pca9536_read(void)
{
  uint8_t tx[2];
  uint8_t rx[2];
  i2cAcquireBus(pca_dev);
  i2cStart(pca_dev,&i2ccfg);
  tx[0] = 0x0;
  msg_t ret = i2cMasterTransmitTimeout(pca_dev,PCA9536_ADR,tx,1,rx,1,TIME_MS2I(100));
  i2cStop(pca_dev);
  i2cReleaseBus(pca_dev);
  
  return rx[0];
}


void pca9536_output(uint8_t bit, uint8_t set)
{
  uint8_t tx[2];
  uint8_t rx[2];
  i2cAcquireBus(pca_dev);
  i2cStart(pca_dev,&i2ccfg);
  tx[0] = 0x1;
  tx[1] = 0x0;
  msg_t ret = i2cMasterTransmitTimeout(pca_dev,PCA9536_ADR,tx,1,rx,2,TIME_MS2I(100));
  if(set)
    tx[1] = rx[0] | (1 << bit);
  else
    tx[1] = rx[0] & (~(1 << bit));
  ret = i2cMasterTransmitTimeout(pca_dev,PCA9536_ADR,tx,2,0,0,TIME_MS2I(100));
  i2cStop(pca_dev);
  i2cReleaseBus(pca_dev);
}
static event_source_t ads_rdy_event;

void ads1x15_drdy(void *arg)
{
  (void)arg;
  chSysLockFromISR();
  chEvtBroadcastFlagsI(&ads_rdy_event,EVENT_MASK(2));
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waADS, 512);
static THD_FUNCTION(procADS, arg) {
  (void)arg;
  event_listener_t el_adsrdy;
  chEvtObjectInit(&ads_rdy_event);
  chEvtRegister(&ads_rdy_event,&el_adsrdy,2);

  ads1015_set_mux(&ads1015,MUX_AIN0_GND);
//  ads1015_set_pga(&ads1015,PGA_2_048);
  ads1015_set_pga(&ads1015,PGA_4_096);
  ads1015_set_mode(&ads1015,MODE_CONT);
  ads1015_set_dr(&ads1015,DR_128);
  ads1015_set_thresLow(&ads1015);
  ads1015_set_thresHigh(&ads1015);
  // config adc drdy signal
  palEnableLineEvent(PAL_LINE(GPIOA, 4U),PAL_EVENT_MODE_FALLING_EDGE);
  palSetLineCallback(PAL_LINE(GPIOA, 4U),ads1x15_drdy,NULL);

  ads1015_start_conversion(&ads1015);
  int32_t result[2];
  uint8_t actCh = 0;
  while(1){
    chThdSleepMilliseconds(50);
    chEvtWaitAny(ALL_EVENTS);
    //chSysLock();
    eventflags_t flags = chEvtGetAndClearFlags(&el_adsrdy);
    //chSysUnlock();
    if(flags & EVENT_MASK(2)){
      result[actCh++] = ads1015_read_data(&ads1015);
      if(actCh == 2)
        actCh = 0;
      ads1015_set_mux(&ads1015,MUX_AIN0_GND+actCh);
    }
//    chThdSleepMilliseconds(50);
  }
  chThdExit(0);
}

const char *di_names[] = {
  "LAMP1",
  "LAMP2",
  "WAKEUP_FROM_CONTROLLER",
  "RST",
  "CS1",
  "CS2",
  "CPS",
  "AAS",
  "MC1_HIGH",
  "MC2_HIGH",
  "MC1_LOW",
  "MC2_LOW",
  "WKUP_INT_LK15",
  "WKUP_EXT",
  "MIS_EXT",
  "5VOUT_PG1",
  "5VOUT_PG2"
};

const char *do_names[] = {
  "LAMP1",
  "LAMP2",
  "WAKEUP_OUT",
  "RST_OUT",
  "VCP",
  "MC1",
  "MC2",
  "NA",
  "5VOUT_EN1",
  "5VOUT_EN2"
};

static THD_WORKING_AREA(waCCU,2048);
static THD_FUNCTION(thCCU ,p)
{
  (void)p;
  char msg[64];
  uint8_t id_old = pca9536_read();
  uint32_t digital_in[DI_ADC_RDY];
  for(uint8_t i=0;i<DI_ADC_RDY;i++){
    digital_in[i] = digital_input_read(i);
  }
  
  uint8_t digital_out[NOF_DO];
  for(uint8_t i=0;i<NOF_DO;i++){
    digital_out[i] = digital_output_read(i);
  }

  j1939_msg_t jmsg;
  jmsg.dst = 0x1234;
  jmsg.pgn = 100;
  jmsg.src =  0x5678;
  jmsg.buffer[0] = 0x00;
  jmsg.buffer[1] = 0x00;
  jmsg.buffer[2] = 0x00;
  jmsg.buffer[3] = 0x00;
  jmsg.buffer[4] = 0x00;
  jmsg.buffer[5] = 0x00;
  jmsg.buffer[6] = 0x00;
  jmsg.buffer[7] = 0x00;
  jmsg.len = 8;
  
  chprintf((BaseSequentialStream*)&SD1,"CCU Test Start\n"); 
  while(1){
    uint8_t id = pca9536_read();
    if(id != id_old){
      // output if changed;
      id_old = id;
      chprintf(&SD1,"ID Changed:[%d,%d,%d]\n",(id_old >>2) & 0x1,(id_old>>1)&0x1,id_old&0x1); 
    }
    
    for(uint8_t i=0;i<DI_ADC_RDY;i++){
      if(digital_in[i] != digital_input_read(i)){
        digital_in[i] = digital_input_read(i);
        chprintf(&SD1,"%s Changed:[%d]\n",di_names[i],digital_in[i]); 
      }
    }
    for(uint8_t i=0;i<NOF_DO;i++){
      if(digital_out[i] != digital_output_read(i)){
        digital_out[i] = digital_output_read(i);
        chprintf(&SD1,"%s Toggled:[%d]\n",do_names[i],digital_out[i]); 
      }
    }
    
    
    canSend(&CAND1,&jmsg);
    for(uint8_t i=0;i<8;i++){
      jmsg.buffer[i]++;
    }
    canSend(&CAND2,&jmsg);
    chThdSleepSeconds(1);
  }
  
}



void ccuInit(void)
{
  sdStart(&SD1,&serialcfg);
  //sdStart(&SD2,&serialcfg);
  //sdStart(&SD3,&serialcfg);
  canStart(&CAND1,&canCfg250K);
  canStart(&CAND2,&canCfg250K);
  
  i2cInit();
  i2cStart(&I2CD1,&i2ccfg);
  at24eep_init(&I2CD1,16,1024,0x50,1);
  
  uint8_t tx[32],rx[32];
  for(uint8_t i=0;i<32;i++)
    tx[i] = i+8;
  
  eepromWrite(0,32,tx);
  eepromRead(0,32,rx);
  digital_output(DO_5VEN1,1);
  //digital_output(DO_5VEN2,1);
  
  pca9536_init();
  // DIO test
  digital_output(DO_5VEN1,1);
  digital_output(DO_5VEN2,0);
//  digital_output(DO_LAMP1,1);
//  digital_output(DO_LAMP1,0);
//  digital_output(DO_LAMP2,1);
//  digital_output(DO_LAMP2,0);
//    chprintf((BaseSequentialStream*)&SD1,"CCU Test Start 123\n"); 
    chThdSleepMilliseconds(1000);

  chThdCreateStatic(waShell,sizeof(waShell),NORMALPRIO ,shellThread,(void*)&shell_cfg);

  
  can1Recv = chThdCreateStatic(waCANRX1,sizeof(waCANRX1),NORMALPRIO,thCanRx,&CAND1);
  can2Recv = chThdCreateStatic(waCANRX2,sizeof(waCANRX2),NORMALPRIO,thCanRx,&CAND2);
  thdCCU = chThdCreateStatic(waCCU,sizeof(waCCU),NORMALPRIO,thCCU,NULL);
  chThdCreateStatic(waADS, sizeof(waADS), NORMALPRIO, procADS, NULL);
  
  
}


enum {
  CAN_TX1,
  CAN_RX1,
  CAN_TX2,
  CAN_RX2
};

static THD_FUNCTION(thd_can_loop ,p)
{
  bool done = false;
  uint8_t canState = CAN_TX1;
  
  j1939_msg_t msg_out,msg_in;
  msg_out.dst = 0x1234;
  msg_out.pgn = 100;
  msg_out.src =  0x5678;
  msg_out.buffer[0] = 0x11;
  msg_out.buffer[1] = 0x22;
  msg_out.buffer[2] = 0x33;
  msg_out.buffer[3] = 0x44;
  msg_out.buffer[4] = 0x55;
  msg_out.buffer[5] = 0x66;
  msg_out.buffer[6] = 0x77;
  msg_out.buffer[7] = 0x88;
  msg_out.len = 8;
  
  CANDriver *c1 = &CAND1;
  CANDriver *c2 = &CAND1;
  event_listener_t el1,el2;
  chEvtRegister(&c1->rxfull_event,&el1,1);
  chEvtRegister(&c2->rxfull_event,&el1,2);
  eventmask_t evt;
  while(!done){
    canSend(&CAND1,&msg_out);
    evt  = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_MS2I(50));
  }
}

static msg_t canSend(CANDriver *p, j1939_msg_t *m)
{
  CANTxFrame ptx;
  uint8_t i;
  
  ptx.EID = (m->prio << 26) | (m->pgn << 8) | m->src;
  ptx.IDE = CAN_IDE_EXT;
  ptx.RTR = CAN_RTR_DATA;
  ptx.DLC = m->len;
  memcpy(ptx.data8,m->buffer,m->len);
  
  return canTransmit(p,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
}


static void cmdReadId(BaseSequentialStream *chp, int argc, char *argv[])
{
  //chprintf(chp,"Read ID:\n");
  uint8_t id = pca9536_read();
  chprintf(chp,"Read ID bits:[%d,%d,%d]\n",((id >>2) & 0x1),((id>>1)&0x1,(id&0x1))); 
}

static void cmddc5v(BaseSequentialStream *chp, int argc, char *argv[])
{
  if(argc == 1){
    if(strncmp(argv[0],"ON",2) == 0){
      digital_output(DO_5VEN2,1);
    }
    else if(strncmp(argv[0],"OFF",3) == 0){
      digital_output(DO_5VEN2,0);
    }
  }
}

static void cmdlamp(BaseSequentialStream *chp, int argc, char *argv[])
{
  if(argc == 2){
    uint8_t id = strtok(*argv[0]) - 0x30;
    if(id > 1) return;
    if(strncmp(argv[1],"ON",2) == 0){
      digital_output(DO_LAMP1 + id,1);
    }
    else if(strncmp(argv[1],"OFF",3) == 0){
      digital_output(DO_LAMP1 + id,0);
    }
  }
}

static void cmdwakeout(BaseSequentialStream *chp, int argc, char *argv[])
{
  if(argc == 1){
    if(strncmp(argv[0],"ON",2) == 0){
      digital_output(DO_WKUP_FC,1);
    }
    else if(strncmp(argv[0],"OFF",3) == 0){
      digital_output(DO_WKUP_FC,0);
    }
  }
}

static void cmdrstout(BaseSequentialStream *chp, int argc, char *argv[])
{
  if(argc == 1){
    if(strncmp(argv[0],"ON",2) == 0){
      digital_output(DO_RST,1);
    }
    else if(strncmp(argv[0],"OFF",3) == 0){
      digital_output(DO_RST,0);
    }
  }
}

static void cmdReadCS(BaseSequentialStream *chp, int argc, char *argv[])
{
  uint8_t cs1_h_in = digital_input_read(DI_CS1);
  uint8_t cs1_l_in = digital_input_read(DI_CS2);
  chprintf(chp,"Read CSH/CSL:[%d,%d]\n",cs1_h_in,cs1_l_in); 
}

static void cmdvcpout(BaseSequentialStream *chp, int argc, char *argv[])
{
  if(argc == 1){
    if(strncmp(argv[0],"ON",2) == 0){
      digital_output(DO_VCP,1);
    }
    else if(strncmp(argv[0],"OFF",3) == 0){
      digital_output(DO_VCP,0);
    }
  }
}

static void cmdmc1out(BaseSequentialStream *chp, int argc, char *argv[])
{
  if(argc == 1){
    if(strncmp(argv[0],"ON",2) == 0){
      digital_output(DO_MC1,1);
    }
    else if(strncmp(argv[0],"OFF",3) == 0){
      digital_output(DO_MC1,0);
    }
  }
}

static void cmdmc2out(BaseSequentialStream *chp, int argc, char *argv[])
{
  if(argc == 1){
    if(strncmp(argv[0],"ON",2) == 0){
      digital_output(DO_MC2,1);
    }
    else if(strncmp(argv[0],"OFF",3) == 0){
      digital_output(DO_MC2,0);
    }
  }
}
