#ifndef _HALCANIO_H
#define _HALCANIO_H

typedef struct {
  GPIO_TypeDef *port;   // port
  unsigned long pin;    // pin
  unsigned long mode;   // mode
  unsigned long otype;
  unsigned long ospeedr;
  unsigned long pupdr;
  unsigned long odr;
  unsigned long afrl;
  unsigned long afrh;
}gpio_config_s;

typedef struct{
  int addr;
  int bit;
  int state;
}gpio_control_s;


void halJ1939Init(void);
void halCANMasterInit();
unsigned long halCANMasterReadInputs();
unsigned char halCANMasterReadInput(unsigned char pin);

void halCANSlaveInit();
unsigned long halCANSlaveReadInputs();
unsigned char halCANSlaveReadInput(unsigned char pin);
void halCANSlaveWriteOutputs(unsigned long val);
void halCANSlaveWriteOutput(unsigned char pin, unsigned char val);
void halCANSlaveReadOutputSense(unsigned char *result);
unsigned char halCANSlaveReadID(void);
unsigned long halCANSlaveReadOutputs();

void halCANLedOn();
void halCANLedOff();
void halCANLedToggle();

#endif