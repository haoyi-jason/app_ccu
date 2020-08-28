#ifndef _HAL_AD7124_
#define _HAL_AD7124_

#define REG_READ_MASK 0x40

#define ADC_REG_CTRL            0x01
#define ADC_REG_DATA            0x02
#define ADC_REG_IOCTRL1         0x03
#define ADC_REG_IOCTRL2         0x04
#define ADC_REG_ID              0x05
#define ADC_REG_ERROR           0x06
#define ADC_REG_ERREN           0x07
#define ADC_REG_MCLKCNT         0x08
#define ADC_REG_CHANNEL(x)      (x + 0x09)
#define ADC_REG_CONFIG(x)       (x + 0x19)
#define ADC_REG_FILTER(x)       (x + 0x21)
#define ADC_REG_OFFSET(x)       (x + 0x29)
#define ADC_REG_GAIN(x)         (x + 0x31)


#define REG_ADCCTRL_ADDR	0x01
#define ADC_LOW_POWER		0x01
#define ADC_MID_POWER		0x02
#define ADC_FULL_POWER		0x03

#define ADC_CLK_INT_OUT_DIS	0x00
#define ADC_CLK_INT_OUT_EN	0x01
#define ADC_CLK_EXT			0x02
#define ADC_CLK_EXT_DIV4	0x03

// ADC modes
#define ADC_MODE_CONT		0x00
#define ADC_MODE_SINGLE		0x01
#define ADC_MODE_STBY		0x02
#define ADC_MODE_PWDN		0x03
#define ADC_MODE_IDLE		0x04
#define ADC_MODE_ZERO_CALI	0x05
#define ADC_MODE_FS_CALI	0x06
#define ADC_MODE_ZERO_SYS	0x07
#define ADC_MODE_FS_SYS		0x08

#define ADC_CTRL_DOUTRDY(x)	(x<<4)
#define ADC_CTRL_CONT(x)	(x<<3)
#define ADC_CTRL_DSTA(x)	(x<<2)
#define ADC_CTRL_CSBEN(x)	(x<<1)
#define ADC_CTRL_REFEN(x)	(x)

#define ADC_CTRL_POWER(x)	(x<<6)
#define ADC_CTRL_MODE(x)	(x<<2)
#define ADC_CTRL_CLKSEL(x)		(x)

#define REG_ID_ADDR			0x05	// read = 0x12
#define REG_ERROR_ADDR		0x06
#define REG_ERROR_EN_ADDR 	0x07
#define REG_MCLK_COUNT_ADDR 0x08

#define ADC_CH_SETUP_MASK 0x70
#define ADC_CH_EN_MASK          0x80
#define REG_CHANNEL_ADDR(x) (x+0x09)
#define ADC_CHANNEL_EN(x)	(x<<7)
#define ADC_CHANNEL_CONFIG(x) (x << 4)
#define ADC_CHANNEL_INP0(x) (x>>2)
#define ADC_CHANNEL_INP1(x) (x<<5)
#define ADC_CHANNEL_INN(x) (x)

#define ADC_IN(x)	x
#define ADC_IN_TEMP	0x10
#define ADC_IN_AVSS	0x11
#define ADC_IN_INTREF	0x12
#define ADC_IN_DGNE	0x13

enum pga_e{PGA_X1,PGA_X2,PGA_X4,PGA_X8,PGA_X16,PGA_X32,PGA_X64,PGA_X128};
enum refsel_e{REF_EXT1,REF_EXT2,REF_INT_2_5,REF_AVDD};

#define ADC_CONFIG_ADDR(x) (x + 0x19)
#define ADC_CONFIG_BIPOLAR(x) (x<<3)
#define ADC_CONFIG_BURNOUT(x) (x<<1)
#define ADC_CONFIG_REF_BUFP(x) (x<<0)
#define ADC_CONFIG_REF_BUFM(x) (x<<7)
#define ADC_CONFIG_AIN_BUFP(x) (x<<6)
#define ADC_CONFIG_AIN_BUFM(x) (x<<4)
#define ADC_CONFIG_REFSEL(x) (x<<3)
#define ADC_CONFIG_PGA(x) (x)

enum filter_type_e{FLT_SINC4,FLT_RSV1,FLT_SINC3,FLT_RSV2,FLT_FAST4,FLT_FAST3,FLT_RSV3,FLT_POST};
enum rej_e{REJ_50HZ,REJ_60HZ};


#define REG_FILTER_ADDR(x) (x+0x21)
#define ADC_FILTER_FILTER(x)	(x<<5)
#define ADC_FILTER_REJ60(x)		(x<<4)
#define ADC_FILTER_POST(x)		(x<<1)
#define ADC_FILTER_SINGLE(x)	(x)
#define ADC_FILTER_FSH(x) (x >> 8)
#define ADC_FILTER_FSL(x) (x & 0xff)


#define REG_GAIN_ADDR(x) (x+0x31)

#define REG_OFFSET_ADDR(x) (x+0x29)

#define REG_IOCTRL1_ADDR	0x03
#define GPIO_PULL_HIGH	1
#define GPIO_PULL_LOW	0
#define IO_GPIO_CFG(x)	x
#define IO_GPIO_HIGH(x)	(1<<(x+4))
#define IO_GPIO_LOW(x)	(~IO_GPIO_HIGH(x))
#define IO_PDSW_SET(x)	(x << 7)
#define IO_IOUT1_RANG(x)	(x << 3)
#define IO_IOUT0_RANG(x)	(x)
#define IO_IOUT1_CH(x)		(x << 4)
#define IO_IOUT0_CH(x)		(x)
#define IO_IOUT0_MASK           (0x7)
#define IO_IOUT1_MASK           (0x7 << 3)


#define IOUT_OFF	0x00
#define IOUT_50		0x01
#define IOUT_100	0x02
#define IOUT_250	0x03
#define IOUT_500	0x04
#define IOUT_750	0x05
#define IOUT_1000	0x06


#define REG_ADDR_DATA	0x02

#define REG_VBIAS_ADDR	0x04
#define VBIAS_EN(x)	(1<<x)

enum polarity_e{UNIPOLAR,BIPOLAR};
enum burnout_e{BO_OFF,BO_05U,BO_20U,BO_40U};
enum refbuf_e{REF_BUF_OFF,REF_BUF_ON};
enum ainbuf_e{AIN_BUF_OFF,AIN_BUF_ON};
enum {REF_SEL_1,REF_SEL_2,REF_SEL_IR,REF_SEL_AV};

enum {CH_DISABLE,CH_ENABLE};
enum {CHS_TC,CHS_RTD,CHS_VOLT,CHS_CURRENT};	// configuration defines
enum {SENSOR_IN,VOLT_IN};
enum {VBIAS_OFF,VBIAS_ON};
enum {IEXEC_OFF,IEXEC_ON};

typedef struct{
  unsigned char regAddr;
  union{
    struct{
      unsigned char rsv:4;
      unsigned char polarity:1;
      unsigned char burnout:2;
      unsigned char ref_bufp:1;
      unsigned char ref_bufm:1;
      unsigned char ain_bufp:1;
      unsigned char ain_bufm:1;
      unsigned char ref_sel:2;
      unsigned char pga:3;
    }u;
    unsigned char b[2];
  };
}setup_t;

#define CHANNEL_ENABLE  0x80
#define CHANNEL_MASK    ~CHANNEL_ENABLE
#define CHANNEL_SETUP_MASK      0xf0
#define CHANNEL_SETUP(x)       (x << 4)
#define AIN_POS(x)      (x << 5)
#define AIN_NEG(x)      (x << 8)

#define CFG_BIPOLAR     0x0800
#define CFG_BIPOLAR_MSK ~CFG_BIPOLAR
#define CFG_BURNOUT(x)  (x << 8)
#define CFG_REF_BUFP    0x0100
#define CFG_REF_BUFP_MSK        ~CFG_REF_BUFP
#define CFG_REF_BUFN    0x0080
#define CFG_REF_BUFN_MSK        ~CFG_REF_BUFN
#define CFG_AIN_BUFP    0x0040
#define CFG_AIN_BUFP_MSK        ~CFG_AIN_BUFP
#define CFG_AIN_BUFN    0x0020
#define CFG_AIN_BUFN_MSK        ~CFG_AIN_BUFN
#define CFG_REF_SEL(x)  (x << 3)
#define CFG_PGA_MASK    0x7
#define CFG_PGA(x)      (x)

#define FLT_TYPE(x)     (x << 21)
#define FLT_REJ60       0x100000
#define FLT_POST(x)     (x << 17)
#define FLT_SGCYCLE     0x010000
#define FLT_FS(x)       (x)

typedef union{
  unsigned char b[2];
  unsigned short s;
}UU16;

typedef union{
  unsigned char b[4];
  unsigned long l;
}UU32;



typedef struct{
  unsigned char regAddr;
  union{
    struct{
      unsigned char filter:3;
      unsigned char rej60:1;
      unsigned char post_filter:3;
      unsigned char single_cycle:1;
      unsigned short fs;
    }u;
    unsigned char b[3];
  };
}filter_t;


typedef struct{
  unsigned char regAddr;
  union{
    struct{
      unsigned char ainm:5;
      unsigned char ainp:5;
      unsigned char rsv:2;
      unsigned char setup:3;
      unsigned char enable:1;
    }u;
    unsigned char b[2];
    uint16_t v;
  };
  long result;
}channel_t;


typedef struct{
  uint8_t regAddr;
  uint8_t b[3];
  uint8_t sz;
  int32_t data;
  float scaleV;
}channel_u;

typedef struct{
  unsigned char regAddr;
  union{
    struct{
      unsigned char ref_en:1;
      unsigned char csb_en:1;
      unsigned char data_status:1;
      unsigned char cont_read:1;
      unsigned char drdy_del:1;
      unsigned char rsv:3;
      unsigned char clk_sel:2;
      unsigned char mode:4;
      unsigned char power_mode:2;
    }u;
    unsigned char b[2];
  };
}adc_control_t;

typedef struct{
  unsigned char regAddr;
  union{
    struct{
      unsigned char gp4_dat:1;
      unsigned char gp3_dat:1;
      unsigned char gp2_dat:1;
      unsigned char gp1_dat:1;
      unsigned char gp4_ctrl:1;
      unsigned char gp3_ctrl:1;
      unsigned char gp2_ctrl:1;
      unsigned char gp1_ctrl:1;
      unsigned char pdsw:1;
      unsigned char iout1:3;
      unsigned char iout2:3;
      unsigned char iout1_ch:4;
      unsigned char iout2_ch:4;
    }u;
      unsigned char b[3];
  };
}io_contro1_t;

typedef struct{
	unsigned char enable;	// enable or disable
	unsigned char setup;
	unsigned char chp;
	unsigned char chm;
	unsigned char highRange;
	unsigned char vbias;
	unsigned char iexc;
	long result;
	float resuV;
}channelSetup_t;


#define NOFCHANNEL	16	// number of channels

#define NOFCONFIG	8
//#define CFG_BIPOLAR(x) (x << 3)
//#define CFG_BURNOUT(x) (x << 1)
//#define CFG_REF_BUFP(x) (x)
//#define CFG_REF_BUFM(x) (x <<7)
//#define CFG_AIN_BUFP(x) (x << 6)
//#define CFG_AIN_BUFM(x) (x << 5)
//#define CFG_REF_SEL(x) (x << 3)
//#define CFG_PGA(x)	(x)


typedef struct{
	unsigned char bipolar;
	unsigned char burnout;
	unsigned char refBufp;
	unsigned char refBufm;
	unsigned char ainBufp;
	unsigned char ainBufm;
	unsigned char refSel;
	unsigned char pga;
}channelConfig_t;

#define NOFFILTER	8
//#define FLT_FLT(x) (x<<5)
//#define FLT_REJ60(x) (x << 4)
//#define FLT_POST(x) (x << 1)
//#define FLT_SC(x) (x)
//#define FLT_FSH(x) (x >> 8)
//#define FLT_FSL(x) (x)

typedef struct{
	unsigned char filterType;
	unsigned char rej60;
	unsigned char postFilter;
	unsigned char singleCycle;
	unsigned short fs;
}filterReg_s;

#define EV_ADC_DATA_READY       EVENT_MASK(1)
#define EV_ADC_DRDY             EVENT_MASK(2)
#define EV_ADC_START_SINGLE     EVENT_MASK(3)
#define EV_ADC_START_CONT       EVENT_MASK(4)
#define EV_ADC_STOP             EVENT_MASK(5)
#define EV_ADC_START_CALI       EVENT_MASK(6)
#define EV_ADC_DONE_CALI        EVENT_MASK(7)
#define EV_ADC_CONFIG           EVENT_MASK(8)
#define EV_ADC_RESTART          EVENT_MASK(9)
#define EV_ADC_INT_OFF_CALI     EVENT_MASK(10)
#define EV_ADC_INT_GAIN_CALI    EVENT_MASK(11)


int halAd7124SetupChannel(void);
int halAd7124Init(SPIDriver *spi);
int halAd7124ChangeChannelSetup(unsigned char ch, unsigned char setup);
int halAd7124EnableChannel(unsigned char ch);
int halAd7124DisableChannel(unsigned char ch);
int halAd7124StartConversionCont(void);
int halAd7124StartConversionSingle(void);
int halAd7124StopConversion(void);
void ad7124RdyHandler(EXTDriver *extp, expchannel_t channel);
long halAd7124DataRead(void);
void hal_ad7124_init_config();
#endif
