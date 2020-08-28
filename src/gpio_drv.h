#ifndef _GPIO_DRV_
#define _GPIO_DRV_

enum _di_names{
  DI_LAMP1,
  DI_LAMP2,
  DI_WKUP_FC,
  DI_RST,
  DI_CS1,
  DI_CS2,
  DI_CPS,
  DI_AAS,
  DI_MC1,
  DI_MC2,
  DI_MC11,
  DI_MC21,
  DI_WKUP_INT_LK15,
  DI_WKUP_EXT,
  DI_MIS_EXT,
  DI_5V_PG1,
  DI_5V_PG2,
  DI_ADC_RDY
};

enum _do_names{
  DO_LAMP1,
  DO_LAMP2,
  DO_WKUP_FC,
  DO_RST,
  DO_VCP,
  DO_MC1,
  DO_MC2,
  DO_NA,
  DO_5VEN1,  // peripheral power
  DO_5VEN2,    // output power
  NOF_DO
};

typedef struct dio_map_s{
  ioportid_t port;
  uint16_t pad;
  void (*set)(struct dio_map_s*);
  void (*clear)(struct dio_map_s*);
  uint8_t (*read)(struct dio_map_s*);
}dio_map_t;

//extern dio_map_t *do_map;
//extern dio_map_t *di_map;

int32_t digital_output(uint8_t ch, uint8_t val);
int32_t digital_output_read(uint8_t ch);
int32_t digital_input_read(uint8_t ch);
#endif
