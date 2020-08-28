#ifndef _DIGITAL_IO_
#define _DIGITAL_IO_


#define NOF_DIG_IO      4

enum _dio_dir_e{
  DIO_DIR_INPUT,
  DIO_DIR_OUTPUT,
  DIO_DIR_BYDIREC
};

enum _dio_type_e{
  DIO_TYPE_GPIO,
  DIO_TYPE_FREQ,    // output: frequency, input: counter
  DIO_TYPE_PWM,     // output: pwm, input: capture
};

typedef struct{
  uint8_t direction;
  uint8_t type;
  uint32_t val;
  uint32_t fraction;
  uint32_t base;
  ioportid_t port;
  uint16_t pad;
}digital_io_type_t;

typedef struct dio_map_s{
  ioportid_t port;
  uint16_t pad;
  void (*set)(struct dio_map_s*);
  void (*clear)(struct dio_map_s*);
  uint8_t (*read)(struct dio_map_s*);
}dio_map_t;



extern digital_io_type_t digIO[4];

int8_t dio_read(uint8_t ch);
void dio_write(uint8_t ch, uint8_t val);
void dio_set_in_def(digital_io_type_t *d, uint8_t nof_pins);
void dio_set_out_def(digital_io_type_t *d, uint8_t nof_pins);

#endif
