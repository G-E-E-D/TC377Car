#ifndef ZF_COMMON_HEADFILE_H
#define ZF_COMMON_HEADFILE_H

#include <stdint.h>

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int8_t   int8;
typedef int16_t  int16;
typedef int32_t  int32;

typedef int pwm_channel_enum;
typedef int gpio_pin_enum;
typedef int gpio_level_enum;
typedef int encoder_index_enum;

#define GPIO_LOW       0
#define GPIO_HIGH      1
#define GPI            0
#define GPO            1
#define GPI_PULL_UP    1
#define GPO_PUSH_PULL  1
#define PWM_DUTY_MAX   10000

#define P20_6  206
#define P20_7  207
#define P21_4  214
#define P21_2  212
#define P02_5  25

#define ATOM0_CH3_P21_5  3
#define ATOM0_CH1_P21_3  1
#define ATOM0_CH4_P02_4  4

#define TIM2_ENCODER             2
#define TIM2_ENCODER_CH1_P33_7   337
#define TIM2_ENCODER_CH2_P33_6   336

#define IPS200_TYPE_SPI  1
#define IPS200_PORTAIT   1
#define TAU1201          1

#define RGB565_RED       0xF800
#define RGB565_BLACK     0x0000
#define RGB565_GREEN     0x07E0
#define RGB565_YELLOW    0xFFE0
#define RGB565_GRAY      0x8410

typedef struct
{
    uint8 state;
    uint8 satellite_used;
    uint8 fix_quality;
    double latitude;
    double longitude;
    int8 ns;
    int8 ew;
    float speed;
    float direction;
    float hdop;
} gnss_info_struct;

extern gnss_info_struct gnss;
extern volatile uint8 gnss_flag;
extern volatile uint8 gnss_rmc_flag;
extern volatile uint8 gnss_gga_flag;
extern volatile uint32 gnss_rmc_count;
extern volatile uint32 gnss_gga_count;
extern int16 mpu6050_gyro_z;
extern int16 mpu6050_acc_x;
extern int16 mpu6050_acc_y;
extern uint16 ips200_width_max;
extern uint16 ips200_height_max;

void gpio_init(gpio_pin_enum pin, int mode, gpio_level_enum level, int config);
void gpio_set_level(gpio_pin_enum pin, gpio_level_enum level);
gpio_level_enum gpio_get_level(gpio_pin_enum pin);
void pwm_init(pwm_channel_enum channel, uint32 frequency, uint32 duty);
void pwm_set_duty(pwm_channel_enum channel, uint32 duty);
void encoder_dir_init(encoder_index_enum index, int ch1, int ch2);
void encoder_quad_init(encoder_index_enum index, int ch1, int ch2);
int16 encoder_get_count(encoder_index_enum index);
void encoder_clear_count(encoder_index_enum index);
int mpu6050_init(void);
void mpu6050_get_gyro(void);
void mpu6050_get_acc(void);
float mpu6050_gyro_transition(int16 raw);
float mpu6050_acc_transition(int16 raw);
void gnss_init(int device);
uint8 gnss_data_parse(void);
void system_delay_ms(uint32 ms);
void ips200_set_dir(int dir);
void ips200_set_color(uint16 foreground, uint16 background);
void ips200_init(int type);
void ips200_clear(void);
void ips200_show_string(uint16 x, uint16 y, const char *text);
void ips200_show_uint(uint16 x, uint16 y, uint32 value, uint8 width);
void ips200_show_int(uint16 x, uint16 y, int32 value, uint8 width);
void ips200_show_float(uint16 x, uint16 y, float value, uint8 int_width, uint8 frac_width);
void ips200_draw_point(uint16 x, uint16 y, uint16 color);
void ips200_draw_line(uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color);

#endif
