#include "main.h"

#define delay_init 10000
#define kpercsec 0.1

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define THRUSTER_PWM_MAX 1200
#define THRUSTER_PWM_NEUTRAL 1500
#define THRUSTER_PWM_MIN 1800
#define THRUSTERS_NUM 8

typedef struct {
    uint8_t address;
    TIM_HandleTypeDef *htim;
    uint32_t chanel;
    uint16_t velocity;
    uint16_t current;
    GPIO_TypeDef *port;
    uint16_t pin;
    bool enablead;
    bool need_update;
} thruster_t;

void init_thrusters();
void get_thrusters();
void thrusters_update();
void set_thruster_velocity(uint8_t address, uint16_t velocity);
int search_thruster_by_address(uint8_t address);

