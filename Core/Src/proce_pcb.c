#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include <string.h>
#include <stdio.h>
#include "messages.h"
#include <stdbool.h>
#include "crc.h"
#include "Thruster.h"
#include "adc_work.h"
#include "communication.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
uint32_t dtick;

void proce_init(){
    init_comm();
    init_thrusters();
    adc_init();
}

void proce_loop(){

	thrusters_update();
	update_com();
	trans_adc();
}
