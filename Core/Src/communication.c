#include <communication.h>
#include <string.h>
#include "crc.h"
#include "messages.h"
#include "Thruster.h"
#include "main.h"
#include "rov.h"


static uint8_t message_buff[RPI_REQUEST_LENGTH];
static uint8_t tx_message_buff[RPI_RESPONSE_LENGTH];

uint32_t hal_com_ticks = 0;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
rov_t rov_state;
extern thruster_t thrusters[THRUSTERS_NUM];

bool frameready = false;

bool parse_package(uint8_t  *message)
{
	//check_crc(message, RPI_REQUEST_LENGTH)
    if  (1)  {
    	for(int i = 0; i < THRUSTERS_NUM; i++)
    	{
        set_thruster_velocity(i, ((struct RPIRequest*)message)->velocity[i]);
    	}

    	memcpy(rov_state.Light_pw, ((struct RPIRequest*)message)->Light_pw, sizeof(rov_state.Light_pw));
    	rov_state.con_state = ((struct RPIRequest*)message)->con_state;
    	rov_state.tilt = ((struct RPIRequest*)message)->tilt;

        return true;
    }
    return false;
}

void init_comm(){
	HAL_UART_Receive_DMA(&huart4, message_buff, RPI_REQUEST_LENGTH);
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if(huart == &huart4){
        HAL_UART_DeInit(&huart4);
        frameready = false;
        HAL_UART_Receive_DMA(&huart4, message_buff, RPI_REQUEST_LENGTH);
    }
}

void update_com(){
    if(frameready){
    	parse_package(message_buff);
        frameready = false;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        ((struct RPIResponse *)tx_message_buff)->A5 = 0xA5;
        ((struct RPIResponse *)tx_message_buff)->con_state = rov_state.con_state;
        ((struct RPIResponse *)tx_message_buff)->current_logic = rov_state.current_logic;
    	memcpy(((struct RPIResponse *)tx_message_buff)->vol_bat_cell, rov_state.vol_bat_cell, sizeof(rov_state.vol_bat_cell));

        ((struct RPIResponse *)tx_message_buff)->current_vma[0] = thrusters[0].current;
        ((struct RPIResponse *)tx_message_buff)->current_vma[1] = thrusters[1].current;
        ((struct RPIResponse *)tx_message_buff)->current_vma[2] = thrusters[2].current;
        ((struct RPIResponse *)tx_message_buff)->current_vma[3] = thrusters[3].current;
        ((struct RPIResponse *)tx_message_buff)->current_vma[4] = thrusters[4].current;
        ((struct RPIResponse *)tx_message_buff)->current_vma[5] = thrusters[5].current;
        ((struct RPIResponse *)tx_message_buff)->current_vma[6] = thrusters[6].current;
        ((struct RPIResponse *)tx_message_buff)->current_vma[7] = thrusters[7].current;
        ((struct RPIResponse *)tx_message_buff)->crc = calculate_crc(tx_message_buff, RPI_RESPONSE_LENGTH-1);

        HAL_UART_Transmit_DMA(&huart4, tx_message_buff, RPI_RESPONSE_LENGTH);
        while(HAL_UART_Receive_DMA(&huart4, message_buff, RPI_REQUEST_LENGTH)!=HAL_OK);
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == &huart4){
        frameready = true;
    }
}

void AddChecksumm8b(uint8_t *msg, uint16_t length)
{
	uint8_t crc = 0;
	int i = 0;

	for(i=0; i < length - 1; i++) {
		crc ^= msg[i];
	}

	msg[length-1] = crc;
}
