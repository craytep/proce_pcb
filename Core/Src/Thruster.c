#include "Thruster.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

thruster_t thrusters[THRUSTERS_NUM];

int search_thruster_by_address(uint8_t address){
    for(int i = 0; i < THRUSTERS_NUM; i++){
        if(thrusters[i].address == address){
            return i;
        }
    }
}

void set_thruster_velocity(uint8_t address, uint16_t velocity){
    int thruster_num = search_thruster_by_address(address);
    thrusters[thruster_num].velocity = velocity;
    thrusters[thruster_num].need_update = true;
}

void thrusters_update(){
    for(int i = 0; i < THRUSTERS_NUM; i++){
        if(thrusters[i].need_update){
        	if((thrusters[i].velocity>THRUSTER_PWM_MIN)&&(thrusters[i].velocity<THRUSTER_PWM_MAX))
        	{
            __HAL_TIM_SET_COMPARE(thrusters[i].htim, thrusters[i].chanel, thrusters[i].velocity);
        	if((HAL_GPIO_ReadPin(thrusters[i].port, thrusters[i].pin)==GPIO_PIN_RESET))
        	{
          	  HAL_GPIO_WritePin(thrusters[i].port, thrusters[i].pin, GPIO_PIN_SET);
              __HAL_TIM_SET_COMPARE(thrusters[i].htim, thrusters[i].chanel, THRUSTER_PWM_NEUTRAL);
        	}

        	}else
        	{
            __HAL_TIM_SET_COMPARE(thrusters[i].htim, thrusters[i].chanel, THRUSTER_PWM_NEUTRAL);
        	if((HAL_GPIO_ReadPin(thrusters[i].port, thrusters[i].pin)==GPIO_PIN_RESET))
          	  HAL_GPIO_WritePin(thrusters[i].port, thrusters[i].pin, GPIO_PIN_SET);

        	}
        	if(thrusters[i].velocity==0)
        	{
        	  HAL_GPIO_WritePin(thrusters[i].port, thrusters[i].pin, GPIO_PIN_RESET);
        	}
            thrusters[i].need_update = false;
        }
    }
}



void init_thrusters_data(){
    thrusters[0].velocity = THRUSTER_PWM_NEUTRAL;
    thrusters[0].htim = &htim1;
    thrusters[0].chanel = TIM_CHANNEL_1;
    thrusters[0].need_update = true;
    thrusters[0].enablead = true;
    thrusters[0].port = CS1_GPIO_Port;
    thrusters[0].pin = CS1_Pin;


    thrusters[1].velocity = THRUSTER_PWM_NEUTRAL;
    thrusters[1].htim = &htim1;
    thrusters[1].chanel = TIM_CHANNEL_2;
    thrusters[1].need_update = true;
    thrusters[1].enablead = true;
    thrusters[1].port = CS2_GPIO_Port;
    thrusters[1].pin = CS2_Pin;


    thrusters[2].velocity = THRUSTER_PWM_NEUTRAL;
    thrusters[2].htim = &htim1;
    thrusters[2].chanel = TIM_CHANNEL_3;
    thrusters[2].need_update = true;
    thrusters[2].enablead = true;
    thrusters[2].port = CS3_GPIO_Port;
    thrusters[2].pin = CS3_Pin;

    thrusters[3].velocity = THRUSTER_PWM_NEUTRAL;
    thrusters[3].htim = &htim1;
    thrusters[3].chanel = TIM_CHANNEL_4;
    thrusters[3].need_update = true;
    thrusters[3].enablead = true;
    thrusters[3].port = CS4_GPIO_Port;
    thrusters[3].pin = CS4_Pin;

    thrusters[4].velocity = THRUSTER_PWM_NEUTRAL;
    thrusters[4].htim = &htim2;
    thrusters[4].chanel = TIM_CHANNEL_3;
    thrusters[4].need_update = true;
    thrusters[4].enablead = true;
    thrusters[4].port = CS5_GPIO_Port;
    thrusters[4].pin = CS5_Pin;

    thrusters[5].velocity = THRUSTER_PWM_NEUTRAL;
    thrusters[5].htim = &htim2;
    thrusters[5].chanel = TIM_CHANNEL_4;
    thrusters[5].need_update = true;
    thrusters[5].enablead = true;
    thrusters[5].port = CS6_GPIO_Port;
    thrusters[5].pin = CS6_Pin;

    thrusters[6].velocity = THRUSTER_PWM_NEUTRAL;
    thrusters[6].htim = &htim2;
    thrusters[6].chanel = TIM_CHANNEL_2;
    thrusters[6].need_update = true;
    thrusters[6].enablead = true;
    thrusters[6].port = CS7_GPIO_Port;
    thrusters[6].pin = CS7_Pin;

    thrusters[7].velocity = THRUSTER_PWM_NEUTRAL;
    thrusters[7].htim = &htim2;
    thrusters[7].chanel = TIM_CHANNEL_1;
    thrusters[7].need_update = true;
    thrusters[7].enablead = true;
    thrusters[7].port = CS8_GPIO_Port;
    thrusters[7].pin = CS8_Pin;

    thrusters[0].address = 2;
    thrusters[1].address = 0;
    thrusters[2].address = 4;
    thrusters[3].address = 7;
    thrusters[4].address = 6;
    thrusters[5].address = 5;
    thrusters[6].address = 3;
    thrusters[7].address = 1;

};

void init_thrusters(){
    init_thrusters_data();
    thrusters_update();
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
}
