#include "main.h"
#include <communication.h>
#include "Thruster.h"
#include "rov.h"


extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern rov_t rov_state;
extern thruster_t thrusters[THRUSTERS_NUM];

volatile uint16_t adc[16] = {0,};


void adc_init()
{
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_Delay(1000);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 15);

}

void trans_adc()
{
	thrusters[1].current=adc[1];
	thrusters[2].current=adc[2];
	thrusters[3].current=adc[3];
	thrusters[4].current=adc[4];
	thrusters[5].current=adc[5];
	thrusters[6].current=adc[6];
	thrusters[7].current=adc[7];

	rov_state.current_light = adc[8];
	rov_state.current_logic = adc[9];

	rov_state.vol_bat_cell[0] = adc[10];
	rov_state.vol_bat_cell[1] = adc[11];
	rov_state.vol_bat_cell[2] = adc[12];
	rov_state.vol_bat_cell[3] = adc[13];
	rov_state.temperature = adc[14];

}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->ErrorCode == HAL_ADC_ERROR_OVR)
  {
	    HAL_ADC_Stop_DMA(&hadc1);
	    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 15);
  }
  else if(hadc->ErrorCode == HAL_ADC_ERROR_DMA)
  {
	    HAL_ADC_Stop_DMA(&hadc1);
	    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 15);
  }
}
