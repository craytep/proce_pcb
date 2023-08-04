/*
 * messages.h
 *
 *  Created on: Aug 3, 2023
 *      Author: Dimitrius
 */

#ifndef INC_MESSAGES_H_
#define INC_MESSAGES_H_

#define RPI_REQUEST_LENGTH               27
#define RPI_RESPONSE_LENGTH              30
#define LIGHT_RESPONSE_LENGTH            7


struct RPIRequest
{
	uint8_t A5;
	uint8_t con_state;
	uint8_t flags; // [0]thrusters_on, [1]rgb_light_on, [2]lower_light_on,

	uint16_t velocity[8]; // pwm to thrusters 8 pcs
	uint16_t tilt;
	uint8_t Light_pw[4]; // power light first-> mono; 234 -> rgb
	uint16_t crc;
};

struct RPIResponse
{
	uint8_t A5;
	uint8_t con_state;

	uint16_t current_logic;
	uint16_t current_vma[8];
	uint16_t vol_bat_cell[4];
	uint16_t crc;
};

struct STMResponse
{
	uint8_t A5;
	uint8_t Light_pw[4]; // power light first-> mono; 234 -> rgb
	uint16_t crc;
};


#endif /* INC_MESSAGES_H_ */
