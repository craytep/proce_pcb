/*
 * rov.h
 *
 *  Created on: Aug 4, 2023
 *      Author: Dimitrius
 */

#ifndef INC_ROV_H_
#define INC_ROV_H_

typedef struct {
	uint8_t con_state;
	uint8_t flags;
	uint16_t tilt;
	uint8_t Light_pw[4]; // power light first-> mono; 234 -> rgb
	uint16_t current_logic;
	uint16_t current_light;
	uint16_t temperature;
	uint16_t vol_bat_cell[4];

} rov_t;

#endif /* INC_ROV_H_ */


