/*
 * Motor.h
 *
 * CAN Bus enumeration used by motor drivers.
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32h7xx_hal.h"
#include "stdint.h"

typedef enum {
	CAN_BUS_I = 0,
	CAN_BUS_II = 1,
	CAN_BUS_COUNT
} CAN_Bus_t;

#endif /* _MOTOR_H_ */