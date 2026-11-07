/*
 * RMD_Motor.h
 *
 *  Created on: Aug 31, 2026
 *      Author: Antigravity AI
 *
 *  RMD Motor CAN/FDCAN Driver for STM32 (MyActuator RMD X6-S2 / RMD series)
 */

#ifndef _RMD_MOTOR_H_
#define _RMD_MOTOR_H_

/* ================== Includes ================== */
#include "stm32h7xx_hal.h"
#include "stdint.h"
#include "fdcan.h"
#include "Motor.h" // For CAN_Bus_t

/* ================== Macros ================== */
#define RMD_MOTOR_ID_BASE       0x141 // Default Command CAN ID for RMD Motor 1
#define RMD_MOTOR_REPLY_ID_BASE 0x241 // Reply CAN ID for RMD V3 Motors (0x240 + ID)
#define RMD_MAX_MOTORS          8     // Supports Motors 1 through 8

/* RMD Command Protocols */
#define RMD_CAN_ID_CONFIG_ADDR 0x300 // Broadcast address for 0x79 CAN ID setting command
#define RMD_CMD_RELEASE_BRAKE  0x77
#define RMD_CMD_SET_CAN_ID     0x79
#define RMD_CMD_SHUTDOWN       0x80
#define RMD_CMD_STOP          0x81
#define RMD_CMD_ENABLE        0x88
#define RMD_CMD_READ_ANGLE    0x92
#define RMD_CMD_READ_STATUS2  0x9C
#define RMD_CMD_TORQUE_CTRL   0xA1
#define RMD_CMD_SPEED_CTRL    0xA2
#define RMD_CMD_POS_CTRL_1    0xA3
#define RMD_CMD_POS_CTRL_2    0xA4
#define RMD_CMD_POS_CTRL_INC  0xA8

/* ================== Structs ================== */
typedef struct {
    int8_t   temperature;           // Motor temperature (°C)
    int16_t  feedback_current;      // Torque current feedback (iq value)
    int16_t  feedback_speed;        // Speed feedback (dps)
    uint16_t feedback_encoder;      // Encoder raw position value
    int32_t  multi_turn_angle_raw;  // Multi-turn angle raw (0.01 deg/LSB)
    float    multi_turn_angle_deg;  // Multi-turn angle in degrees
    uint8_t  angle_valid;           // 1 when 0x92 multi-turn angle has been received
    uint32_t last_update_tick;      // Timestamp of last feedback frame received
} RMD_Feedback_t;

/* ================== Function Prototypes ================== */

/**
 * @brief Enable/Activate RMD Motor (Command 0x88)
 */
HAL_StatusTypeDef RMD_EnableMotor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);

/**
 * @brief Disable/Power off RMD Motor (Command 0x80)
 */
HAL_StatusTypeDef RMD_DisableMotor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);

/**
 * @brief Stop RMD Motor movement (Command 0x81)
 */
HAL_StatusTypeDef RMD_StopMotor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);

/**
 * @brief Release RMD Motor brake (Command 0x77)
 */
HAL_StatusTypeDef RMD_ReleaseBrake(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);

/**
 * @brief Write new Motor ID (1..32) to RMD Motor ROM (Command 0x79)
 * @note  Connect ONLY ONE motor to the CAN bus when calling this, then power-cycle the motor.
 */
HAL_StatusTypeDef RMD_SetMotorID(FDCAN_HandleTypeDef *hfdcan, uint8_t new_id);

/**
 * @brief Request Multi-turn Angle Feedback (Command 0x92)
 */
HAL_StatusTypeDef RMD_RequestMultiTurnAngle(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);

/**
 * @brief Send Speed Closed-Loop Control Command (Command 0xA2)
 * @param speed_0_01dps Speed in 0.01 deg/s (e.g. 36000 = 360 dps)
 */
HAL_StatusTypeDef RMD_SetSpeed(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, int32_t speed_0_01dps);

/**
 * @brief Send Multi-turn Position Closed-Loop Control Command 1 (Command 0xA3)
 * @param position_0_01deg Target angle position in 0.01 deg (e.g. 36000 = 360 degrees)
 */
HAL_StatusTypeDef RMD_SetPositionDirect(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, int32_t position_0_01deg);

/**
 * @brief Send Multi-turn Position Closed-Loop Control Command 2 with Speed Limit (Command 0xA4)
 * @param max_speed_dps Maximum speed limit in 1 dps (e.g. 500)
 * @param position_0_01deg Target angle position in 0.01 deg (e.g. 36000 = 360 degrees)
 */
HAL_StatusTypeDef RMD_SetPosition(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint16_t max_speed_dps, int32_t position_0_01deg);

/**
 * @brief Send Incremental Position Closed-Loop Control Command with Speed Limit (Command 0xA8)
 * @param max_speed_dps Maximum speed limit in 1 dps (e.g. 500)
 * @param angle_inc_0_01deg Incremental angle in 0.01 deg (e.g. 9000 = +90 degrees)
 */
HAL_StatusTypeDef RMD_SetPositionIncremental(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint16_t max_speed_dps, int32_t angle_inc_0_01deg);

/**
 * @brief Send Torque / Current Control Command (Command 0xA1)
 * @param current_iq Target current value (int16_t)
 */
HAL_StatusTypeDef RMD_SetTorque(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, int16_t current_iq);

/**
 * @brief Parse incoming CAN response frame from RMD motor
 */
void RMD_ProcessFeedback(CAN_Bus_t bus, uint16_t can_id, const uint8_t *rxData);

/**
 * @brief Get feedback structure pointer for a specific motor
 * @param motor_index Index from 0 to (RMD_MAX_MOTORS - 1), where 0 corresponds to 0x141
 */
RMD_Feedback_t* RMD_GetFeedback(CAN_Bus_t bus, uint8_t motor_index);

/**
 * @brief Check if feedback data is fresh
 */
uint8_t RMD_IsFeedbackFresh(CAN_Bus_t bus, uint8_t motor_index, uint32_t timeout_ms);

#endif /* _RMD_MOTOR_H_ */
