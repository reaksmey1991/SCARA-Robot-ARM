/*
 * RMD_Motor.c
 *
 *  Created on: Aug 31, 2026
 *      Author: Antigravity AI
 *
 *  RMD Motor CAN/FDCAN Driver for STM32 (MyActuator RMD X6-S2 / RMD series)
 */

#include "RMD_Motor.h"

/* ================== Variables ================== */
static RMD_Feedback_t rmd_feedback[CAN_BUS_COUNT][RMD_MAX_MOTORS] = {0};

/* Helper function to transmit FDCAN frame */
static HAL_StatusTypeDef RMD_SendFrame(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, const uint8_t data[8])
{
	if (!hfdcan) return HAL_ERROR;
	if (hfdcan->State == HAL_FDCAN_STATE_RESET || hfdcan->State == HAL_FDCAN_STATE_ERROR) {
		return HAL_ERROR;
	}

	FDCAN_TxHeaderTypeDef txHeader;
	txHeader.Identifier = motor_id;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0;

	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, (uint8_t*)data);
}

/* Enable Motor Command (0x88) */
HAL_StatusTypeDef RMD_EnableMotor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
	uint8_t data[8] = {RMD_CMD_ENABLE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Disable Motor Command (0x80) */
HAL_StatusTypeDef RMD_DisableMotor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
	uint8_t data[8] = {RMD_CMD_SHUTDOWN, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Stop Motor Command (0x81) */
HAL_StatusTypeDef RMD_StopMotor(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
	uint8_t data[8] = {RMD_CMD_STOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Release Brake Command (0x77) */
HAL_StatusTypeDef RMD_ReleaseBrake(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
	uint8_t data[8] = {RMD_CMD_RELEASE_BRAKE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Set Motor CAN ID Command (0x79) - Connect ONLY ONE motor on the bus! */
HAL_StatusTypeDef RMD_SetMotorID(FDCAN_HandleTypeDef *hfdcan, uint8_t new_id)
{
	if (new_id < 1 || new_id > 32) return HAL_ERROR;
	uint8_t data[8] = {RMD_CMD_SET_CAN_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, new_id};
	/* Send to standard 0x300 ID-config address and 0x141 default motor address */
	RMD_SendFrame(hfdcan, RMD_CAN_ID_CONFIG_ADDR, data);
	return RMD_SendFrame(hfdcan, RMD_MOTOR_ID_BASE, data);
}

/* Request Multi-turn Angle Command (0x92) */
HAL_StatusTypeDef RMD_RequestMultiTurnAngle(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
	uint8_t data[8] = {RMD_CMD_READ_ANGLE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Speed Closed-Loop Control Command (0xA2) */
HAL_StatusTypeDef RMD_SetSpeed(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, int32_t speed_0_01dps)
{
	uint8_t data[8] = {
		RMD_CMD_SPEED_CTRL,
		0x00, 0x00, 0x00,
		(uint8_t)(speed_0_01dps & 0xFF),
		(uint8_t)((speed_0_01dps >> 8) & 0xFF),
		(uint8_t)((speed_0_01dps >> 16) & 0xFF),
		(uint8_t)((speed_0_01dps >> 24) & 0xFF)
	};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Position Closed-Loop Control Command 1 without Speed Limit (0xA3) */
HAL_StatusTypeDef RMD_SetPositionDirect(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, int32_t position_0_01deg)
{
	uint8_t data[8] = {
		RMD_CMD_POS_CTRL_1,
		0x00, 0x00, 0x00,
		(uint8_t)(position_0_01deg & 0xFF),
		(uint8_t)((position_0_01deg >> 8) & 0xFF),
		(uint8_t)((position_0_01deg >> 16) & 0xFF),
		(uint8_t)((position_0_01deg >> 24) & 0xFF)
	};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Position Closed-Loop Control Command 2 with Speed Limit (0xA4) */
HAL_StatusTypeDef RMD_SetPosition(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint16_t max_speed_dps, int32_t position_0_01deg)
{
	uint8_t data[8] = {
		RMD_CMD_POS_CTRL_2,
		0x00,
		(uint8_t)(max_speed_dps & 0xFF),
		(uint8_t)((max_speed_dps >> 8) & 0xFF),
		(uint8_t)(position_0_01deg & 0xFF),
		(uint8_t)((position_0_01deg >> 8) & 0xFF),
		(uint8_t)((position_0_01deg >> 16) & 0xFF),
		(uint8_t)((position_0_01deg >> 24) & 0xFF)
	};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Incremental Position Closed-Loop Control Command with Speed Limit (0xA8) */
HAL_StatusTypeDef RMD_SetPositionIncremental(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint16_t max_speed_dps, int32_t angle_inc_0_01deg)
{
	uint8_t data[8] = {
		RMD_CMD_POS_CTRL_INC,
		0x00,
		(uint8_t)(max_speed_dps & 0xFF),
		(uint8_t)((max_speed_dps >> 8) & 0xFF),
		(uint8_t)(angle_inc_0_01deg & 0xFF),
		(uint8_t)((angle_inc_0_01deg >> 8) & 0xFF),
		(uint8_t)((angle_inc_0_01deg >> 16) & 0xFF),
		(uint8_t)((angle_inc_0_01deg >> 24) & 0xFF)
	};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Torque / Current Control Command (0xA1) */
HAL_StatusTypeDef RMD_SetTorque(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, int16_t current_iq)
{
	uint8_t data[8] = {
		RMD_CMD_TORQUE_CTRL,
		0x00, 0x00, 0x00,
		(uint8_t)(current_iq & 0xFF),
		(uint8_t)((current_iq >> 8) & 0xFF),
		0x00, 0x00
	};
	return RMD_SendFrame(hfdcan, motor_id, data);
}

/* Parse incoming RMD feedback frame */
void RMD_ProcessFeedback(CAN_Bus_t bus, uint16_t can_id, const uint8_t *rxData)
{
	if (!rxData || bus >= CAN_BUS_COUNT) return;
	if (can_id < RMD_MOTOR_ID_BASE || can_id >= (RMD_MOTOR_ID_BASE + RMD_MAX_MOTORS)) return;

	uint8_t idx = can_id - RMD_MOTOR_ID_BASE;
	RMD_Feedback_t *fb = &rmd_feedback[bus][idx];
	uint8_t cmd = rxData[0];

	if (cmd == RMD_CMD_READ_ANGLE) {
		int32_t raw_angle = 0;
		if (rxData[1] != 0 || rxData[2] != 0 || rxData[3] != 0) {
			// RMD V2 format: bytes 1..7 (0.01 deg/LSB, Little-Endian)
			raw_angle = (int32_t)(
				((uint32_t)rxData[1]) |
				((uint32_t)rxData[2] << 8) |
				((uint32_t)rxData[3] << 16) |
				((uint32_t)rxData[4] << 24)
			);
		} else {
			// RMD V3 format: bytes 4..7 (0.01 deg/LSB, Little-Endian)
			raw_angle = (int32_t)(
				((uint32_t)rxData[4]) |
				((uint32_t)rxData[5] << 8) |
				((uint32_t)rxData[6] << 16) |
				((uint32_t)rxData[7] << 24)
			);
		}
		fb->multi_turn_angle_raw = raw_angle;
		fb->multi_turn_angle_deg = (float)raw_angle * 0.01f;
		fb->angle_valid          = 1;
	} else if (cmd == RMD_CMD_TORQUE_CTRL || cmd == RMD_CMD_SPEED_CTRL ||
	           cmd == RMD_CMD_POS_CTRL_1  || cmd == RMD_CMD_POS_CTRL_2 ||
	           cmd == RMD_CMD_POS_CTRL_INC || cmd == RMD_CMD_READ_STATUS2) {
		// Byte 1: Temperature int8_t
		fb->temperature = (int8_t)rxData[1];

		// Bytes 2-3: Torque current int16_t (Little-Endian)
		fb->feedback_current = (int16_t)(rxData[2] | (rxData[3] << 8));

		// Bytes 4-5: Speed int16_t (Little-Endian)
		fb->feedback_speed = (int16_t)(rxData[4] | (rxData[5] << 8));

		// Bytes 6-7: Encoder uint16_t (Little-Endian)
		fb->feedback_encoder = (uint16_t)(rxData[6] | (rxData[7] << 8));
	}

	fb->last_update_tick = HAL_GetTick();
}

RMD_Feedback_t* RMD_GetFeedback(CAN_Bus_t bus, uint8_t motor_index)
{
	if (bus >= CAN_BUS_COUNT || motor_index >= RMD_MAX_MOTORS) return NULL;
	return &rmd_feedback[bus][motor_index];
}

uint8_t RMD_IsFeedbackFresh(CAN_Bus_t bus, uint8_t motor_index, uint32_t timeout_ms)
{
	if (bus >= CAN_BUS_COUNT || motor_index >= RMD_MAX_MOTORS) return 0;
	uint32_t last = rmd_feedback[bus][motor_index].last_update_tick;
	return ((HAL_GetTick() - last) <= timeout_ms) ? 1 : 0;
}
