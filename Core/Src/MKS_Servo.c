/*
 * MKS_Servo.c
 *
 *  MKS SERVO57D / SERVO42D V1.3 CAN Driver for STM32 FDCAN2
 */

#include "MKS_Servo.h"

/* ================== Feedback & Diagnostic Variables ================== */
/* MKS #1 Feedback */
volatile uint32_t         mks_rx_msg_count   = 0;
volatile uint32_t         mks_rx_crc_errors  = 0;
volatile uint32_t         mks_tx_err_count   = 0;
volatile MKS_MoveStatus_t mks_move_status    = MKS_MOVE_IDLE;
volatile uint8_t          mks_motor_enabled  = 0;
volatile uint8_t          mks_speed_cmd_ok   = 0;
volatile int64_t          mks_enc_value      = 0;    // Accumulated encoder value (16384 per rev)
volatile float            mks_enc_deg        = 0.0f; // Accumulated encoder angle in degrees

/* MSK2_stepper Feedback */
volatile uint32_t         msk2_rx_msg_count  = 0;
volatile MKS_MoveStatus_t msk2_move_status   = MKS_MOVE_IDLE;
volatile uint8_t          msk2_motor_enabled = 0;
volatile int64_t          msk2_enc_value     = 0;    // Accumulated encoder value (16384 per rev)
volatile float            msk2_enc_deg       = 0.0f; // Accumulated encoder angle in degrees

/* ================== Private Helpers ================== */
static const uint32_t dlc_tbl[9] = {
	FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
	FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
	FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
};

static uint8_t dlc_to_len(uint32_t dlc)
{
	for (uint8_t i = 0; i < 9; i++) {
		if (dlc_tbl[i] == dlc) return i;
	}
	return 0;
}

/* Sends [payload..., CRC]. payload length 1..7. CRC = (ID + payload bytes) & 0xFF */
static HAL_StatusTypeDef MKS_Send(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, const uint8_t *payload, uint8_t len)
{
	if (!hfdcan || len == 0 || len > 7) return HAL_ERROR;

	FDCAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	uint16_t sum = can_id;

	for (uint8_t i = 0; i < len; i++) {
		TxData[i] = payload[i];
		sum += payload[i];
	}
	TxData[len] = (uint8_t)(sum & 0xFF);

	TxHeader.Identifier = can_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = dlc_tbl[len + 1];
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData);
	if (st != HAL_OK) mks_tx_err_count++;
	return st;
}

static uint16_t clamp_speed(uint16_t speed)
{
	return (speed > MKS_MAX_RPM) ? MKS_MAX_RPM : speed;
}

static int32_t clamp_int24(int32_t v)
{
	if (v > MKS_INT24_MAX) return MKS_INT24_MAX;
	if (v < MKS_INT24_MIN) return MKS_INT24_MIN;
	return v;
}

static int32_t deg_to_counts(float deg)
{
	float c = deg * (MKS_ENC_COUNTS_PER_REV / 360.0f);
	return clamp_int24((int32_t)(c + ((c >= 0.0f) ? 0.5f : -0.5f)));
}

/* ================== Public API ================== */

// Command 0xF3: enable (1) / disable (0) the motor
HAL_StatusTypeDef MKS_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t enable)
{
	uint8_t p[2] = { MKS_CMD_ENABLE, (uint8_t)(enable ? 1 : 0) };
	return MKS_Send(hfdcan, can_id, p, 2);
}

// Command 0xF6: Run motor in continuous Speed Mode (dir: 0 = CW, 1 = CCW | speed: 0..3000 RPM | acc: 0..255)
HAL_StatusTypeDef MKS_SetSpeed(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t dir, uint16_t speed_rpm, uint8_t acc)
{
	uint8_t p[4];
	speed_rpm = clamp_speed(speed_rpm);

	p[0] = MKS_CMD_SPEED;
	p[1] = ((dir & 0x01) << 7) | ((speed_rpm >> 8) & 0x0F);
	p[2] = speed_rpm & 0xFF;
	p[3] = acc;
	return MKS_Send(hfdcan, can_id, p, 4);
}

// Stop speed mode with deceleration (speed 0)
HAL_StatusTypeDef MKS_StopSpeed(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t acc)
{
	return MKS_SetSpeed(hfdcan, can_id, 0, 0, acc);
}

// Command 0xF7: emergency stop
HAL_StatusTypeDef MKS_EmergencyStop(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id)
{
	uint8_t p[1] = { MKS_CMD_ESTOP };
	return MKS_Send(hfdcan, can_id, p, 1);
}

// Command 0xFD: relative move by pulses (microsteps), unsigned 24-bit
HAL_StatusTypeDef MKS_MoveRelPulses(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t dir, uint16_t speed_rpm, uint8_t acc, uint32_t pulses)
{
	uint8_t p[7];
	speed_rpm = clamp_speed(speed_rpm);
	pulses &= 0xFFFFFF;

	p[0] = MKS_CMD_REL_PULSES;
	p[1] = ((dir & 0x01) << 7) | ((speed_rpm >> 8) & 0x0F);
	p[2] = speed_rpm & 0xFF;
	p[3] = acc;
	p[4] = (pulses >> 16) & 0xFF;
	p[5] = (pulses >> 8) & 0xFF;
	p[6] = pulses & 0xFF;
	return MKS_Send(hfdcan, can_id, p, 7);
}

// Command 0xFE: absolute move to pulse position (signed 24-bit, relative to zero)
HAL_StatusTypeDef MKS_MoveAbsPulses(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint16_t speed_rpm, uint8_t acc, int32_t pulses)
{
	uint8_t p[7];
	speed_rpm = clamp_speed(speed_rpm);
	uint32_t u = (uint32_t)clamp_int24(pulses) & 0xFFFFFF;

	p[0] = MKS_CMD_ABS_PULSES;
	p[1] = (speed_rpm >> 8) & 0x0F;
	p[2] = speed_rpm & 0xFF;
	p[3] = acc;
	p[4] = (u >> 16) & 0xFF;
	p[5] = (u >> 8) & 0xFF;
	p[6] = u & 0xFF;
	return MKS_Send(hfdcan, can_id, p, 7);
}

// Commands 0xF4 / 0xF5: move by encoder axis counts (16384 per rev), signed 24-bit
static HAL_StatusTypeDef MKS_MoveAxis(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t cmd, uint16_t speed_rpm, uint8_t acc, int32_t counts)
{
	uint8_t p[7];
	speed_rpm = clamp_speed(speed_rpm);
	uint32_t u = (uint32_t)clamp_int24(counts) & 0xFFFFFF;

	p[0] = cmd;
	p[1] = (speed_rpm >> 8) & 0x0F;
	p[2] = speed_rpm & 0xFF;
	p[3] = acc;
	p[4] = (u >> 16) & 0xFF;
	p[5] = (u >> 8) & 0xFF;
	p[6] = u & 0xFF;
	return MKS_Send(hfdcan, can_id, p, 7);
}

// Command 0xF4: Relative move in degrees
HAL_StatusTypeDef MKS_MoveRelDeg(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint16_t speed_rpm, uint8_t acc, float deg)
{
	return MKS_MoveAxis(hfdcan, can_id, MKS_CMD_REL_AXIS, speed_rpm, acc, deg_to_counts(deg));
}

// Command 0xF5: Absolute move in degrees (relative to driver's zero)
HAL_StatusTypeDef MKS_MoveAbsDeg(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint16_t speed_rpm, uint8_t acc, float deg)
{
	return MKS_MoveAxis(hfdcan, can_id, MKS_CMD_ABS_AXIS, speed_rpm, acc, deg_to_counts(deg));
}

// Command 0x31: read accumulated encoder value (reply: int48, 16384 counts per rev)
HAL_StatusTypeDef MKS_ReadEncoder(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id)
{
	uint8_t p[1] = { MKS_CMD_READ_ENC };
	return MKS_Send(hfdcan, can_id, p, 1);
}

// Command 0x92: set current position as zero
HAL_StatusTypeDef MKS_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id)
{
	uint8_t p[1] = { MKS_CMD_SET_ZERO };
	return MKS_Send(hfdcan, can_id, p, 1);
}

// Command 0x86: set positive rotation direction in driver NVM (0 = CW, 1 = CCW)
HAL_StatusTypeDef MKS_SetDirection(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t dir)
{
	uint8_t p[2] = { MKS_CMD_SET_DIR, (uint8_t)(dir ? 1 : 0) };
	return MKS_Send(hfdcan, can_id, p, 2);
}

// Command 0x3D: release stall protection
HAL_StatusTypeDef MKS_ReleaseProtect(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id)
{
	uint8_t p[1] = { MKS_CMD_RELEASE_PROT };
	return MKS_Send(hfdcan, can_id, p, 1);
}

// Parse incoming CAN reply from MKS SERVO57D (routes by CAN ID to MKS #1 or MSK2_stepper)
void MKS_ProcessRxFrame(uint32_t can_id, uint32_t dlc, const uint8_t *d)
{
	uint8_t len = dlc_to_len(dlc);
	if (len < 3 || !d) return;

	uint16_t sum = (uint16_t)can_id;
	for (uint8_t i = 0; i < len - 1; i++) {
		sum += d[i];
	}
	if ((uint8_t)(sum & 0xFF) != d[len - 1]) {
		mks_rx_crc_errors++;
		return;
	}

	uint8_t is_msk2 = ((uint16_t)can_id == msk2_can_id) ? 1u : 0u;
	if (is_msk2) {
		msk2_rx_msg_count++;
	} else {
		mks_rx_msg_count++;
	}

	switch (d[0]) {
	case MKS_CMD_ENABLE:
		if (is_msk2) msk2_motor_enabled = (d[1] == 1);
		else         mks_motor_enabled  = (d[1] == 1);
		break;

	case MKS_CMD_SPEED:
		if (!is_msk2) mks_speed_cmd_ok = (d[1] == 1);
		break;

	case MKS_CMD_REL_PULSES:
	case MKS_CMD_ABS_PULSES:
	case MKS_CMD_REL_AXIS:
	case MKS_CMD_ABS_AXIS:
	{
		MKS_MoveStatus_t st = MKS_MOVE_IDLE;
		if      (d[1] == 0) st = MKS_MOVE_FAILED;
		else if (d[1] == 1) st = MKS_MOVE_STARTED;
		else if (d[1] == 2) st = MKS_MOVE_DONE;
		else if (d[1] == 3) st = MKS_MOVE_LIMIT;
		if (is_msk2) msk2_move_status = st;
		else         mks_move_status  = st;
		break;
	}

	case MKS_CMD_READ_ENC:
		if (len == 8) {
			uint64_t v = 0;
			for (uint8_t i = 1; i <= 6; i++) {
				v = (v << 8) | d[i];
			}
			if (v & (1ULL << 47)) {
				v |= 0xFFFF000000000000ULL; // sign-extend int48
			}
			int64_t enc_val = (int64_t)v;
			float   enc_deg = (float)((double)enc_val * 360.0 / 16384.0);
			if (is_msk2) {
				msk2_enc_value = enc_val;
				msk2_enc_deg   = enc_deg;
			} else {
				mks_enc_value  = enc_val;
				mks_enc_deg    = enc_deg;
			}
		}
		break;

	default:
		break;
	}
}
