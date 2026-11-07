/*
 * MKS_Servo.h
 *
 *  MKS SERVO57D / SERVO42D V1.3 CAN Driver for STM32 FDCAN2
 */

#ifndef _MKS_SERVO_H_
#define _MKS_SERVO_H_

#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include <stdint.h>

/* ================== Macros ================== */
#define MKS_DEFAULT_ID          0x01u    // Default MKS #1 slave CAN ID (0x01..0x7F)
#define MSK2_DEFAULT_ID         0x02u    // Default MSK2_stepper slave CAN ID (0x02)
#define MKS_MAX_RPM             3000u    // Maximum speed in RPM
#define MKS_ENC_COUNTS_PER_REV  16384.0f // Encoder resolution per revolution (14-bit)
#define MKS_INT24_MAX           8388607
#define MKS_INT24_MIN           (-8388608)

/* MKS SERVO57D Command Opcodes */
#define MKS_CMD_READ_ENC        0x31u    // Read accumulated encoder value (int48)
#define MKS_CMD_RELEASE_PROT    0x3Du    // Release stall protection
#define MKS_CMD_SET_DIR         0x86u    // Set positive direction of motor rotation (0 = CW, 1 = CCW)
#define MKS_CMD_SET_ZERO        0x92u    // Set current position as zero
#define MKS_CMD_ENABLE          0xF3u    // Enable (1) / Disable (0) motor
#define MKS_CMD_REL_AXIS        0xF4u    // Relative move by axis counts
#define MKS_CMD_ABS_AXIS        0xF5u    // Absolute move by axis counts
#define MKS_CMD_SPEED           0xF6u    // Continuous speed mode
#define MKS_CMD_ESTOP           0xF7u    // Emergency stop
#define MKS_CMD_REL_PULSES      0xFDu    // Relative move by pulses (microsteps)
#define MKS_CMD_ABS_PULSES      0xFEu    // Absolute move by pulses (microsteps)

/* ================== Enums ================== */

/* Which action to run when mks_cmd_send_flag is set in Live Expressions */
typedef enum {
	MKS_ACTION_NONE = 0,
	MKS_ACTION_ENABLE,          // 1  enable motor (0xF3)
	MKS_ACTION_DISABLE,         // 2  disable motor (0xF3)
	MKS_ACTION_SPEED_RUN,       // 3  continuous speed mode (0xF6)
	MKS_ACTION_SPEED_STOP,      // 4  stop speed mode (0xF6 speed 0, uses mks_acc)
	MKS_ACTION_MOVE_REL_PULSES, // 5  relative move in pulses (0xFD)
	MKS_ACTION_MOVE_ABS_PULSES, // 6  absolute move in pulses (0xFE)
	MKS_ACTION_MOVE_REL_DEG,    // 7  relative move in degrees (0xF4)
	MKS_ACTION_MOVE_ABS_DEG,    // 8  absolute move in degrees (0xF5)
	MKS_ACTION_ESTOP,           // 9  emergency stop (0xF7)
	MKS_ACTION_SET_ZERO,        // 10 set current position as zero (0x92)
	MKS_ACTION_RELEASE_PROTECT, // 11 release stall protection (0x3D)
	MKS_ACTION_READ_ENCODER     // 12 read encoder once (0x31)
} MKS_Action_t;

/* Result of the last position command (from driver reply) */
typedef enum {
	MKS_MOVE_IDLE = 0,
	MKS_MOVE_STARTED,   // driver replied status 1
	MKS_MOVE_DONE,      // driver replied status 2
	MKS_MOVE_FAILED,    // driver replied status 0
	MKS_MOVE_LIMIT      // driver replied status 3 (stopped by limit)
} MKS_MoveStatus_t;

/* ================== Extern Feedback Variables ================== */
extern volatile uint16_t         mks_can_id;
extern volatile uint16_t         msk2_can_id;

/* MKS #1 (Calibrated with PD4) Feedback */
extern volatile uint32_t         mks_rx_msg_count;
extern volatile uint32_t         mks_rx_crc_errors;
extern volatile uint32_t         mks_tx_err_count;
extern volatile MKS_MoveStatus_t mks_move_status;
extern volatile uint8_t          mks_motor_enabled;
extern volatile uint8_t          mks_speed_cmd_ok;
extern volatile int64_t          mks_enc_value;
extern volatile float            mks_enc_deg;

/* MSK2_stepper (No Calibration) Feedback */
extern volatile uint32_t         msk2_rx_msg_count;
extern volatile MKS_MoveStatus_t msk2_move_status;
extern volatile uint8_t          msk2_motor_enabled;
extern volatile int64_t          msk2_enc_value;
extern volatile float            msk2_enc_deg;

/* ================== Function Prototypes ================== */
HAL_StatusTypeDef MKS_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t enable);
HAL_StatusTypeDef MKS_SetSpeed(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t dir, uint16_t speed_rpm, uint8_t acc);
HAL_StatusTypeDef MKS_StopSpeed(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t acc);
HAL_StatusTypeDef MKS_EmergencyStop(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id);
HAL_StatusTypeDef MKS_MoveRelPulses(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t dir, uint16_t speed_rpm, uint8_t acc, uint32_t pulses);
HAL_StatusTypeDef MKS_MoveAbsPulses(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint16_t speed_rpm, uint8_t acc, int32_t pulses);
HAL_StatusTypeDef MKS_MoveRelDeg(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint16_t speed_rpm, uint8_t acc, float deg);
HAL_StatusTypeDef MKS_MoveAbsDeg(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint16_t speed_rpm, uint8_t acc, float deg);
HAL_StatusTypeDef MKS_ReadEncoder(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id);
HAL_StatusTypeDef MKS_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id);
HAL_StatusTypeDef MKS_SetDirection(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t dir);
HAL_StatusTypeDef MKS_ReleaseProtect(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id);

void MKS_ProcessRxFrame(uint32_t can_id, uint32_t dlc, const uint8_t *rxData);

#endif /* _MKS_SERVO_H_ */
