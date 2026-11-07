/*
 * Motor.c
 *
 * FDCAN Interrupt Callback for RMD Motor Feedback Processing.
 */

#include "Motor.h"
#include "RMD_Motor.h"
#include "MKS_Servo.h"
#include "fdcan.h"

extern volatile uint16_t mks_can_id;

/* Diagnostic variables viewable in Live Expressions to see raw motor replies */
volatile uint32_t last_rx_can_id = 0;
volatile uint8_t  last_rx_cmd    = 0;
volatile uint8_t  last_rx_data[8] = {0};
volatile uint8_t  last_rx_bus    = 0; /* 1 = FDCAN1, 2 = FDCAN2, 3 = FDCAN3 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		FDCAN_RxHeaderTypeDef rxHeader;
		uint8_t rxData[8];

		/* Drain all pending messages in RX FIFO0 so simultaneous M1 & M2 replies are never missed */
		while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
				if (rxHeader.IdType == FDCAN_STANDARD_ID) {
					last_rx_can_id = rxHeader.Identifier;
					last_rx_cmd    = rxData[0];
					last_rx_bus    = (hfdcan->Instance == FDCAN1) ? 1 : ((hfdcan->Instance == FDCAN2) ? 2 : 3);
					for (int i = 0; i < 8; i++) {
						last_rx_data[i] = rxData[i];
					}

					CAN_Bus_t bus = (hfdcan->Instance == FDCAN1) ? CAN_BUS_I : CAN_BUS_II;

					/* Route MKS SERVO57D replies (ID 0x01..0x7F on FDCAN1/2/3) */
					if (rxHeader.Identifier >= 0x01u && rxHeader.Identifier <= 0x7Fu)
					{
						MKS_ProcessRxFrame(rxHeader.Identifier, rxHeader.DataLength, rxData);
					}
					/* Support both 0x141..0x148 (RMD V1/V2) and 0x241..0x248 (RMD V3) reply IDs */
					else if (rxHeader.Identifier >= RMD_MOTOR_ID_BASE &&
					         rxHeader.Identifier < (RMD_MOTOR_ID_BASE + RMD_MAX_MOTORS))
					{
						RMD_ProcessFeedback(bus, rxHeader.Identifier, rxData);
					}
					else if (rxHeader.Identifier >= RMD_MOTOR_REPLY_ID_BASE &&
					         rxHeader.Identifier < (RMD_MOTOR_REPLY_ID_BASE + RMD_MAX_MOTORS))
					{
						uint16_t normalized_id = (rxHeader.Identifier - RMD_MOTOR_REPLY_ID_BASE) + RMD_MOTOR_ID_BASE;
						RMD_ProcessFeedback(bus, normalized_id, rxData);
					}
				}
			} else {
				break;
			}
		}
	}
}
