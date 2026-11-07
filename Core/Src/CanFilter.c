#include "CanFilter.h"
#include "fdcan.h"

void CAN_FilterConfig(void)
{
    FDCAN_FilterTypeDef f = {0};

    // Global: accept all non-matching to FIFO0 (standard & extended)
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE,
        FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE,
        FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_FILTER_REMOTE,
        FDCAN_FILTER_REMOTE);

    // Dedicated filter: accept ALL standard IDs -> FIFO0
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x000;
    f.FilterID2    = 0x000; // mask=0 => accept all

    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
    HAL_FDCAN_ConfigFilter(&hfdcan2, &f);
    HAL_FDCAN_ConfigFilter(&hfdcan3, &f);

    /* Ensure hardware Auto-Retransmission is enabled even if CubeMX regenerates fdcan.c */
    SET_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
    SET_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_CCE);
    CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_DAR);

    SET_BIT(hfdcan2.Instance->CCCR, FDCAN_CCCR_INIT);
    SET_BIT(hfdcan2.Instance->CCCR, FDCAN_CCCR_CCE);
    CLEAR_BIT(hfdcan2.Instance->CCCR, FDCAN_CCCR_DAR);

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan3);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
