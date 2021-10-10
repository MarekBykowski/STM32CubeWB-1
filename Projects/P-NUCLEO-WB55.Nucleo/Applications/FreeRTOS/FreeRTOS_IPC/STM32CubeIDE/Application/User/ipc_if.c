#include "ipc_api.h"
#include "ipc_if.h"
#include "stm32wbxx_hal.h"
#include <string.h>
#include "cmsis_os.h"

#define IPC_UART_TX_READY_EVT           (1 << 0)                    /** Data ready event flag */
#define IPC_UART_TASK_STACK             (256U)                      /** Size of the uart rx task stack */
#define IPC_UART_TASK_PRIO              (tskIDLE_PRIORITY + 1U)     /** Tx task priority */

static EventGroupHandle_t ipcUartEvt;
static UART_HandleTypeDef ipcUartHandle;
static DMA_HandleTypeDef  ipcUartDmaTx;
static DMA_HandleTypeDef  ipcUartDmaRx;
static TaskHandle_t       ipcUartTaskHandle = NULL;

static uint8_t dmaRxBuf[2][IPC_PACKET_LEN];
static uint8_t dmaRxTmpBuf[IPC_PACKET_LEN];
static uint8_t *dmaReadyBufPtr;
static uint8_t *dmaActiveBufPtr;
static uint8_t dmaTxBuffer[IPC_PACKET_LEN];

/**
  * \brief This function handles DMA2 channel3 global interrupt.
  */
void DMA2_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&ipcUartDmaTx);
}

/**
  * \brief This function handles DMA2 channel5 global interrupt.
  */
void DMA2_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&ipcUartDmaRx);
}

/**
  * \brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&ipcUartHandle);
}

/**
 * \brief IPC Uart task handler function
 *
 * \param [in]    param Optional argument passed to this handler
 */
static void ipcUartTask(void *param)
{
    uint32_t dataLen = 0U;
    BaseType_t res;
    (void) param;

    while (1) {
        res = xTaskNotifyWait(0x00, 0x00, &dataLen, portMAX_DELAY);
        if (res == pdTRUE) {
            ipcProcessReceivedData(dmaReadyBufPtr, dataLen);
        }
    }
}

/**
 * \brief Initialization of the IPC Uart
 *
 * \param [in]    baudrate Uart speed
 * \return                 0 on success negative value in case of error
 */
int ipcUartInit(int baudrate)
{
    ipcUartHandle.Instance = USART1;
    ipcUartHandle.Init.BaudRate = baudrate;
    ipcUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    ipcUartHandle.Init.StopBits = UART_STOPBITS_1;
    ipcUartHandle.Init.Parity = UART_PARITY_NONE;
    ipcUartHandle.Init.Mode = UART_MODE_TX_RX;
    ipcUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    ipcUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    ipcUartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    ipcUartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&ipcUartHandle) != HAL_OK)
    {
        return -1;
    }

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Rx DMA configuration */
    ipcUartDmaRx.Instance = DMA2_Channel5;
    ipcUartDmaRx.Init.Request = DMA_REQUEST_USART1_RX;
    ipcUartDmaRx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    ipcUartDmaRx.Init.PeriphInc = DMA_PINC_DISABLE;
    ipcUartDmaRx.Init.MemInc = DMA_MINC_ENABLE;
    ipcUartDmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    ipcUartDmaRx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    ipcUartDmaRx.Init.Mode = DMA_CIRCULAR;
    ipcUartDmaRx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&ipcUartDmaRx) != HAL_OK)
    {
        return -2;
    }
    __HAL_LINKDMA(&ipcUartHandle, hdmarx, ipcUartDmaRx);

    /* Tx DMA configuration */
    ipcUartDmaTx.Instance = DMA2_Channel3;
    ipcUartDmaTx.Init.Request = DMA_REQUEST_USART1_TX;
    ipcUartDmaTx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    ipcUartDmaTx.Init.PeriphInc = DMA_PINC_DISABLE;
    ipcUartDmaTx.Init.MemInc = DMA_MINC_ENABLE;
    ipcUartDmaTx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    ipcUartDmaTx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    ipcUartDmaTx.Init.Mode = DMA_NORMAL;
    ipcUartDmaTx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&ipcUartDmaTx) != HAL_OK)
    {
        return -3;
    }

    __HAL_LINKDMA(&ipcUartHandle, hdmatx, ipcUartDmaTx);

    /* DMA2_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5U, 1U);
    HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

    /* DMA2_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    ipcUartEvt = xEventGroupCreate();
    xEventGroupSetBits(ipcUartEvt, IPC_UART_TX_READY_EVT);

    xTaskCreate(ipcUartTask, "ipc_uart", IPC_UART_TASK_STACK, NULL, IPC_UART_TASK_PRIO, &ipcUartTaskHandle);
    if (ipcUartTaskHandle == 0U) {
        return -4;
    }

    /* Start lisining for incoming messages */
    dmaActiveBufPtr = dmaRxBuf[0];
    dmaReadyBufPtr = dmaRxBuf[1];
    if (HAL_UARTEx_ReceiveToIdle_DMA(&ipcUartHandle, dmaRxTmpBuf, IPC_PACKET_LEN) != HAL_OK) {
      return -5;
    }

    return 0;
}

/**
 * \brief Uart Tx complete callback handler
 *
 * \param [in]    uart_handle Uart handle
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart_handle)
{
    BaseType_t taskWoken = pdFALSE;
    BaseType_t res;

    res = xEventGroupSetBitsFromISR(ipcUartEvt, IPC_UART_TX_READY_EVT, &taskWoken);
    if(res == pdPASS) {
        portYIELD_FROM_ISR(taskWoken);
    }
}

/**
 * \brief Send IPC packet over Uart
 *
 * \param [in]    packet Pointer to the ipc packet
 * \return               0 on success negative value in case of error
 */
int ipcUartSendData(ipcPacket_t *packet)
{
    HAL_StatusTypeDef res;

    xEventGroupWaitBits(ipcUartEvt, IPC_UART_TX_READY_EVT, pdFALSE, pdFALSE, portMAX_DELAY);

    (void) memcpy(dmaTxBuffer, (uint8_t *) packet, IPC_PACKET_LEN);

    res = HAL_UART_Transmit_DMA(&ipcUartHandle, dmaTxBuffer, IPC_PACKET_LEN);
    if (res != HAL_OK) {
        return -res;
    }

    xEventGroupClearBits(ipcUartEvt, IPC_UART_TX_READY_EVT);

    return 0;
}

/**
  * \brief  This function handles buffer containing received data
  * \note   This routine is executed in Interrupt context.
  * \param [in]    huart UART handle.
  * \param [in]    data  Pointer on received data buffer to be processed
  * \param [in]    len   Nb of received characters available in buffer
  */
void bufferDataReady(UART_HandleTypeDef *huart, uint8_t* data, uint16_t len)
{
    /*
     * This function might be called in any of the following interrupt contexts :
     *  - DMA TC and HT events
     *  - UART IDLE line event
     *
     * data and len defines the buffer where received data have been copied, in order to be processed.
     * During this processing of already received data, reception is still ongoing.
     */
    BaseType_t taskWoken = pdFALSE;
    xTaskNotifyFromISR(ipcUartTaskHandle, len, eSetBits, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
}

/**
  * \brief  User implementation of the Reception Event Callback
  *         (Rx event notification called after use of advanced reception service).
  * \param [in]    huart UART handle
  * \param [in]    size  Number of data available in application reception buffer (indicates a position in
  *                      reception buffer until which, data are available)
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    static uint16_t oldSize = 0;
    uint8_t *ptemp;
    uint16_t i;
    uint16_t receivedData = 0;

    /* Check if number of received data in recpetion buffer has changed */
    if (size != oldSize) {
        if (size > oldSize) {
            receivedData = size - oldSize;
            /* Copy received data in active buffer */
            for (i = 0U; i < receivedData; i++) {
                dmaActiveBufPtr[oldSize + i] = dmaRxTmpBuf[oldSize + i];
            }
        }

        oldSize = size;

        if (size == IPC_PACKET_LEN) {
            /* Swap buffers for next bytes to be processed */
            ptemp = dmaReadyBufPtr;
            dmaReadyBufPtr = dmaActiveBufPtr;
            dmaActiveBufPtr = ptemp;

            /* Data complete, prepare for new packet */
            oldSize = 0;

            /* Process received data that has been extracted from Rx User buffer */
            bufferDataReady(huart, dmaReadyBufPtr, size);
        }
    }
}
