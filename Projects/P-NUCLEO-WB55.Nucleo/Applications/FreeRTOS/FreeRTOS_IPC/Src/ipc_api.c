#include "ipc_api.h"
#include "ipc_if.h"
#include <string.h>
#include "cmsis_os.h"

#define IPC_TX_TASK_STACK               (256U)              /** Size of the tx task stack */
#define IPC_RX_TASK_STACK               (256U)              /** Size of the rx task stack */
#define IPC_TX_TASK_PRIO                (tskIDLE_PRIORITY)  /** Tx task priority */
#define IPC_RX_TASK_PRIO                (tskIDLE_PRIORITY)  /** Rx task priority */
#define IPC_QUEUE_TIMEOUT               (1000)              /** Queue operation timeout */

static QueueHandle_t txCmdQueue;
static QueueHandle_t rxCmdQueue;
static TaskHandle_t  txTaskHandle = NULL;
static TaskHandle_t  rxTaskHandle = NULL;
static ipcRxCb_t     rxMsgCb = NULL;
static uint8_t       pktFrmCnt = 0;

/**
 * \brief Calculate packet payload checksum
 * 
 * \param  [in]    data Pointer to the payload data
 * \param  [in]    len  Length of the payload
 * \return              Payload checksum
 */
static uint8_t calculateChecksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 0U; i < len; i++) {
        checksum += data[i];
    }

    return checksum;
}

/**
 * \brief Check if received packet is valid
 * 
 * \param [in]    packet Pointer to the packet
 * \return               0 if valid negative value if not
 */
static int validatePacket(ipcPacket_t *packet)
{
    uint8_t checksum;

    checksum = calculateChecksum(packet->payload, packet->header.dataLen);
    if (checksum != packet->payloadChecksum) {
        return -1;
    }

    return 0;
}

/**
 * \brief IPC Tx task handler function
 * 
 * \param [in]    param Optional argument passed to this handler
 */
static void ipcTxTask(void *param)
{
    ipcPacket_t *packet = NULL;
    BaseType_t res;
    (void) param;

    while (1) {
        res = xQueueReceive(txCmdQueue, &packet, portMAX_DELAY);
        if (res == pdTRUE) {
            /* prepare packet and send it to the uart task */
            ipcUartSendData(packet);
        }
    }
}

/**
 * \brief IPC Rx task handler function
 * 
 * \param [in]    param Optional argument passed to this handler
 */
static void ipcRxTask(void *param)
{
    ipcPacket_t *packet = NULL;
    BaseType_t res;
    (void) param;

    while (1) {
        res = xQueueReceive(rxCmdQueue, &packet, portMAX_DELAY);
        if (res == pdTRUE) {
            if (validatePacket(packet) != 0) {
                /* TODO: Corrupted packet, report an error */
            } else {
                if (rxMsgCb != NULL) {
                    rxMsgCb(packet);
                }
            }
            vPortFree(packet);
        }
    }
}

/**
 * \brief IPC initialization
 * 
 * \return      0 on success negative value in case of error
 */
int ipcInit(void)
{
    txCmdQueue = xQueueCreate(IPC_TX_QUEUE_LEN, sizeof(ipcPacket_t*));
	if (txCmdQueue == 0U){
		/* Queue was not created and must not be used. */
        return -1;
	}

    rxCmdQueue = xQueueCreate(IPC_TX_QUEUE_LEN, sizeof(ipcPacket_t*));
	if (rxCmdQueue == 0U){
		/* Queue was not created and must not be used. */
        return -2;
	}

    xTaskCreate(ipcTxTask, "ipc_tx", IPC_TX_TASK_STACK, NULL, IPC_TX_TASK_PRIO, &txTaskHandle);
    if (txTaskHandle == 0U) {
        return -3;
    }

    xTaskCreate(ipcRxTask, "ipc_rx", IPC_RX_TASK_STACK, NULL, IPC_RX_TASK_PRIO, &rxTaskHandle);
	if (rxTaskHandle == 0U) {
        return -4;
    }

    return 0;
}

/**
 * \brief Register callback function which will be called when new IPC packet will be reived
 * 
 * \param [in]    rxCb Pointer of the callback function
 * \return             0 on success negative value in case of error
 */
int ipcRegisterRxCb(ipcRxCb_t rxCb)
{
    if (rxCb == NULL) {
        return -1;
    }

    rxMsgCb = rxCb;

    return 0;
}

/**
 * \brief Send new IPC command
 * 
 * \param [in]    srcId   Source ID
 * \param [in]    dstId   Destination ID
 * \param [in]    cmdId   Command
 * \param [in]    payload Pointer to the payload data
 * \param [in]    len     Length of the payload
 * \return                0 on success negative value in case of error 
 */
int ipcSendCommand(uint8_t srcId, uint8_t dstId, uint16_t cmdId, uint8_t *payload, uint8_t len)
{
    ipcPacket_t *packet = NULL;
    BaseType_t res;

    if ((len > 0U) && (payload == NULL)) {
        return -1;
    }
    
    packet = pvPortMalloc(IPC_PACKET_LEN);
    if (packet == NULL) {
        return -2;
    }

    /* Clear the packet structure */
    memset(packet, 0, IPC_PACKET_LEN);

    packet->header.frameCnt = pktFrmCnt;
    packet->header.srcId = srcId;
    packet->header.dstId = dstId;
    packet->header.cmdId = cmdId;
    packet->header.dataLen = len;
    if (len > 0U) {
        /* TODO: Check amout of copied data */
        (void) memcpy(packet->payload, payload, len);
    }
    packet->payloadChecksum = calculateChecksum(payload, len);

    res = xQueueSend(txCmdQueue, (void*) &packet, (TickType_t) IPC_QUEUE_TIMEOUT);
    if (res != pdTRUE) {
        vPortFree(packet);
        return -3;
    }

    /* Increment packet frame counter */
    pktFrmCnt++;

    return 0;
}

/**
 * \brief Process incomming data from the uart interface
 * 
 * \param [in]    dataPtr Pointer to the received data
 * \param [in]    dataLen Lenght of the received data
 * \return                0 on success negative value in case of error
 */
int ipcProcessReceivedData(uint8_t *dataPtr, uint32_t dataLen)
{
    ipcPacket_t *packet = NULL;
    BaseType_t res;

    if (dataLen != IPC_PACKET_LEN) {
        return -1;
    }

    packet = pvPortMalloc(IPC_PACKET_LEN);
    if (packet == NULL) {
        return -2;
    }

    /* TODO: Check amout of copied data */
    (void) memcpy(packet, dataPtr, dataLen);

    res = xQueueSend(rxCmdQueue, (void*) &packet, (TickType_t) IPC_QUEUE_TIMEOUT);
    if (res != pdTRUE) {
        vPortFree(packet);
        return -3;
    }

    return 0;
}
