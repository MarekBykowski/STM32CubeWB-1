#ifndef __IPC_TYPES_H
#define __IPC_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define IPC_DATA_LEN                (64U)   /** Lenght of the packet payload */
#define IPC_HEADER_LEN              (6U)    /** Lenght of the packet header  */
#define IPC_PACKET_LEN              (72U)   /** Lenght of the packet         */
#define IPC_TX_QUEUE_LEN            (10U)   /** Lenght of the tx queue       */
#define IPC_RX_QUEUE_LEN            (10U)   /** Lenght of the rx queue       */

/**
 * \brief IPC packet header type definiton
 * 
 */
typedef struct {
    uint8_t  frameCnt;              /** Frame counter           */
    uint8_t  srcId;                 /** Source ID               */
    uint8_t  dstId;                 /** Destination ID          */
    uint8_t  dataLen;               /** Data Length             */
    uint16_t cmdId;                 /** Command ID              */
} ipcPacketHeader_t;

/**
 * \brief IPC packet type definition
 * 
 */
typedef struct {
    ipcPacketHeader_t header;       /** Packet header           */
    uint8_t  reserved;              /** Padding                 */
    uint8_t  payloadChecksum;       /** Data checksum           */
    uint8_t  payload[IPC_DATA_LEN]; /** Actual Data in Message  */
} ipcPacket_t;

/**
 * \brief Packet reveived callback type definition
 * 
 */
typedef void (*ipcRxCb_t)(ipcPacket_t *rx_cmd);

#ifdef __cplusplus
}
#endif

#endif
