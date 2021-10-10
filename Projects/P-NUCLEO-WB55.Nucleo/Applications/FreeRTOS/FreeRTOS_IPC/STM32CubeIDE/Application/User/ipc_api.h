#ifndef __IPC_API_H
#define __IPC_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "ipc_types.h"

int ipcInit(void);
int ipcRegisterRxCb(ipcRxCb_t rx_cb);
int ipcSendCommand(uint8_t srcId, uint8_t dstId, uint16_t cmdId, uint8_t *payload, uint8_t len);
int ipcProcessReceivedData(uint8_t *dataPtr, uint32_t dataLen);

#ifdef __cplusplus
}
#endif

#endif
