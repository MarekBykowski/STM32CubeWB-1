#ifndef __IPC_IF_H
#define __IPC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "ipc_types.h"

int ipcUartInit(int baudrate);
int ipcUartSendData(ipcPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif
