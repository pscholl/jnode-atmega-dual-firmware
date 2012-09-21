#ifndef __SDMANAGER_H__
#define __SDMANAGER_H__

#include "USBtoSerial.h"
#include "Descriptors.h"


#include <LUFA/Common/Common.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Peripheral/SPI.h>

#define SD_DEFAULT_BLOCK_SIZE 512


void SDManager_Init(void);
uint32_t SDManager_BlockCount(void);

bool SDManager_CheckDataflashOperation(void);

void SDManager_WriteBlocks(USB_ClassInfo_MS_Device_t* const MSInterfaceInfo, const uint32_t BlockAddress, uint16_t TotalBlocks);
void SDManager_ReadBlocks(USB_ClassInfo_MS_Device_t* const MSInterfaceInfo, const uint32_t BlockAddress, uint16_t TotalBlocks);

#endif
