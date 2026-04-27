#ifndef __W25Q64_H
#define __W25Q64_H

#include "main.h"

 //W25Q64 命令定义
#define W25Q64_CMD_WRITE_ENABLE    0x06
#define W25Q64_CMD_WRITE_DISABLE   0x04
#define W25Q64_CMD_READ_DATA       0x03
#define W25Q64_CMD_FAST_READ       0x0B
#define W25Q64_CMD_PAGE_PROGRAM    0x02
#define W25Q64_CMD_SECTOR_ERASE    0x20
#define W25Q64_CMD_BLOCK_ERASE_32K 0x52
#define W25Q64_CMD_BLOCK_ERASE_64K 0xD8
#define W25Q64_CMD_CHIP_ERASE      0xC7
#define W25Q64_CMD_READ_STATUS1    0x05
#define W25Q64_CMD_READ_STATUS2    0x35
#define W25Q64_CMD_WRITE_STATUS    0x01
#define W25Q64_CMD_POWER_DOWN      0xB9
#define W25Q64_CMD_RELEASE_PD      0xAB
#define W25Q64_CMD_MANUFACTURER_ID 0x90
#define W25Q64_CMD_READ_UNIQUE_ID  0x4B

 W25Q64 容量：8MB
#define W25Q64_SECTOR_SIZE         4096    4KB 扇区
#define W25Q64_PAGE_SIZE           256     256字节页

 函数声明
void W25Q64_Init(void);
uint8_t W25Q64_ReadID(void);
void W25Q64_ReadData(uint32_t addr, uint8_t buffer, uint32_t len);
void W25Q64_PageProgram(uint32_t addr, uint8_t buffer, uint32_t len);
void W25Q64_SectorErase(uint32_t addr);
void W25Q64_WriteEnable(void);
void W25Q64_WaitBusy(void);

#endif