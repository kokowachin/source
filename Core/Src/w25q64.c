//W25Q64 SPI Flash 的驱动程序
#include "w25q64.h"
#include "spi.h"  // 根据你的 SPI 接口修改

extern SPI_HandleTypeDef hspi1;  // 假设使用 SPI1
#define W25Q64_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)  // 根据实际引脚修改
#define W25Q64_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

// 初始化
void W25Q64_Init(void)
{
    // 配置 CS 引脚
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    W25Q64_CS_HIGH();
    
    // 等待芯片上电
    HAL_Delay(10);
    
    // 可选：读取 ID 验证连接
    // uint8_t id = W25Q64_ReadID();
}

// 读取芯片 ID
uint8_t W25Q64_ReadID(void)
{
    uint8_t tx[4] = {W25Q64_CMD_MANUFACTURER_ID, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0};
    
    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, rx, 4, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
    
    return rx[3];  // 返回设备 ID
}

// 等待芯片空闲
void W25Q64_WaitBusy(void)
{
    uint8_t status;
    W25Q64_CS_LOW();
    
    uint8_t cmd = W25Q64_CMD_READ_STATUS1;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    
    do {
        HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    } while (status & 0x01);  // BUSY 位为1时等待
    
    W25Q64_CS_HIGH();
}

// 写使能
void W25Q64_WriteEnable(void)
{
    W25Q64_CS_LOW();
    uint8_t cmd = W25Q64_CMD_WRITE_ENABLE;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
}

// 读取数据
void W25Q64_ReadData(uint32_t addr, uint8_t *buffer, uint32_t len)
{
    uint8_t tx[4];
    
    tx[0] = W25Q64_CMD_READ_DATA;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;
    
    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, len, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
}

// 页编程（最多256字节）
void W25Q64_PageProgram(uint32_t addr, uint8_t *buffer, uint32_t len)
{
    if (len > W25Q64_PAGE_SIZE) {
        len = W25Q64_PAGE_SIZE;  // 限制长度
    }
    
    W25Q64_WriteEnable();
    W25Q64_WaitBusy();
    
    uint8_t tx[4];
    tx[0] = W25Q64_CMD_PAGE_PROGRAM;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;
    
    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, buffer, len, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
    
    W25Q64_WaitBusy();
}

// 扇区擦除（4KB）
void W25Q64_SectorErase(uint32_t addr)
{
    W25Q64_WriteEnable();
    W25Q64_WaitBusy();
    
    uint8_t tx[4];
    tx[0] = W25Q64_CMD_SECTOR_ERASE;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;
    
    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
    
    W25Q64_WaitBusy();
}