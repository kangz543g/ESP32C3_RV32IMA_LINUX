#ifndef MAIN_W25Q64_H_
#define MAIN_W25Q64_H_

#include "driver/spi_master.h"

// W25Q128
#define CMD_WRITE_ENABLE      0x06
#define CMD_WRITE_DISABLE     0x04
#define CMD_READ_STATUS_R1    0x05
#define CMD_READ_STATUS_R2    0x35
#define CMD_WRITE_STATUS_R    0x01 // Unimplemented
#define CMD_PAGE_PROGRAM      0x02
#define CMD_QUAD_PAGE_PROGRAM 0x32 // Unimplemented
#define CMD_BLOCK_ERASE64KB   0xd8
#define CMD_BLOCK_ERASE32KB   0x52
#define CMD_SECTOR_ERASE      0x20
#define CMD_CHIP_ERASE        0xC7
#define CMD_ERASE_SUPPEND     0x75 // Unimplemented
#define CMD_ERASE_RESUME      0x7A // Unimplemented
#define CMD_POWER_DOWN        0xB9
#define CMD_HIGH_PERFORM_MODE 0xA3 // Unimplemented
#define CMD_CNT_READ_MODE_RST 0xFF // Unimplemented
#define CMD_RELEASE_PDOWN_ID  0xAB // Unimplemented
#define CMD_MANUFACURER_ID    0x90
#define CMD_READ_UNIQUE_ID    0x4B
#define CMD_JEDEC_ID          0x9f

#define CMD_READ_DATA         0x03
#define CMD_READ_DATA4B       0x13
#define CMD_FAST_READ         0x0B
#define CMD_FAST_READ4B       0x0C
#define CMD_READ_DUAL_OUTPUT  0x3B // Unimplemented
#define CMD_READ_DUAL_IO      0xBB // Unimplemented
#define CMD_READ_QUAD_OUTPUT  0x6B // Unimplemented
#define CMD_READ_QUAD_IO      0xEB // Unimplemented
#define CMD_WORD_READ         0xE3 // Unimplemented

#define SR1_BUSY_MASK	        0x01
#define SR1_WEN_MASK	        0x02

// PSRAM64H
#define CMD_WRITE             0x02
#define CMD_READ              0x03
#define CMD_FAST_READ         0x0b
#define CMD_RESET_EN          0x66
#define CMD_RESET             0x99
#define CMD_READ_UNIQUE_ID    0x4B
#define CMD_READ_ID           0x9f

typedef struct {
	bool _4bmode;
	spi_device_handle_t _SPIHandle;
} W25Q128_t;


static esp_err_t W25Q128_init(W25Q128_t *dev);
static esp_err_t W25Q128_readStatusReg1(W25Q128_t * dev, uint8_t * reg1);
static esp_err_t W25Q128_readStatusReg2(W25Q128_t * dev, uint8_t * reg2);
static esp_err_t W25Q128_readUniqieID(W25Q128_t * dev, uint8_t * id);
static esp_err_t W25Q128_readManufacturer(W25Q128_t * dev, uint8_t * id);

uint16_t W25Q128_read(W25Q128_t * dev, uint32_t addr, uint8_t *buf, uint16_t n);
uint16_t W25Q128_fastread(W25Q128_t * dev, uint32_t addr, uint8_t *buf, uint16_t n);

static esp_err_t w25q_init(void);
#endif /* MAIN_W25Q64_H_ */
