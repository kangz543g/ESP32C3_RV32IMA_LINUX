// w25q.h

#ifndef W25Q_H
#define W25Q_H

#include <Arduino.h>
#include <SPI.h>

// Define the pin numbers for SPI communication
#define MISO_PIN          5  
#define MOSI_PIN          6  
#define SCK_PIN           4  
#define CS_PIN           10

///////////
// Standard types
#include <stdint.h>
#include <stdbool.h>

typedef unsigned int UINT;
typedef unsigned char BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef uint64_t QWORD;
typedef WORD WCHAR;

// Disk status and result enums
typedef BYTE DSTATUS;

typedef enum {
    RES_OK = 0,
    RES_ERROR,
    RES_WRPRT,
    RES_NOTRDY,
    RES_PARERR
} DRESULT;

// Disk status bits
#define STA_NOINIT        0x01
#define STA_NODISK        0x02
#define STA_PROTECT       0x04

///////////

// Declare the data structure types
typedef struct {
  uint8_t  uuid[8];
  uint32_t jedec_id;
  uint32_t blockCount;
  uint32_t sectorSize;
  uint32_t sectorCount;
  uint32_t pageSize;
  uint32_t pageCount;
  uint32_t blockSize;
  uint32_t capacityKB;
  uint8_t  statusRegister1;
  uint8_t  statusRegister2;
  uint8_t  statusRegister3;
  uint8_t  lock;
  DRESULT  Stat;
} W25Q_Data;

// Declare the variables as extern
extern uint8_t rxbuf[10];
extern uint8_t txbuf[10];

extern W25Q_Data w25q_data;

#define W25Q_uniqueid        0x4b
#define W25Q_page_program    0x02
#define W25Q_read_data       0x03
#define W25Q_fast_read_data  0x0b
#define W25Q_write_disable   0x04
#define W25Q_read_status_r1  0x05
#define W25Q_read_status_r2  0x35
#define W25Q_read_status_r3  0x15
#define W25Q_write_status_r1 0x01
#define W25Q_write_status_r2 0x31
#define W25Q_write_status_r3 0x11
#define W25Q_sector_erase    0x20
#define W25Q_block_erase_32k 0x52
#define W25Q_block_erase_64k 0xd8
#define W25Q_write_enable    0x06
#define W25Q_erase_chip      0xc7
 
#define W25Q_device_id       0x90
#define W25Q_JEDEC_ID        0x9f


class W25Q {
  private:
    void spi_cs_low();
    void spi_cs_high();

  public:
    W25Q();
    ~W25Q();
    
    void begin();
    void reset();
    void getUID(uint8_t* uid);
    uint32_t get_JEDEC_ID();
    int getMID();
    
    bool w25q_spi_init();
    void send_simple_cmd(uint8_t cmd);
    
    void write_enable();
    void write_disable();
    
    void send_cmd_read(uint8_t cmd, uint32_t address, uint8_t *buf, uint32_t len, bool is_fast);
    void send_cmd_write(uint8_t cmd, uint32_t address, uint8_t *buf, uint32_t len);
    void send_cmd_addr(uint8_t cmd, uint32_t address);
    void send_cmd(uint8_t cmd, uint8_t *buf, uint32_t len);
    
    void Read_Status_Register_1();
    void Read_Status_Register_2();
    void Read_Status_Register_3();
    void Write_Status_Register_1();
    void Write_Status_Register_2();
    void Write_Status_Register_3();
    void ReadStatusRegister(W25Q_Data* status);
    
    uint8_t Read_Status_1(void);
    uint8_t Read_Status_2(void);
    uint8_t Read_Status_3(void);

    uint32_t page_to_sector_address(uint32_t pageAddress);
    uint32_t page_to_block_address(uint32_t pageAddress);
    uint32_t data_sector_to_block_address(uint32_t sectorAddress);
    uint32_t sector_to_page_address(uint32_t sectorAddress);
    uint32_t block_to_page_address(uint32_t blockAddress);
        
    uint8_t check_write_enable();
    void wait_busy();
    uint32_t check_erase(uint32_t addr);

    void Read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
    void FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
    void Write (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);

    void Erase_Sector(uint16_t numsector);
    void Write_Page (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
    uint32_t BytesToWrite (uint32_t size, uint16_t offset);
    uint32_t BytesToModify (uint32_t size, uint16_t offset);
    
    void Write_Clean (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
    
    uint8_t Read_Byte(uint32_t Addr);
    void Write_Byte(uint32_t Addr, uint8_t data);

    void Float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable);
    float Bytes2Float(uint8_t * ftoa_bytes_temp);
    float Read_NUM (uint32_t page, uint16_t offset);
    void Write_NUM (uint32_t page, uint16_t offset, float data);
    void Read_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);
    void Write_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);

    DRESULT disk_initialize();
};

#endif // W25Q_H
