#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "hal/usb_serial_jtag_ll.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "cache.h"
#include "w25q128.h"

#include "default8mbdtc.h"

struct MiniRV32IMAState *core;  // Declare core as a pointer

#define LED_PIN               8

uint8_t buf[256];             // Read data buffer
uint16_t n;                   // Number of data read

#define GPIO_MOSI	            6 
#define GPIO_MISO	            5 
#define GPIO_SCLK	            4 
#define GPIO_CS1              9   // PSRAM64H (8MB)
#define GPIO_CS2              10  // W25Q128 (8MB)

#define SPI_HOST_ID	          SPI2_HOST
#define PSRAM_SPI_FREQ	      40000000 // 40MHz
#define W25Q_SPI_FREQ         40000000 // 40MHZ

int fail_on_all_faults = 0;

////////////////////////////////////////////////////////////////

static uint32_t RAM_SIZE = 8 * 1024 * 1024;   // 8,388,608bytes

static uint32_t HandleException(uint32_t ir, uint32_t retval);
static uint32_t HandleControlStore(uint32_t addy, uint32_t val);
static uint32_t HandleControlLoad(uint32_t addy );
static void HandleOtherCSRWrite(uint8_t *image, uint16_t csrno, uint32_t value);
static int32_t HandleOtherCSRRead(uint8_t *image, uint16_t csrno);
static void MiniSleep();

// This is the functionality we want to override in the emulator.
//  think of this as the way the emulator's processor is connected to the outside world.
#define MINIRV32WARN(x...) printf(x);
#define MINI_RV32_RAM_SIZE RAM_SIZE  
#define MINIRV32_IMPLEMENTATION
#define MINIRV32_POSTEXEC(pc, ir, retval)             \
    {                                                 \
        if (retval > 0)                               \
        {                                             \
            if (fail_on_all_faults)                   \
            {                                         \
                printf("FAULT\n");                    \
                return 3;                             \
            }                                         \
            else                                      \
                retval = HandleException(ir, retval); \
        }                                             \
    }
//#define MINIRV32_POSTEXEC(pc, ir, retval) { if (retval > 0) {  retval = HandleException(ir, retval); } }
#define MINIRV32_HANDLE_MEM_STORE_CONTROL(addy, val) if (HandleControlStore(addy, val)) return val;
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL(addy, rval) rval = HandleControlLoad(addy);
#define MINIRV32_OTHERCSR_WRITE(csrno, value) HandleOtherCSRWrite(image, csrno, value);
#define MINIRV32_OTHERCSR_READ(csrno, value) value = HandleOtherCSRRead(image, csrno);

#define MINIRV32_CUSTOM_MEMORY_BUS

#define CACHESIZE  4096
struct cacheline {
  uint8_t data[64];
};

static uint64_t accessed, hit;
static uint32_t tags[CACHESIZE/64/2][2];
static struct cacheline cachelines[CACHESIZE/64/2][2];

int psram_read(uint32_t addr, void *buf, int len);
int psram_write(uint32_t addr, void *buf, int len);

/*
 * bit[0]: valid
 * bit[1]: dirty
 * bit[2]: for LRU
 * bit[3:10]: reserved
 * bit[11:31]: tag
 */
#define VALID   (1 << 0)
#define DIRTY   (1 << 1)
#define LRU   (1 << 2)
#define LRU_SFT   2
#define TAG_MSK   0xfffff800

/*
 * bit[0: 5]: offset
 * bit[6: 10]: index
 * bit[11: 31]: tag
 */
static inline int get_index(uint32_t addr)
{
  return (addr >> 6) & 0x1f;
}

void cache_write(uint32_t ofs, void *buf, uint32_t size)
{
  if (((ofs | (64 - 1)) != ((ofs + size - 1) | (64 - 1))))
    //printf("write cross boundary, ofs:%x size:%x\n", ofs, size);
    printf("write cross boundary, ofs:%lx size:%lx\n", (unsigned long)ofs, (unsigned long)size);

  int ti, i, index = get_index(ofs);
  uint32_t *tp;
  uint8_t *p;

  ++accessed;

  for (i = 0; i < 2; i++) {
    tp = &tags[index][i];
    p = cachelines[index][i].data;
    if (*tp & VALID) {
      if ((*tp & TAG_MSK) == (ofs & TAG_MSK)) {
        ++hit;
        ti = i;
        break;
      } else {
        if (i != 1)
          continue;

        ti = 1 - ((*tp & LRU) >> LRU_SFT);
        tp = &tags[index][ti];
        p = cachelines[index][ti].data;

        if (*tp & DIRTY) {
          psram_write(*tp & ~0x3f, p, 64);
        }
        psram_read(ofs & ~0x3f, p, 64);
        *tp = ofs & ~0x3f;
        *tp |= VALID;
      }
    } else {
      if (i != 1)
        continue;

      ti = i;
      psram_read(ofs & ~0x3f, p, 64);
      *tp = ofs & ~0x3f;
      *tp |= VALID;
    }
  }

  tags[index][1] &= ~(LRU);
  tags[index][1] |= (ti << LRU_SFT);
  memcpy(p + (ofs & 0x3f), buf, size);
  *tp |= DIRTY;
}

void cache_read(uint32_t ofs, void *buf, uint32_t size)
{
  if (((ofs | (64 - 1)) != ((ofs + size - 1) | (64 - 1))))
    //printf("read cross boundary, ofs:%x size:%x\n", ofs, size);
    printf("read cross boundary, ofs:%lx size:%lx\n", (unsigned long)ofs, (unsigned long)size);

  int ti, i, index = get_index(ofs);
  uint32_t *tp;
  uint8_t *p;

  ++accessed;

  for (i = 0; i < 2; i++) {
    tp = &tags[index][i];
    p = cachelines[index][i].data;
    if (*tp & VALID) {
      if ((*tp & TAG_MSK) == (ofs & TAG_MSK)) {
        ++hit;
        ti = i;
        break;
      } else {
        if (i != 1)
          continue;

        ti = 1 - ((*tp & LRU) >> LRU_SFT);
        tp = &tags[index][ti];
        p = cachelines[index][ti].data;

        if (*tp & DIRTY) {
          psram_write(*tp & ~0x3f, p, 64);
        }
        psram_read(ofs & ~0x3f, p, 64);
        *tp = ofs & ~0x3f;
        *tp |= VALID;
      }
    } else {
      if (i != 1)
        continue;

      ti = i;
      psram_read(ofs & ~0x3f, p, 64);
      *tp = ofs & ~0x3f;
      *tp |= VALID;
    }
  }

  tags[index][1] &= ~(LRU);
  tags[index][1] |= (ti << LRU_SFT);
  memcpy(buf, p + (ofs & 0x3f), size);
}

void cache_get_stat(uint64_t *phit, uint64_t *paccessed)
{
  *phit = hit;
  *paccessed = accessed;
}

////////////////////////////////////////////////////////////////

static void MINIRV32_STORE4(uint32_t ofs, uint32_t val)
{
  cache_write(ofs, &val, 4);
}

static void MINIRV32_STORE2(uint32_t ofs, uint16_t val)
{
  cache_write(ofs, &val, 2);
}

static void MINIRV32_STORE1(uint32_t ofs, uint8_t val)
{
  cache_write(ofs, &val, 1);
}

static uint32_t MINIRV32_LOAD4(uint32_t ofs)
{
  uint32_t val;
  cache_read(ofs, &val, 4);
  return val;
}

static uint16_t MINIRV32_LOAD2(uint32_t ofs)
{
  uint16_t val;
  cache_read(ofs, &val, 2);
  return val;
}

static uint8_t MINIRV32_LOAD1(uint32_t ofs)
{
  uint8_t val;
  cache_read(ofs, &val, 1);
  return val;
}

#include "mini-rv32ima.h"

void DumpState(struct MiniRV32IMAState *core)
{
  unsigned int pc = core->pc;
  unsigned int *regs = (unsigned int *)core->regs;
  uint64_t thit, taccessed;

  cache_get_stat(&thit, &taccessed);
  Serial.printf("hit: %llu accessed: %llu\r\n", thit, taccessed);
  Serial.printf("PC: %08x \r\n", pc);
  Serial.printf("Z:%08x ra:%08x sp:%08x gp:%08x tp:%08x t0:%08x t1:%08x t2:%08x s0:%08x s1:%08x a0:%08x a1:%08x a2:%08x a3:%08x a4:%08x a5:%08x\r\n",
    regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7],
    regs[8], regs[9], regs[10], regs[11], regs[12], regs[13], regs[14], regs[15] );
  Serial.printf("a6:%08x a7:%08x s2:%08x s3:%08x s4:%08x s5:%08x s6:%08x s7:%08x s8:%08x s9:%08x s10:%08x s11:%08x t3:%08x t4:%08x t5:%08x t6:%08x\r\n",
    regs[16], regs[17], regs[18], regs[19], regs[20], regs[21], regs[22], regs[23],
    regs[24], regs[25], regs[26], regs[27], regs[28], regs[29], regs[30], regs[31] );
}


////////////////////////////////////////////////////////////////
static spi_device_handle_t handle;

static esp_err_t psram_send_cmd(spi_device_handle_t h, const uint8_t cmd)
{
	spi_transaction_ext_t t = { };
	t.base.flags = SPI_TRANS_VARIABLE_ADDR;
	t.base.cmd = cmd;
	t.base.length = 0;
  t.command_bits = 8U;
  t.address_bits = 0;

	return spi_device_polling_transmit(h, (spi_transaction_t*)&t);
}

static esp_err_t psram_read_id(spi_device_handle_t h, uint8_t *rxdata)
{
	spi_transaction_t t = { };
	t.cmd = CMD_READ_ID;
	t.addr = 0;
	t.rx_buffer = rxdata;
	t.length = 6 * 8;
	return spi_device_polling_transmit(h, &t);
}

static esp_err_t psram_init(void) {
  esp_err_t ret;
  uint8_t id[6];

  // Initialize GPIO for CS pin (chip select)
  gpio_reset_pin((gpio_num_t)GPIO_CS1);
  gpio_set_direction((gpio_num_t)GPIO_CS1, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)GPIO_CS1, 1);

#if 0
  // Configure the SPI bus settings
  spi_bus_config_t spi_bus_config = {
    .mosi_io_num = GPIO_MOSI,
    .miso_io_num = GPIO_MISO,
    .sclk_io_num = GPIO_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 0,
    .flags = 0,
  };

  Serial.printf("SPI_HOST_ID = %d\n", SPI_HOST_ID);
  ret = spi_bus_initialize(SPI_HOST_ID, &spi_bus_config, SPI_DMA_CH_AUTO);
  Serial.printf("spi_bus_initialize = %d\n", ret);
  if (ret != ESP_OK)
    return -1;
#endif

  // Configure the SPI device interface for PSRAM
  spi_device_interface_config_t devcfg;
  memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
  devcfg.clock_speed_hz = PSRAM_SPI_FREQ;
  devcfg.spics_io_num = -1;  // Use the correct chip select
  devcfg.queue_size = 1;
  devcfg.command_bits = 8;
  devcfg.address_bits = 24;

  ret = spi_bus_add_device(SPI_HOST_ID, &devcfg, &handle);
  if (ret != ESP_OK) {
    Serial.printf("spi_bus_add_device failed with error: %d\r\n", ret);
    return ret;
  }

  // Reset PSRAM and check its ID
  gpio_set_level((gpio_num_t)GPIO_CS1, 1);
  usleep(200);

  gpio_set_level((gpio_num_t)GPIO_CS1, 0);
  psram_send_cmd(handle, CMD_RESET_EN);
  psram_send_cmd(handle, CMD_RESET);
  gpio_set_level((gpio_num_t)GPIO_CS1, 1);
  usleep(200);

  gpio_set_level((gpio_num_t)GPIO_CS1, 0);
  ret = psram_read_id(handle, id);
  gpio_set_level((gpio_num_t)GPIO_CS1, 1);
  if (ret != ESP_OK) {
    Serial.printf("psram_read_id failed with error: %d\r\n", ret);
    return ret;
  }

  // ESP-IDF PSRAM ID: 0d5d52d26fd0(80MHz), ARDUINO ESP32 PSRAM ID: 0d5d50b1c027(40MHz)
  Serial.printf("PSRAM ID: %02X-%02X-%02X-%02X-%02X-%02X\r\n", id[0], id[1], id[2], id[3], id[4], id[5]);
  return ESP_OK;  // Success
}

int psram_read(uint32_t addr, void *buf, int len)
{
	esp_err_t ret;
	spi_transaction_ext_t t = { };

	t.base.cmd = CMD_FAST_READ;
	t.base.addr = addr;
	t.base.rx_buffer = buf;
	t.base.length = len * 8;
	t.base.flags = SPI_TRANS_VARIABLE_DUMMY;
	t.dummy_bits = 8;

  //memcpy(buf, (void *)(0x3F800000 + addr), len);
  
	gpio_set_level((gpio_num_t)GPIO_CS1, 0);
	ret = spi_device_polling_transmit(handle, (spi_transaction_t*)&t);
	gpio_set_level((gpio_num_t)GPIO_CS1, 1);

	if (ret != ESP_OK) {
		Serial.printf("psram_read failed %lx %d\r\n", addr, len);
		return -1;
	}

	return len;
}

int psram_write(uint32_t addr, void *buf, int len)
{
	esp_err_t ret;
	spi_transaction_t t = {};

	t.cmd = CMD_WRITE;
	t.addr = addr;
	t.tx_buffer = buf;
	t.length = len * 8;
  
  //memcpy((void *)(0x3F800000 + addr), buf, len);
  
	gpio_set_level((gpio_num_t)GPIO_CS1, 0);
	ret = spi_device_polling_transmit(handle, &t);
	gpio_set_level((gpio_num_t)GPIO_CS1, 1);

	if (ret != ESP_OK) {
		// Replacing printf with ESP_LOGE
		Serial.printf("psram_write failed 0x%lx %d\r\n", addr, len);
		return -1;
	}

	return len;
}


/*
 esp32c3 리눅스 에뮬레이터

~ # cat /proc/cpuinfo
processor : 0
hart    : 0
isa   : rv32ima_zicntr_zicsr_zifencei_zihpm
mmu   : none
mvendorid : 0xff0ff0ff
marchid : 0x0
mimpid    : 0x0
hart isa  : rv32ima_zicntr_zicsr_zifencei_zihpm

Image(1875967 bytes)  리눅스 실행됨


~ # cat /proc/cpuinfo
processor : 0
hart    : 0
isa   : rv32ima
mmu   : none
mvendorid : 0xff0ff0ff
marchid : 0x0
mimpid    : 0x0

Image(3476752 bytes)  리눅스 실행되지 않음
*/
#define ESP_FLASH  0
#define W25Q_FLASH 1

#if ESP_FLASH
#define kernel_start  0x200000
#define kernel_end    0x3c9fff 

static char dmabuf[64];

int verify_images(int ram_size)
{
  long flen;
  uint32_t addr, flashaddr;
  char flash_buf[64], psram_buf[64];

  flen = kernel_end - kernel_start;
  if (flen > ram_size) {
    Serial.println("Error: RAM size is smaller than the image size!");
    return -1;
  }

  addr = 0;
  flashaddr = kernel_start;
  Serial.printf("Verifying kernel image (%ld bytes) between flash:%lx and psram:%lx\r\n", flen, flashaddr, addr);

  while (flen >= 64) {
    esp_flash_read(NULL, flash_buf, flashaddr, 64);
    psram_read(addr, psram_buf, 64);

    if (memcmp(flash_buf, psram_buf, 64) != 0) {
      Serial.printf("Verification failed at addr: 0x%lx!\r\n", addr);
      return -1;
    }

    addr += 64;
    flashaddr += 64;
    flen -= 64;
  }

  if (flen) {
    esp_flash_read(NULL, flash_buf, flashaddr, flen);
    psram_read(addr, psram_buf, flen);

    if (memcmp(flash_buf, psram_buf, flen) != 0) {
      Serial.printf("Verification failed at addr: 0x%lx!\r\n", addr);
      return -1;
    }
  }

  Serial.println("Verification successful! Flash and PSRAM contents match.");
  return 0;
}

int load_images(int ram_size, int *kern_len)
{
  long flen;
  uint32_t addr, flashaddr;

  flen = kernel_end - kernel_start;
  if (flen > ram_size) {
    Serial.printf("Error: Could not fit RAM image (%lu bytes) into %d\r\n", flen, ram_size);
    return -1;
  }
  if (kern_len)
    *kern_len = flen;

  addr = 0;
  flashaddr = kernel_start;
  Serial.printf("Loading kernel Image (%ld bytes) from flash:%lx into psram:%lx\r\n", flen, flashaddr, addr);
  while (flen >= 64) {
    esp_flash_read(NULL, dmabuf, flashaddr, 64);
    psram_write(addr, dmabuf, 64);
    addr += 64;
    flashaddr += 64;
    flen -= 64;
  }
  if (flen) {
    esp_flash_read(NULL, dmabuf, flashaddr, flen);
    psram_write(addr, dmabuf, flen);
  }

  return 0;
}


#define dtb_start  0x3FF000
#define dtb_end    0x3FF600 

int dtb_load_images(int ram_size, int *kern_len)
{
  long flen;
  uint32_t addr, flashaddr;
  esp_flash_t *chip = NULL;  // 플래시 핸들

  flen = dtb_end - dtb_start;
  if (flen > ram_size) {
    Serial.printf("Error: Could not fit DTB image (%lu bytes) into %d\r\n", flen, ram_size);
    return -1;
  }
  if (kern_len)
    *kern_len = flen;

  addr = 0x7FF940;
  flashaddr = dtb_start;
  Serial.printf("Loading DTB Image (%ld bytes) from flash:0x%lx into psram:0x%lx\r\n", flen, flashaddr, addr);

  while (flen >= 64) {
    if (esp_flash_read(chip, dmabuf, flashaddr, 64) != ESP_OK) {
      Serial.println("Error: Failed to read flash");
      return -1;
    }
    psram_write(addr, dmabuf, 64);  // psram_write() 함수 필요
    addr += 64;
    flashaddr += 64;
    flen -= 64;
  }

  if (flen) {
    if (esp_flash_read(chip, dmabuf, flashaddr, flen) != ESP_OK) {
      Serial.println("Error: Failed to read flash (last chunk)");
      return -1;
    }
    psram_write(addr, dmabuf, flen);
  }

  return 0;
}

#endif


#if W25Q_FLASH
/*
int verify_images(int ram_size)
{
  long flen;
  uint32_t addr, flashaddr;
  uint8_t flash_buf[DMA_BUF_SIZE], psram_buf[DMA_BUF_SIZE];

  flen = kernel_end - kernel_start;
  if (flen > ram_size) {
    Serial.println("Error: RAM size is smaller than the image size!");
    return -1;
  }

  addr = 0;
  flashaddr = kernel_start;
  Serial.printf("Verifying kernel image (%ld bytes) between flash:%lx and psram:%lx\r\n", flen, flashaddr, addr);

  while (flen >= DMA_BUF_SIZE) {
    // Read 64 bytes from flash
    W25Q128_Read(flashaddr / 256, flashaddr % 256, DMA_BUF_SIZE, flash_buf);
    
    // Read 64 bytes from PSRAM
    psram_read(addr, psram_buf, DMA_BUF_SIZE);

    // Compare flash and PSRAM data
    if (memcmp(flash_buf, psram_buf, DMA_BUF_SIZE) != 0) {
      Serial.printf("Verification failed at addr: 0x%lx!\r\n", addr);
      return -1;
    }

    addr += DMA_BUF_SIZE;
    flashaddr += DMA_BUF_SIZE;
    flen -= DMA_BUF_SIZE;
  }

  if (flen) {
    // Read remaining bytes from flash
    W25Q128_Read(flashaddr / 256, flashaddr % 256, flen, flash_buf);
    
    // Read remaining bytes from PSRAM
    psram_read(addr, psram_buf, flen);

    // Compare remaining flash and PSRAM data
    if (memcmp(flash_buf, psram_buf, flen) != 0) {
      Serial.printf("Verification failed at addr: 0x%lx!\r\n", addr);
      return -1;
    }
  }

  Serial.println("Verification successful! Flash and PSRAM contents match.");
  return 0;
}
*/

uint8_t dmabuf[256];        // Flash에서 읽은 데이터를 저장할 버퍼
//uint8_t verify_buf[256];    // PSRAM에서 읽은 데이터를 저장할 버퍼

#define kernel_start  0x000000
#define kernel_end    0x1C9FFF   // 2532208
//0x1C9FFF  // 1875967 bytes test ok 
//0x350D1F  // 3476752 bytes test error

int load_images(W25Q128_t * dev, int ram_size, int *kern_len)
{
  long flen;
  uint32_t addr, flashaddr;

  flen = kernel_end - kernel_start;
  if (flen > ram_size) {
    Serial.printf("Error: Could not fit RAM image (%lu bytes) into %d\r\n", flen, ram_size);
    return -1;
  }
  if (kern_len)
    *kern_len = flen;

  addr = 0;
  flashaddr = kernel_start;

  // LED 켜기 (이미지 로딩 시작)
  digitalWrite(LED_PIN, LOW);
  
  Serial.printf("Loading kernel Image (%ld bytes) from flash:%lx into psram:%lx\r\n", flen, flashaddr, addr);

  while (flen >= 64) {
    // Using W25Q128_fastread instead of W25Q128_read
    W25Q128_fastread(dev, flashaddr, dmabuf, 64);
    psram_write(addr, dmabuf, 64);
    addr += 64;
    flashaddr += 64;
    flen -= 64;
  }

  if (flen) {
    // Using W25Q128_fastread for remaining bytes
    W25Q128_fastread(dev, flashaddr, dmabuf, flen);
    psram_write(addr, dmabuf, flen);
  }

  // LED 끄기 (이미지 로딩 완료)
  digitalWrite(LED_PIN, HIGH);
  
  return 0;
}

#endif

////////////////////////////////////////////////////////////////

uint64_t GetTimeMicroseconds()
{
  return esp_timer_get_time();
}

static void MiniSleep(void)
{
  usleep(10);
}

static uint32_t HandleException(uint32_t ir, uint32_t code)
{
  // Weird opcode emitted by duktape on exit.
  if (code == 3) {
    // Could handle other opcodes here.
  }
  return code;
}

uint32_t HandleControlStore( uint32_t addy, uint32_t val ) {
  if( addy == 0x10000000 ) { //UART 8250 / 16550 Data Buffer
    Serial.print((char)val);
  }
  return 0;
}


static uint32_t HandleControlLoad( uint32_t addy )
{
  // Emulating a 8250 / 16550 UART
  if( addy == 0x10000005 )
    return 0x60 | (Serial.available() > 0);
  else if( addy == 0x10000000 && (Serial.available() > 0) )
    return Serial.read();
  return 0;
}

static void HandleOtherCSRWrite(uint8_t *image, uint16_t csrno, uint32_t value) {
  if (csrno == 0x136) {
    Serial.print(value);
  } else if (csrno == 0x137) {
    Serial.print(value, HEX);
  } else if (csrno == 0x138) {
    // Print "string"
    uint32_t ptrstart = value - MINIRV32_RAM_IMAGE_OFFSET;
    uint32_t ptrend = ptrstart;
    if (ptrstart >= RAM_SIZE) {
      Serial.printf("DEBUG PASSED INVALID PTR (%08x)\n", value);
      return;
    }
    while (ptrend < RAM_SIZE) {
      if (image[ptrend] == 0) break;
      ptrend++;
    }
    if (ptrend != ptrstart) {
      for (uint32_t i = ptrstart; i < ptrend; i++) {
        Serial.print((char)image[i]);
      }
    }
  } else if (csrno == 0x139) {
    Serial.print((char)value);
  }
}

static int32_t HandleOtherCSRRead(uint8_t *image, uint16_t csrno)
{
  if (csrno == 0x140) {
    if (!Serial.available())
      return -1;
    return Serial.read();
  }
  return 0;
}

////////////////////////////////////////////////////////////////

// Data dump list
// dt(in):Data to dump
// n(in) :Number of bytes of data
//
void dump(uint8_t *dt, int n) {
  uint16_t clm = 0;
  uint8_t sum;
  uint8_t vsum[16] = {0};
  uint8_t total = 0;
  uint32_t saddr = 0;
  uint32_t eaddr = n - 1;

  Serial.printf("--------------------------------------------------------------------------------\r\n");
  
  uint32_t addr;
  for (addr = saddr; addr <= eaddr; addr++) {
    if (clm == 0) {
      sum = 0;
      Serial.printf("%05" PRIx32 ": ", addr);  // 시작 주소 출력
    }

    uint8_t data = dt[addr];
    Serial.printf("%02x ", data);
    sum += data;
    vsum[addr % 16] += data;
    clm++;

    // ASCII 데이터를 따로 저장
    char ascii_char = (data >= 0x20 && data <= 0x7E) ? (char)data : '.';

    // 16바이트마다 줄바꿈 + ASCII 출력
    if (clm == 16 || addr == eaddr) {
      // 남은 빈칸 채우기 (마지막 줄이 16바이트보다 작을 경우)
      for (int j = clm; j < 16; j++) {
        Serial.print("   ");
      }
      Serial.printf("| %02x  ", sum);

      // ASCII 출력
      Serial.print("  ");
      for (int j = addr - clm + 1; j <= addr; j++) {
        char c = (dt[j] >= 0x20 && dt[j] <= 0x7E) ? (char)dt[j] : '.';
        Serial.print(c);
      }
      Serial.println();
      clm = 0;
    }
  }

  Serial.printf("--------------------------------------------------------------------------------\r\n");
  Serial.printf("       ");
  for (int i = 0; i < 16; i++) {
    total += vsum[i];
    Serial.printf("%02x ", vsum[i]);
  }
  Serial.printf("| %02x \n\n", total);
}

////////////////////////////////////////////////////////////////

// Define the W25Q128_init function
static esp_err_t W25Q128_init(W25Q128_t *dev)  // Pass the dev pointer to initialize the device
{
  esp_err_t ret;

  // Initialize GPIO for CS pin (chip select)
  gpio_reset_pin((gpio_num_t)GPIO_CS2);
  gpio_set_direction((gpio_num_t)GPIO_CS2, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)GPIO_CS2, 0);

  // Configure the SPI bus settings
  spi_bus_config_t spi_bus_config = {
    .mosi_io_num = GPIO_MOSI,
    .miso_io_num = GPIO_MISO,
    .sclk_io_num = GPIO_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1
  };

  // Initialize SPI bus
  ret = spi_bus_initialize(SPI_HOST_ID, &spi_bus_config, SPI_DMA_CH_AUTO);
  Serial.printf("spi_bus_initialize=%d\r\n", ret);
  if (ret != ESP_OK) {
    return ret; // Return the error code if SPI initialization fails
  }

  // Configure the SPI device interface
  spi_device_interface_config_t devcfg;
  memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
  devcfg.clock_speed_hz = W25Q_SPI_FREQ;
  devcfg.spics_io_num = GPIO_CS2;
  devcfg.queue_size = 7;
  devcfg.mode = 0;

  // Add the SPI device to the bus
  ret = spi_bus_add_device(SPI_HOST_ID, &devcfg, &(dev->_SPIHandle));
  Serial.printf("spi_bus_add_device=%d\r\n", ret);
  if (ret != ESP_OK) {
    return ret; // Return the error code if adding the device fails
  }

  // Set 4-byte address mode flag
  dev->_4bmode = false;

#if CONFIG_4B_MODE
  Serial.println("4-Byte Address Mode");
  dev->_4bmode = true;
#endif

  return ESP_OK; // Return success if everything goes well
}

static esp_err_t W25Q128_readUniqieID(W25Q128_t * dev, uint8_t * id)
{
  spi_transaction_t SPITransaction;
  uint8_t data[13];
  data[0] = CMD_READ_UNIQUE_ID;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 13 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  memcpy(id, &data[5], 8);
  return ret ;
}

// Get JEDEC ID(Manufacture, Memory Type,Capacity)
// d(out):Stores 3 bytes of Manufacture, Memory Type, Capacity
//
static esp_err_t W25Q128_readManufacturer(W25Q128_t * dev, uint8_t * id)
{
  spi_transaction_t SPITransaction;
  uint8_t data[4];
  data[0] = CMD_JEDEC_ID;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 4 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  memcpy(id, &data[1], 3);
  return ret ;
}

// Check during processing such as writing
// Return value: true:processing false:idle
//
bool W25Q128_IsBusy(W25Q128_t * dev)
{
  spi_transaction_t SPITransaction;
  uint8_t data[2];
  data[0] = CMD_READ_STATUS_R1;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 2 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  if (ret != ESP_OK) return false;
  if( (data[1] & SR1_BUSY_MASK) != 0) return true;
  return false;
}

// Power down 
//
static esp_err_t W25Q128_powerDown(W25Q128_t * dev)
{
  spi_transaction_t SPITransaction;
  uint8_t data[1];
  data[0] = CMD_POWER_DOWN;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 1 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  return ret;
}


// Get status register 1
// reg1(out):Value of status register 1
//
static esp_err_t W25Q128_readStatusReg1(W25Q128_t * dev, uint8_t * reg1)
{
  spi_transaction_t SPITransaction;
  uint8_t data[2];
  data[0] = CMD_READ_STATUS_R1;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 2 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  
  *reg1 = data[1];
  return ret;
}

// Get status register 2
// reg2(out):Value of status register 2
//
static esp_err_t W25Q128_readStatusReg2(W25Q128_t * dev, uint8_t * reg2)
{
  spi_transaction_t SPITransaction;
  uint8_t data[2];
  data[0] = CMD_READ_STATUS_R2;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 2 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  
  *reg2 = data[1];
  return ret;
}

// Read data
// addr(in):Read start address
//          3 Bytes Address Mode : 24 Bits 0x000000 - 0xFFFFFF
//          4 Bytes Address Mode : 32 Bits 0x00000000 - 0xFFFFFFFF
// n(in):Number of read data
//
uint16_t W25Q128_read(W25Q128_t * dev, uint32_t addr, uint8_t *buf, uint16_t n)
{ 
  spi_transaction_t SPITransaction;
  uint8_t *data;
  data = (uint8_t *)malloc(n+5);
  size_t offset;
  if (dev->_4bmode) {
    data[0] = CMD_READ_DATA4B;
    data[1] = (addr>>24) & 0xFF; // A31-A24
    data[2] = (addr>>16) & 0xFF; // A23-A16
    data[3] = (addr>>8) & 0xFF; // A15-A08
    data[4] = addr & 0xFF; // A07-A00
    offset = 5;
  } else {
    data[0] = CMD_READ_DATA;
    data[1] = (addr>>16) & 0xFF; // A23-A16
    data[2] = (addr>>8) & 0xFF; // A15-A08
    data[3] = addr & 0xFF; // A07-A00
    offset = 4;
  }
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = (n+offset) * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  memcpy(buf, &data[offset], n);
  free(data);
  if (ret != ESP_OK) return 0;
  return n;
}

// Fast read data
// addr(in):Read start address
//          3 Bytes Address Mode : 24 Bits 0x000000 - 0xFFFFFF
//          4 Bytes Address Mode : 32 Bits 0x00000000 - 0xFFFFFFFF
// n(in):Number of read data
//
uint16_t W25Q128_fastread(W25Q128_t * dev, uint32_t addr, uint8_t *buf, uint16_t n)
{
  spi_transaction_t SPITransaction;
  uint8_t *data;
  data = (uint8_t *)malloc(n+6);
  size_t offset;
  if (dev->_4bmode) {
    data[0] = CMD_FAST_READ4B;
    data[1] = (addr>>24) & 0xFF; // A31-A24
    data[2] = (addr>>16) & 0xFF; // A23-A16
    data[3] = (addr>>8) & 0xFF; // A15-A08
    data[4] = addr & 0xFF; // A07-A00
    data[5] = 0; // Dummy
    offset = 6;
  } else {
    data[0] = CMD_FAST_READ;
    data[1] = (addr>>16) & 0xFF; // A23-A16
    data[2] = (addr>>8) & 0xFF; // A15-A08
    data[3] = addr & 0xFF; // A07-A00
    data[4] = 0; // Dummy
    offset = 5;
  }
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = (n+offset) * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  memcpy(buf, &data[offset], n);
  free(data);
  if (ret != ESP_OK) return 0;
  return n;
}

// Write permission setting
//
static esp_err_t W25Q128_WriteEnable(W25Q128_t * dev)
{
  spi_transaction_t SPITransaction;
  uint8_t data[1];
  data[0] = CMD_WRITE_ENABLE;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 1 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  return ret;
}


// Write-protected setting
//
static esp_err_t W25Q128_WriteDisable(W25Q128_t * dev)
{
  spi_transaction_t SPITransaction;
  uint8_t data[1];
  data[0] = CMD_WRITE_DISABLE;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 1 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  return ret;
}

//
// Erasing data in 4kb space units
// sect_no(in):Sector number(0 - 2048)
// flgwait(in):true:Wait for complete / false:No wait for complete
// Return value: true:success false:fail
//
// Note:
// The data sheet states that erasing usually takes 30ms and up to 400ms.
// The upper 11 bits of the 23 bits of the address correspond to the sector number.
// The lower 12 bits are the intra-sectoral address.
//
// 補足:
// データシートでは消去に通常 30ms 、最大400msかかると記載されている
// アドレス23ビットのうち上位 11ビットがセクタ番号の相当する。
// 下位12ビットはセクタ内アドレスとなる。
//
bool W25Q128_eraseSector(W25Q128_t * dev, uint16_t sect_no, bool flgwait)
{
  spi_transaction_t SPITransaction;
  uint8_t data[4];
  uint32_t addr = sect_no;
  addr<<=12;

  // Write permission setting
  esp_err_t ret;
  ret = W25Q128_WriteEnable(dev);
  if (ret != ESP_OK) return false;

  data[0] = CMD_SECTOR_ERASE;
  data[1] = (addr>>16) & 0xff;
  data[2] = (addr>>8) & 0xff;
  data[3] = addr & 0xff;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 4 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  if (ret != ESP_OK) return false;

  // Busy check
  while( W25Q128_IsBusy(dev) & flgwait) {
    vTaskDelay(1);
  }
  return true;
}

// Erasing data in 64kb space units
// blk_no(in):Block number(0 - 127)
// flgwait(in):true:Wait for complete / false:No wait for complete
// Return value: true:success false:fail
//
// Note:
// The data sheet states that erasing usually takes 150ms and up to 1000ms.
// The upper 7 bits of the 23 bits of the address correspond to the block.
// The lower 16 bits are the address in the block.
//
// 補足:
// データシートでは消去に通常 150ms 、最大1000msかかると記載されている
// アドレス23ビットのうち上位 7ビットがブロックの相当する。下位16ビットはブロック内アドレスとなる。
//
bool W25Q128_erase64Block(W25Q128_t * dev, uint16_t blk_no, bool flgwait)
{
  spi_transaction_t SPITransaction;
  uint8_t data[4];
  uint32_t addr = blk_no;
  addr<<=16;

  // Write permission setting
  esp_err_t ret;
  ret = W25Q128_WriteEnable(dev);
  if (ret != ESP_OK) return false;

  data[0] = CMD_BLOCK_ERASE64KB;
  data[1] = (addr>>16) & 0xff;
  data[2] = (addr>>8) & 0xff;
  data[3] = addr & 0xff;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 4 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  if (ret != ESP_OK) return false;

  // Busy check
  while( W25Q128_IsBusy(dev) & flgwait) {
    vTaskDelay(1);
  }
  return true;
}

// Erasing data in 32kb space units
// blk_no(in):Block number(0 - 255)
// flgwait(in):true:Wait for complete / false:No wait for complete
// Return value: true:success false:fail
//
// Note:
// The data sheet states that erasing usually takes 120ms and up to 800ms.
// The upper 8 bits of the 23 bits of the address correspond to the block.
// The lower 15 bits are the in-block address.
//
// 補足:
// データシートでは消去に通常 120ms 、最大800msかかると記載されている
// アドレス23ビットのうち上位 8ビットがブロックの相当する。下位15ビットはブロック内アドレスとなる。
//
bool W25Q128_erase32Block(W25Q128_t * dev, uint16_t blk_no, bool flgwait)
{
  spi_transaction_t SPITransaction;
  uint8_t data[4];
  uint32_t addr = blk_no;
  addr<<=15;

  // Write permission setting
  esp_err_t ret;
  ret = W25Q128_WriteEnable(dev);
  if (ret != ESP_OK) return false;

  data[0] = CMD_BLOCK_ERASE32KB;
  data[1] = (addr>>16) & 0xff;
  data[2] = (addr>>8) & 0xff;
  data[3] = addr & 0xff;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 4 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  if (ret != ESP_OK) return false;

  // Busy check
  while( W25Q128_IsBusy(dev) & flgwait) {
    vTaskDelay(1);
  }
  return true;
}

// Erase all data
// flgwait(in):true:Wait for complete / false:No wait for complete
// Return value: true:success false:fail
//
// Note:
// The data sheet states that erasing usually takes 15s and up to 30s.
//
// 補足:
// データシートでは消去に通常 15s 、最大30sかかると記載されている
//
bool W25Q128_eraseAll(W25Q128_t * dev, bool flgwait)
{
  spi_transaction_t SPITransaction;
  uint8_t data[1];

  // Write permission setting
  esp_err_t ret;
  ret = W25Q128_WriteEnable(dev);
  if (ret != ESP_OK) return false;

  data[0] = CMD_CHIP_ERASE;
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = 1 * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  assert(ret==ESP_OK);
  if (ret != ESP_OK) return false;

  // Busy check
  while( W25Q128_IsBusy(dev) & flgwait) {
    vTaskDelay(1);
  }
  return true;
}

// Page write
// sect_no(in):Sector number(0x00 - 0x7FF) 
// inaddr(in):In-sector address(0x00-0xFFF)
// data(in):Write data
// n(in):Number of bytes to write(0～256)
//
int16_t W25Q128_pageWrite(W25Q128_t * dev, uint16_t sect_no, uint16_t inaddr, uint8_t* buf, int16_t n)
{
  if (n > 256) return 0;
  spi_transaction_t SPITransaction;
  uint8_t *data;

  uint32_t addr = sect_no;
  addr<<=12;
  addr += inaddr;

  // Write permission setting
  esp_err_t ret;
  ret = W25Q128_WriteEnable(dev);
  if (ret != ESP_OK) return 0;

  // Busy check
  if (W25Q128_IsBusy(dev)) return 0;  

  data = (unsigned char*)malloc(n+4);
  data[0] = CMD_PAGE_PROGRAM;
  data[1] = (addr>>16) & 0xff;
  data[2] = (addr>>8) & 0xff;
  data[3] = addr & 0xFF;
  memcpy( &data[4], buf, n );
  memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
  SPITransaction.length = (n+4) * 8;
  SPITransaction.tx_buffer = data;
  SPITransaction.rx_buffer = data;
  ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
  free(data);
  assert(ret==ESP_OK);
  if (ret != ESP_OK) return 0;

  // Busy check
  while( W25Q128_IsBusy(dev) ) {
    vTaskDelay(1);
  }
  return n;
}

////
#define PSRAM_BASE 0x3F800000  // PSRAM 시작 주소 (ESP32 WROVER 기준)

// W25Q128 → PSRAM 복사 함수
void load_from_w25q128_to_psram(W25Q128_t * dev, uint32_t src_addr, uint32_t dst_addr, uint32_t size) {
    uint8_t buffer[256];

    Serial.printf("W25Q128에서 PSRAM으로 %d 바이트 복사 중...\n", size);
    for (uint32_t i = 0; i < size; i += 256) {
        W25Q128_read(dev, src_addr + i, buffer, 256); // W25Q128에서 읽기
        memcpy((void*)(dst_addr + i), buffer, 256); // PSRAM으로 복사
    }
    Serial.println("복사 완료!");
}

// PSRAM 데이터 검증 함수
void verify_psram_data(uint32_t addr, uint32_t size) {
    Serial.println("PSRAM 데이터 검증 중...");
    for (uint32_t i = 0; i < size; i++) {
        uint8_t data = *((uint8_t*)(addr + i)); // PSRAM에서 데이터 읽기
        Serial.printf("%02X ", data);
        if ((i + 1) % 16 == 0) Serial.println(); // 16바이트마다 줄바꿈
    }
    Serial.println("\nPSRAM 데이터 덤프 완료!");
}

////////////////////////////////////////////////////////////////

#include "esp_heap_caps.h"

#define TIME_DIVISOR 1

// Example 
#if 0
// セクタ単位の削除
  // Erase data by Sector
  bool flag = W25Q64_eraseSector(&dev, 0, true);
  if (flag == false) {
    ESP_LOGE(TAG, "eraseSector fail %d",ret);
    while(1) { vTaskDelay(1); }
  }
  memset(rbuf, 0, 256);
  len =  W25Q64_read(&dev, 0, rbuf, 256);
  if (len != 256) {
    ESP_LOGE(TAG, "read fail");
    while(1) { vTaskDelay(1); }
  }
  ESP_LOGI(TAG, "Read Data: len=%d", len);
  dump(rbuf, 256);

  // データ書き込みテスト
  // Write data to Sector=0 Address=10
  uint8_t wdata[26];    // 書込みデータ
  for (int i=0; i<26; i++) {
    wdata[i]='A'+i; // A-Z     
  }  
  len =  W25Q64_pageWrite(&dev, 0, 10, wdata, 26);
  if (len != 26) {
    ESP_LOGE(TAG, "pageWrite fail");
    while(1) { vTaskDelay(1); }
  }
  ESP_LOGI(TAG, "Page Write(Sector=0 Address=10) len=%d", len);

  // 高速データの読み込み(アドレス0から256バイト取得)
  // First read 256 byte data from Address=0
  memset(rbuf, 0, 256);
  len =  W25Q64_fastread(&dev, 0, rbuf, 256);
  if (len != 256) {
    ESP_LOGE(TAG, "fastread fail");
    while(1) { vTaskDelay(1); }
  }
  ESP_LOGI(TAG, "Fast Read Data: len=%d", len);
  dump(rbuf, 256);

  // データ書き込みテスト
  // Write data to Sector=0 Address=0
  for (int i=0; i < 10;i++) {
    wdata[i]='0'+i; // 0-9     
  }  
  len =  W25Q64_pageWrite(&dev, 0, 0, wdata, 10);
  if (len != 10) {
    ESP_LOGE(TAG, "pageWrite fail");
    while(1) { vTaskDelay(1); }
  }
  ESP_LOGI(TAG, "Page Write(Sector=0 Address=0) len=%d", len);

  // 高速データの読み込み(アドレス0から256バイト取得)
  // First read 256 byte data from Address=0
  memset(rbuf, 0, 256);
  len =  W25Q64_fastread(&dev, 0, rbuf, 256);
  if (len != 256) {
    ESP_LOGE(TAG, "fastread fail");
    while(1) { vTaskDelay(1); }
  }
  ESP_LOGI(TAG, "Fast Read Data: len=%d", len);
  dump(rbuf, 256);

  ESP_LOGI(TAG, "Success All Test");
#endif

////////////////////////////////////////////////////////////////

#define DTB_SIZE 1536

void setup()
{
  esp_err_t ret;
  
  // Initialize UART with baudrate 9600
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Disable the task watchdog timer
  esp_timer_deinit();
             
  usleep(5000000);  // Sleep for 5seconds
 
  
////////////////////////////////////////////////////////////////
   
  // Initialize W25Q128
  W25Q128_t dev;
  W25Q128_init(&dev);

  uint8_t reg1;
  ret = W25Q128_readStatusReg1(&dev, &reg1);
  
  if (ret != ESP_OK) {
    Serial.printf("readStatusReg1 fail %d\r\n", ret);
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("readStatusReg1 : %x\r\n", reg1);

  uint8_t reg2;
  ret = W25Q128_readStatusReg2(&dev, &reg2);
  
  if (ret != ESP_OK) {
    Serial.printf("readStatusReg2 fail %d\r\n", ret);
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("readStatusReg2 : %x\r\n", reg2);

  // Get Unique ID
  uint8_t uid[8];
  ret = W25Q128_readUniqieID(&dev, uid);
  
  if (ret != ESP_OK) {
    Serial.printf("readUniqieID fail %d\r\n", ret);
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("W25Q128 ID : %X-%X-%X-%X-%X-%X-%X-%X\r\n",
    uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]);

  // JEDEC ID Retrieval Test
  uint8_t jid[3];
  ret = W25Q128_readManufacturer(&dev, jid);
  if (ret != ESP_OK) {
    Serial.printf("readManufacturer fail %d\r\n", ret);
    while(1) { vTaskDelay(1); }
  }
  Serial.printf("JEDEC ID : %X%X%X\r\n", jid[0], jid[1], jid[2]);

  Serial.println("");

  Serial.println("psram init");
    
  // Initialize PSRAM
  if (psram_init() < 0) {
    Serial.println("failed to init psram");
    return;
  }  
  Serial.println("");


restart:
  
#if ESP_FLASH 
  // Loading the kernel into PSRAM
  if (load_images(RAM_SIZE, NULL) < 0) {
    Serial.println("Error loading kernel image into PSRAM");
    return;
  }
/*
  // 커널 이미지와 DTB(Device Tree Blob) 이미지를 PSRAM에 로드하는 과정
  if (dtb_load_images(RAM_SIZE, NULL) < 0) {
    Serial.println("Error loading dtb image into PSRAM");
    return;
  }

  delay(1000);
*/  
#endif

#if W25Q_FLASH
  // Loading the kernel into PSRAM
  if (load_images(&dev,RAM_SIZE, NULL) < 0) {
    Serial.println("Error loading kernel image into W25Q128");
    return;
  }

#endif
/*
  // W25Q128에서 PSRAM으로 데이터 복사
  uint32_t src_addr = 0x000000;  // W25Q128 내 시작 주소
  uint32_t dst_addr = PSRAM_BASE; // PSRAM 내 저장 주소
  uint32_t size = 1024; // 1KB 복사

  load_from_w25q128_to_psram(&dev, src_addr, dst_addr, size);

  // PSRAM에서 정상적으로 복사되었는지 확인
  verify_psram_data(dst_addr, size);
*/
    
#if 0
uint8_t rbuf[256];    // Buffer to store read data
int len;
uint32_t flashaddr = 0x0000;  // Start address of the flash
uint32_t dump_size = 0x2000;  // 0x2000 = 8192 bytes (2 KB)

// Loop through the range from 0x0000 to 0x2000, reading in 256-byte chunks
  while (flashaddr < dump_size) {
#if ESP_FLASH
    len = psram_read(flashaddr, rbuf, sizeof(rbuf)); 
#endif
#if W25Q_FLASH    
    // Read 256 bytes from the flash
    len = W25Q128_fastread(&dev, flashaddr, rbuf, sizeof(rbuf));  // Replace psram_read with W25Q128_fastread
#endif

    // Check if the read was successful
    if (len != sizeof(rbuf)) {
        Serial.println("fastread fail");
        while(1) { 
            vTaskDelay(1);  // Infinite loop for error handling
        }
    }

    // Print the address being dumped
    Serial.printf("Dumping data from flash address 0x%08X:\r\n", flashaddr);
    
    // Dump the contents of the buffer (print in hex format)
    dump(rbuf, sizeof(rbuf));

    // Increment the flash address by the size of the buffer (256 bytes)
    flashaddr += sizeof(rbuf);
  }
#endif
////////////////////////////////////////////////////////////////

  // Initialize emulator struct
  core = (struct MiniRV32IMAState *)malloc(sizeof(struct MiniRV32IMAState));
  memset(core, 0, sizeof(struct MiniRV32IMAState));

  uint32_t dtb_ptr = 0;
      
  // dtb 포인터 설정 (8MB - DTB 크기 - 코어 구조체 크기)   
  dtb_ptr = RAM_SIZE - sizeof(default8mbdtb) - sizeof( struct MiniRV32IMAState );
         
  // Set the program counter and Hart ID for the RISC-V core
  core->pc = MINIRV32_RAM_IMAGE_OFFSET;
  core->regs[10] = 0x00; //hart ID
  core->regs[11] = dtb_ptr?(dtb_ptr+MINIRV32_RAM_IMAGE_OFFSET):0;
  core->extraflags |= 3; // Machine-mode.

  Serial.println("");
  Serial.printf("Core PC after step: 0x%lx\r\n", core->pc);
  Serial.printf("Core Cycle: 0x%" PRIu32 " %" PRIu32 "\r\n", core->cycleh, core->cyclel);
  Serial.printf("Core Registers: [10] 0x%lx, [11] 0x%lx\r\n", core->regs[10], core->regs[11]);
  Serial.printf("Core Extra Flags: 0x%lx\r\n", core->extraflags);
     
  // Initialize the execution environment
  uint64_t lastTime = GetTimeMicroseconds();
  int instrs_per_flip = 1024;  // Number of instructions to execute per cycle

  Serial.println("");
  Serial.println("RV32IMA starting");
  Serial.println("");
  
  while(1)
  {
    int ret;
    
    // Calculate pseudo time
    uint64_t * this_ccount = ((uint64_t*)&core->cyclel);
    uint32_t elapsedUs = 0;
    elapsedUs = *this_ccount / TIME_DIVISOR - lastTime;
    lastTime += elapsedUs;
           
    // Execute up to 1024 cycles before breaking out
    ret = MiniRV32IMAStep(core, NULL, 0, elapsedUs, instrs_per_flip);  // Execute RV32IMA step
                
    switch (ret) {
      case 0: // No issues, continue the loop
            break;

      case 1: // A condition requiring MiniSleep
            MiniSleep();  // Likely a delay, could be useful for simulation timing
            *this_ccount += instrs_per_flip;  // Increment cycle counter by the number of instructions
            break;

      case 3: // Another valid condition
            break;

      case 0x7777:  // System control: restart
            Serial.printf("System restart triggered at 0x%" PRIu32 " %\r\n" PRIu32 "", core->cycleh, core->cyclel);
            goto restart;  // Ensure clean restart logic, clean up state if necessary before restarting
          
      case 0x5555:  // Power-off triggered
            Serial.printf("POWEROFF triggered at 0x%" PRIu32 " %\r\n" PRIu32 "", core->cycleh, core->cyclel);
            DumpState(core);  // Dump state for debugging before power off
            return;  // Exit the loop (assuming this is equivalent to shutting down)

      default:
            Serial.printf("Unknown failure with return code: %d\r\n", ret);
            break;
    }
  }
  DumpState( core );
}

void loop()
{
}
