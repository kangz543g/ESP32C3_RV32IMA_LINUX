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

#include <SPI.h>
#include "w25q.h"

#include "default64mbdtc.h"

#define LED_PIN       8

uint8_t buf[256];     // Read data buffer
uint16_t n;           // Number of data read

// Declare the flash object of W25Q class globally
W25Q flash;

#define GPIO_MOSI	    6 
#define GPIO_MISO	    5 
#define GPIO_SCLK	    4 
#define GPIO_CS1      9  // PSRAM64H (8MB)
#define GPIO_CS2     10  // W25Q64 (8MB)

#define SPI_HOST_ID	  SPI2_HOST
#define SPI_FREQ	    40000000 // 80MHz

#define CMD_WRITE	    0x02
#define CMD_READ	    0x03
#define CMD_FAST_READ	0x0b
#define CMD_RESET_EN	0x66
#define CMD_RESET	    0x99
#define CMD_READ_ID	  0x9f

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
#define MINIRV32_POSTEXEC(pc, ir, retval) { if (retval > 0) {  retval = HandleException(ir, retval); } }
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

int psram_init(void)
{
	esp_err_t ret;
	uint8_t id[6];

	gpio_reset_pin((gpio_num_t)GPIO_CS1);
	gpio_set_direction((gpio_num_t)GPIO_CS1, GPIO_MODE_OUTPUT);
	gpio_set_level((gpio_num_t)GPIO_CS1, 1);

	spi_bus_config_t spi_bus_config = {
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = GPIO_MISO,
		.sclk_io_num = GPIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0,
	};

	Serial.printf("SPI_HOST_ID = %d\r\n", (gpio_num_t)SPI_HOST_ID);
 	ret = spi_bus_initialize(SPI_HOST_ID, &spi_bus_config, SPI_DMA_CH_AUTO);
	Serial.printf("spi_bus_initialize = %d\r\n", ret);
	if (ret != ESP_OK)
		return -1;

	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
	devcfg.clock_speed_hz = SPI_FREQ;
	devcfg.spics_io_num = -1;
	devcfg.queue_size = 1;
	devcfg.command_bits = 8;
	devcfg.address_bits = 24;

	ret = spi_bus_add_device(SPI_HOST_ID, &devcfg, &handle);
	Serial.printf("spi_bus_add_device = %d\r\n", ret);
	if (ret != ESP_OK)
		return -1;

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
	if (ret != ESP_OK)
		return -1;
  // ESP-IDF PSRAM ID: 0d5d52d26fd0(80MHZ),ARDUINO ESP32 PSRAM ID: 0d5d50b1c027(40MHZ)
	Serial.printf("PSRAM ID: %02x%02x%02x%02x%02x%02x\r\n", id[0], id[1], id[2], id[3], id[4], id[5]);
	return 0;
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

#define ESP_FLASH  1
#define W25Q_FLASH 0

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
#endif

#if W25Q_FLASH
#define DMA_BUF_SIZE 64
uint8_t dmabuf[DMA_BUF_SIZE];  // 전역 버퍼 선언

#define kernel_start  0x200000
#define kernel_end    0x550D0F     // 3476752 bytes

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
    flash.Read(flashaddr / 256, flashaddr % 256, DMA_BUF_SIZE, flash_buf);
    
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
    flash.Read(flashaddr / 256, flashaddr % 256, flen, flash_buf);
    
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

  // LED 켜기 (이미지 로딩 시작)
  digitalWrite(LED_PIN, LOW);
  
  Serial.printf("Loading kernel Image (%ld bytes) from flash:%lx into psram:%lx\r\n", flen, flashaddr, addr);

  while (flen >= 64) {
    flash.Read(flashaddr / 256, flashaddr % 256, 64, dmabuf);
    psram_write(addr, dmabuf, 64);
    addr += 64;
    flashaddr += 64;
    flen -= 64;
  }
  if (flen) {
    flash.Read(flashaddr / 256, flashaddr % 256, flen, dmabuf);
    psram_write(addr, dmabuf, flen);
  }

  // LED 끄기 (이미지 로딩 완료)
  digitalWrite(LED_PIN, HIGH);
  
  return 0;
}
#endif

////////////////////////////////////////////////////////////////

struct MiniRV32IMAState *core;  // Declare core as a pointer

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

uint32_t corebase = 0;
int coresize32 = 0;
    
#define TIME_DIVISOR 1

void setup()
{
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
  
    
#if 1  
  // Initialize the W25Q flash memory
  if (flash.disk_initialize() < 0) {
    Serial.println("failed to init W25Q128");
    return;
  }
  
  // Get and print the JEDEC ID to confirm chip identification
  uint32_t jedec_id = flash.get_JEDEC_ID();
  Serial.printf("JEDEC ID: %08X\r\n", jedec_id);

  flash.getUID(w25q_data.uuid);  // Get UID into the w25q_data.uuid array

  printf("UID: ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("%s%02X", (i > 0) ? ":" : "", w25q_data.uuid[i]);
  }
  Serial.println("");
#endif
  
  Serial.println("psram init");

  // Initialize PSRAM
  if (psram_init() < 0) {
    Serial.println("failed to init psram");
    return;
  }  
  Serial.println("");

  if (load_images(RAM_SIZE, NULL) < 0) {
     return;  // Exit if loading the image fails
  }

/*
  if (load_images(RAM_SIZE, NULL) == 0) {
    Serial.println("Kernel image loaded successfully");
        
    if (verify_images(RAM_SIZE) == 0) {
      Serial.println("Memory verification passed!");
    } else {
      Serial.println("Memory verification failed!");
    }
  } else {
    Serial.println("Kernel image loading failed!");
  }
*/  
restart:
     
  // Initialize emulator struct
  core = (struct MiniRV32IMAState *)malloc(sizeof(struct MiniRV32IMAState));
  memset(core, 0, sizeof(struct MiniRV32IMAState));
    
  // Calculate variables
  corebase = RAM_SIZE - sizeof(struct MiniRV32IMAState); // Base address of core struct
  coresize32 = sizeof(struct MiniRV32IMAState) / 4;      // Number of UInt32 in core struct

  
  // Clear the struct
  for (int i = 0; i < coresize32; i++) {
    *(uint32_t*)((uint8_t*)core + 4*i) = 0;
  }
  
  int dtb_ptr = 0;
     
  dtb_ptr = RAM_SIZE - sizeof(default64mbdtb) - sizeof( struct MiniRV32IMAState );
    
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
            printf("System restart triggered at 0x%" PRIu32 " %\r\n" PRIu32 "", core->cycleh, core->cyclel);
            goto restart;  // Ensure clean restart logic, clean up state if necessary before restarting
          
      case 0x5555:  // Power-off triggered
            printf("POWEROFF triggered at 0x%" PRIu32 " %\r\n" PRIu32 "", core->cycleh, core->cyclel);
            DumpState(core);  // Dump state for debugging before power off
            return;  // Exit the loop (assuming this is equivalent to shutting down)

      default:
            printf("Unknown failure with return code: %d\r\n", ret);
            break;
    }
  }
  DumpState( core );
}

void loop()
{
}
