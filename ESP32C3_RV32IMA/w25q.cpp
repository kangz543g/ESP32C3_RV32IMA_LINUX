// w25q.cpp

#include "w25q.h"

// Define the variables
uint8_t rxbuf[10];
uint8_t txbuf[10];

W25Q_Data w25q_data;

//////////////////////////////////////////
// Constructor to initialize any defaults
W25Q::W25Q() {
  // Constructor can be used to initialize variables if needed
}

//////////////////////////////////////////
// Destructor if necessary (optional)
W25Q::~W25Q() {
  // Destructor can be used to clean up if necessary
}

//////////////////////////////////////////
// SPI Chip Select Pin Control
void W25Q::spi_cs_low() {
  digitalWrite(CS_PIN, LOW);  // Chip Select active (LOW)
}

void W25Q::spi_cs_high() {
  digitalWrite(CS_PIN, HIGH);  // Chip Select inactive (HIGH)
}

//////////////////////////////////////////
// Initialize SPI and configure the CS pin
void W25Q::begin() {
  pinMode(CS_PIN, OUTPUT);  // Set CS pin as output
  digitalWrite(CS_PIN, HIGH);  // Ensure CS is initially HIGH

  // Initialize the SPI communication
  SPI.begin();  
}

//////////////////////////////////////////
// Reset the W25Q chip
void W25Q::reset() {
  spi_cs_low();  
  SPI.transfer(0x66);  // Enable Reset command
  SPI.transfer(0x99);  // Reset command
  SPI.endTransaction();  // End SPI transaction
  spi_cs_high();  
  delay(100);  // Wait for reset to complete
}

//////////////////////////////////////////
// Get the Unique ID (UID) of the W25Q chip
void W25Q::getUID(uint8_t* uid) {
  uint8_t txbuf[5];  // Command buffer for SPI

  // Command to read the UID from W25Q
  txbuf[0] = 0x4B;  // Command for "Read UID"
  txbuf[1] = 0x00;   // Dummy byte 1
  txbuf[2] = 0x00;   // Dummy byte 2
  txbuf[3] = 0x00;   // Dummy byte 3
  txbuf[4] = 0x00;   // Dummy byte 4

  // Begin SPI transaction
  spi_cs_low();
  
  // Send the command to read the UID
  for (uint8_t i = 0; i < 5; i++) {
    SPI.transfer(txbuf[i]);
  }

  // Read the 8-byte UID into the provided buffer
  for (uint8_t i = 0; i < 8; i++) {
    uid[i] = SPI.transfer(0x00);  // Send dummy byte and receive the response
  }

  // End SPI transaction
  SPI.endTransaction();
  spi_cs_high();  // Deselect the chip (CS high)
}

uint32_t W25Q::get_JEDEC_ID() {
  spi_cs_low();
  SPI.transfer(W25Q_JEDEC_ID);  // JEDEC ID 요청
  uint32_t id = SPI.transfer(0) << 16;
  id |= SPI.transfer(0) << 8;
  id |= SPI.transfer(0);
  spi_cs_high();
  return id;
}

int W25Q::getMID() {
  uint32_t ID = 0;
    
  spi_cs_low();
  SPI.transfer(W25Q_device_id); // Send the command for Device ID
  SPI.transfer(0x00);   // Dummy byte to start reading
  SPI.transfer(0x00);   // Dummy byte
  SPI.transfer(0x00);   // Dummy byte
  SPI.transfer(0x00);   // Dummy byte
  
  ID |= SPI.transfer(0x00) << 16; // Read the first byte (high byte)
  ID |= SPI.transfer(0x00) << 8;  // Read the second byte
  ID |= SPI.transfer(0x00);       // Read the third byte (low byte)
  spi_cs_high();

  return ID;
}

void W25Q::send_simple_cmd(uint8_t cmd) {
  spi_cs_low();
  SPI.transfer(&cmd, 1);
  spi_cs_high();
}
 
void W25Q::write_enable() {
  send_simple_cmd(W25Q_write_enable);
  delay(1);
}
void W25Q::write_disable() {
  send_simple_cmd(W25Q_write_disable);
  delay(1);
}

void W25Q::send_cmd_read(uint8_t cmd, uint32_t address, uint8_t *buf, uint32_t len, bool is_fast) {
  uint8_t addr[4];
  int addr_len = 3;

  addr[0] = (address >> 16) & 0xFF;
  addr[1] = (address >> 8) & 0xFF;
  addr[2] = address & 0xFF;

  if (is_fast) {  // Fast Read 모드일 경우 4바이트 주소 사용
    addr[3] = 0x00;  // 더미 바이트 추가
    addr_len = 4;
  }

  spi_cs_low();
    
  SPI.transfer(cmd);         // 명령어 전송
  SPI.transfer(addr, addr_len); // 주소 전송

  // Fast Read 모드일 경우 더미 바이트(0x00) 전송 후 데이터 읽기
  if (is_fast) {
    SPI.transfer(0x00);  // 더미 바이트 전송
  }

  SPI.transfer(buf, len);   // 데이터 읽기

  spi_cs_high();
}

void W25Q::send_cmd_write(uint8_t cmd, uint32_t address, uint8_t *buf, uint32_t len) {
  uint8_t addr[3];
    
  addr[0] = (address & 0x00ff0000) >> 16;
  addr[1] = (address & 0x0000ff00) >> 8;
  addr[2] = (address & 0x000000ff);
  
  write_enable();
  spi_cs_low();
  SPI.transfer(&cmd, 1);
  SPI.transfer(addr, 3);
  SPI.transfer(buf, len);
  spi_cs_high();
}

void W25Q::send_cmd_addr(uint8_t cmd, uint32_t address) {
  uint8_t addr[3];
  addr[0] = (address & 0x00ff0000) >> 16;
  addr[1] = (address & 0x0000ff00) >> 8;
  addr[2] = (address & 0x000000ff);
  spi_cs_low();
  SPI.transfer(&cmd, 1);
  SPI.transfer(addr, 3);
  spi_cs_high();
}
 
void W25Q::send_cmd(uint8_t cmd, uint8_t *buf, uint32_t len) {
  spi_cs_low();         // Activate chip select (CS)
    
  // Send the command byte
  SPI.transfer(cmd);
    
  // Send the data buffer byte-by-byte
  if (len > 0) {
    SPI.transfer(buf, len);  // If SPI library supports it, send the entire buffer at once
  }

  spi_cs_high();        // Deactivate chip select (CS)
}

void W25Q::Read_Status_Register_1(){
  send_cmd(W25Q_read_status_r1, &w25q_data.statusRegister1, 1);
}
void W25Q::Read_Status_Register_2(){
  send_cmd(W25Q_read_status_r2, &w25q_data.statusRegister2, 1);
}
void W25Q::Read_Status_Register_3(){
  send_cmd(W25Q_read_status_r3, &w25q_data.statusRegister3, 1);
}
 
void W25Q::Write_Status_Register_1(){
  send_cmd(W25Q_write_status_r1, &w25q_data.statusRegister1, 1);
}

void W25Q::Write_Status_Register_2(){
  send_cmd(W25Q_write_status_r2, &w25q_data.statusRegister2, 1);
}

void W25Q::Write_Status_Register_3(){
  send_cmd(W25Q_write_status_r3, &w25q_data.statusRegister3, 1);
}

// Function to read all status registers
void W25Q::ReadStatusRegister(W25Q_Data* status) {
  // Read the first status register
  Read_Status_Register_1();  // This fills statusRegister1
  status->statusRegister1 = w25q_data.statusRegister1;

  // Read the second status register
  Read_Status_Register_2();  // This fills statusRegister2
  status->statusRegister2 = w25q_data.statusRegister2;

  // Read the third status register
  Read_Status_Register_3();  // This fills statusRegister3
  status->statusRegister3 = w25q_data.statusRegister3;
}

// 상태 레지스터를 읽는 함수
uint8_t W25Q::Read_Status_1(void) {
  uint8_t status;
  uint8_t command = W25Q_read_status_r1;  // 읽을 명령어

  spi_cs_low(); // Chip Select 핀 활성화

  // 상태 레지스터 읽기 명령 전송
  SPI.transfer(command);  // 상태 레지스터 읽기 명령 전송
  status = SPI.transfer(0x00);  // 응답으로 상태 레지스터 값 수신

  spi_cs_high();  // Chip Select 핀 비활성화

  return status;
}

uint8_t W25Q::Read_Status_2(void) {
  uint8_t status;
  uint8_t command = W25Q_read_status_r2;  // 읽을 명령어

  spi_cs_low(); // Chip Select 핀 활성화

  // 상태 레지스터 읽기 명령 전송
  SPI.transfer(command);  // 상태 레지스터 읽기 명령 전송
  status = SPI.transfer(0x00);  // 응답으로 상태 레지스터 값 수신

  spi_cs_high();  // Chip Select 핀 비활성화

  return status;
}

uint8_t W25Q::Read_Status_3(void) {
  uint8_t status;
  uint8_t command = W25Q_read_status_r3;  // 읽을 명령어

  spi_cs_low(); // Chip Select 핀 활성화

  // 상태 레지스터 읽기 명령 전송
  SPI.transfer(command);  // 상태 레지스터 읽기 명령 전송
  status = SPI.transfer(0x00);  // 응답으로 상태 레지스터 값 수신

  spi_cs_high();  // Chip Select 핀 비활성화

  return status;
}

uint32_t W25Q::page_to_sector_address(uint32_t pageAddress)
{
	return ((pageAddress * w25q_data.pageSize) / w25q_data.sectorSize);
}

uint32_t W25Q::page_to_block_address(uint32_t pageAddress)
{
	return ((pageAddress * w25q_data.pageSize) / w25q_data.blockSize);
}

uint32_t W25Q::data_sector_to_block_address(uint32_t sectorAddress)
{
	return ((sectorAddress * w25q_data.sectorSize) / w25q_data.blockSize);
}

uint32_t W25Q::sector_to_page_address(uint32_t sectorAddress)
{
	return (sectorAddress * w25q_data.sectorSize) / w25q_data.pageSize;
}

uint32_t W25Q::block_to_page_address(uint32_t blockAddress)
{
	return (blockAddress * w25q_data.blockSize) / w25q_data.pageSize;
}

uint32_t numBLOCK          = 256;

uint8_t W25Q::check_write_enable() {
  spi_cs_low();
  SPI.transfer(0x05);  // Read Status Register Command
  uint8_t status = SPI.transfer(0x00);  // Get status byte
  spi_cs_high();
  
  return status;
}

void W25Q::wait_busy() {
  spi_cs_low();
  SPI.transfer(0x05);  // Read Status Register Command
  while (SPI.transfer(0x00) & 0x01) {  // WIP 비트 체크
    delay(1);
  }
  spi_cs_high();
}

uint32_t W25Q::check_erase(uint32_t addr) {
  uint8_t readData[4];

  spi_cs_low();
  SPI.transfer(0x03);  // Read command
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
    
  for (int i = 0; i < 4; i++) {
    readData[i] = SPI.transfer(0x00);
  }
  spi_cs_high();

  // 4바이트 데이터를 uint32_t로 변환하여 반환
  return (uint32_t(readData[0]) << 24) |
         (uint32_t(readData[1]) << 16) |
         (uint32_t(readData[2]) << 8) |
         (uint32_t(readData[3]));
}

void W25Q::Read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
  uint8_t tData[5];
  uint32_t memAddr = (startPage * 256) + offset;

  if (numBLOCK < 512)   // Chip Size < 256Mb
  {
    tData[0] = 0x03;  // Read Command for 3-byte Address
    tData[1] = (memAddr >> 16) & 0xFF;  // MSB of the memory Address
    tData[2] = (memAddr >> 8) & 0xFF;
    tData[3] = (memAddr) & 0xFF;  // LSB of the memory Address
  }
  else
  {
    tData[0] = 0x13;  // Read Command for 4-byte Address
    tData[1] = (memAddr >> 24) & 0xFF;  // MSB of the memory Address
    tData[2] = (memAddr >> 16) & 0xFF;
    tData[3] = (memAddr >> 8) & 0xFF;
    tData[4] = (memAddr) & 0xFF;  // LSB of the memory Address
  }

  spi_cs_low();  // Pull the CS Low

  // Send read instruction and address
  if (numBLOCK < 512)
  {
    for (int i = 0; i < 4; i++) {
      SPI.transfer(tData[i]);
    }
  }
  else
  {
    for (int i = 0; i < 5; i++) {
      SPI.transfer(tData[i]);
    }
  }

  // Read the data into rData[]
  for (uint32_t i = 0; i < size; i++) {
    rData[i] = SPI.transfer(0x00);  // Send dummy 0x00 to receive data
  }

  spi_cs_high();  // Pull the CS High
}

void W25Q::FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x0B;  // enable Fast Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
		tData[4] = 0;  // Dummy clock
	}
	else
	{
		tData[0] = 0x0C;  // Fast Read with 4-Byte Address
		tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = (memAddr)&0xFF; // LSB of the memory Address
		tData[5] = 0;  // Dummy clock
	}

	spi_cs_low();  // pull the CS Low
	if (numBLOCK<512)
	{
		// send read instruction along with the 24 bit memory address
    for (int i = 0; i < 5; i++) {
      SPI.transfer(tData[i]);
    }
	}
	else
	{
		// send read instruction along with the 32 bit memory address
    for (int i = 0; i < 6; i++) {
      SPI.transfer(tData[i]);
    }
	}

	// Read the data into rData[]
  for (uint32_t i = 0; i < size; i++) {
    rData[i] = SPI.transfer(0x00);  // Send dummy 0x00 to receive data
  }

	spi_cs_high();  // pull the CS High
}

void W25Q::Erase_Sector(uint16_t numsector) {
  uint8_t tData[6];
  uint32_t memAddr = numsector * 16 * 256;  // Each sector contains 16 pages * 256 bytes

  write_enable();  // Ensure write is enabled

  if (numBLOCK < 512) {  // Chip Size < 256Mb
    tData[0] = 0x20;  // Erase sector command
    tData[1] = (memAddr >> 16) & 0xFF;  // MSB of memory address
    tData[2] = (memAddr >> 8) & 0xFF;
    tData[3] = memAddr & 0xFF; // LSB of memory address

    spi_cs_low();
    for (int i = 0; i < 4; i++) {
      SPI.transfer(tData[i]);  // Send each byte individually
    }
    spi_cs_high();
  } else {  // Use 32-bit memory address for chips >= 256Mb
    tData[0] = 0x21;  // Erase sector command for large chips
    tData[1] = (memAddr >> 24) & 0xFF;
    tData[2] = (memAddr >> 16) & 0xFF;
    tData[3] = (memAddr >> 8) & 0xFF;
    tData[4] = memAddr & 0xFF;

    spi_cs_low();
    for (int i = 0; i < 5; i++) {
      SPI.transfer(tData[i]);  // Send each byte individually
    }
    spi_cs_high();
  }

  delay(450);  // 450ms delay for sector erase

  write_disable();  // Disable write operation
}

void W25Q::Write_Page(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data) {
  uint8_t tData[266];
  uint32_t startPage = page;
  uint32_t endPage = startPage + ((size + offset - 1) / 256);
  uint32_t numPages = endPage - startPage + 1;

  uint16_t startSector = startPage / 16;
  uint16_t endSector = endPage / 16;
  uint16_t numSectors = endSector - startSector + 1;

  // 섹터 지우기
  for (uint16_t i = 0; i < numSectors; i++) {
    Erase_Sector(startSector++);
  }

  uint32_t dataPosition = 0;

  // 페이지에 데이터 쓰기
  for (uint32_t i = 0; i < numPages; i++) {
    uint32_t memAddr = (startPage * 256) + offset;
    uint16_t bytesRemaining = (size > 256) ? 256 : size;  // 한 번에 쓸 최대 바이트수는 256 바이트

    write_enable();  // 쓰기 허용

    uint32_t indx = 0;
    if (numBLOCK < 512) {  // Chip Size < 256Mb
      tData[0] = 0x02;  // page program
      tData[1] = (memAddr >> 16) & 0xFF;  // MSB of the memory Address
      tData[2] = (memAddr >> 8) & 0xFF;
      tData[3] = memAddr & 0xFF; // LSB of the memory Address
      indx = 4;
    } else {  // 256Mb 이상 칩에서 4바이트 주소 사용
      tData[0] = 0x12;  // page program with 4-Byte Address
      tData[1] = (memAddr >> 24) & 0xFF;  // MSB of the memory Address
      tData[2] = (memAddr >> 16) & 0xFF;
      tData[3] = (memAddr >> 8) & 0xFF;
      tData[4] = memAddr & 0xFF; // LSB of the memory Address
      indx = 5;
    }

    uint16_t bytesToSend = bytesRemaining + indx;

    for (uint16_t j = 0; j < bytesRemaining; j++) {
      tData[indx++] = data[dataPosition + j];
    }

    spi_cs_low();
    SPI.transfer(tData, bytesToSend);  // 데이터를 SPI로 전송
    spi_cs_high();

    startPage++;  // 다음 페이지로 이동
    offset = 0;  // 첫 페이지에서 오프셋을 0으로 리셋
    size -= bytesRemaining;  // 남은 데이터 크기 업데이트
    dataPosition += bytesRemaining;  // 데이터 포지션 업데이트

    delay(5);  // 5ms 대기
    write_disable();  // 쓰기 비활성화
  }
}

uint32_t W25Q::BytesToWrite (uint32_t size, uint16_t offset)
{
	if ((size+offset)<256) return size;
	else return 256-offset;
}     
  
uint32_t W25Q::BytesToModify (uint32_t size, uint16_t offset)
{
	if ((size+offset)<4096) return size;
	else return 4096-offset;
}

void W25Q::Write_Clean (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage  = startPage + ((size+offset-1)/256);
	uint32_t numPages = endPage-startPage+1;

	uint16_t startSector  = startPage/16;
	uint16_t endSector  = endPage/16;
	uint16_t numSectors = endSector-startSector+1;
	for (uint16_t i=0; i<numSectors; i++)
	{
		Erase_Sector(startSector++);
	}

	uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i=0; i<numPages; i++)
	{
		uint32_t memAddr = (startPage*256)+offset;
		uint16_t bytesremaining  = BytesToWrite(size, offset);
		uint32_t indx = 0;

		write_enable();

		if (numBLOCK<512)   // Chip Size<256Mb
		{
			tData[0] = 0x02;  // page program
			tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>8)&0xFF;
			tData[3] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 4;
		}

		else // we use 32bit memory address for chips >= 256Mb
		{
			tData[0] = 0x12;  // page program with 4-Byte Address
			tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>16)&0xFF;
			tData[3] = (memAddr>>8)&0xFF;
			tData[4] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 5;
		}

		uint16_t bytestosend  = bytesremaining + indx;

		for (uint16_t i=0; i<bytesremaining; i++)
		{
			tData[indx++] = data[i+dataPosition];
		}

		if (bytestosend > 250)
		{
			spi_cs_low();
			SPI.transfer(tData, 100);
			SPI.transfer(tData+100, bytestosend-100);
			spi_cs_high();
		}
		else
		{
			spi_cs_low();
			SPI.transfer(tData, bytestosend);
			spi_cs_high();
		}

		startPage++;
		offset = 0;
		size = size-bytesremaining;
		dataPosition = dataPosition+bytesremaining;

		delay(5);
		write_disable();
	}
}

void W25Q::Write (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint16_t startSector  = page/16;
	uint16_t endSector  = (page + ((size+offset-1)/256))/16;
	uint16_t numSectors = endSector-startSector+1;

  uint8_t previousData[4096];
	uint32_t sectorOffset = ((page%16)*256)+offset;
	uint32_t dataindx = 0;

  for (uint16_t i=0; i<numSectors; i++)
	{
		uint32_t startPage = startSector*16;
		FastRead(startPage, 0, 4096, previousData);

    uint16_t bytesRemaining = BytesToModify(size, sectorOffset);
		for (uint16_t i=0; i<bytesRemaining; i++)
		{
			previousData[i+sectorOffset] = data[i+dataindx];
		}

    Write_Clean(startPage, 0, 4096, previousData);

    startSector++;
		sectorOffset = 0;
		dataindx = dataindx+bytesRemaining;
		size = size-bytesRemaining;
	}
}

uint8_t W25Q::Read_Byte(uint32_t Addr) {
  uint8_t tData[5];
  uint8_t rData;

  if (numBLOCK < 512) {
    tData[0] = 0x03;  // Read Data instruction
    tData[1] = (Addr >> 16) & 0xFF;
    tData[2] = (Addr >> 8) & 0xFF;
    tData[3] = Addr & 0xFF;
  } else {
    tData[0] = 0x13;  // Read Data with 4-Byte Address instruction
    tData[1] = (Addr >> 24) & 0xFF;
    tData[2] = (Addr >> 16) & 0xFF;
    tData[3] = (Addr >> 8) & 0xFF;
    tData[4] = Addr & 0xFF;
  }

  spi_cs_low();
  if (numBLOCK < 512) {
    // Send the read command with 24-bit address
    for (int i = 0; i < 4; i++) {
      SPI.transfer(tData[i]);
    }
  } else {
    // Send the read command with 32-bit address
    for (int i = 0; i < 5; i++) {
      SPI.transfer(tData[i]);
    }
  }

  rData = SPI.transfer(0x00);  // Read data
  spi_cs_high();

  return rData;
}

void W25Q::Write_Byte(uint32_t Addr, uint8_t data) {
  uint8_t tData[6];
  uint8_t indx;

  if (numBLOCK < 512) {
    tData[0] = 0x02;  // Page program instruction
    tData[1] = (Addr >> 16) & 0xFF;
    tData[2] = (Addr >> 8) & 0xFF;
    tData[3] = Addr & 0xFF;
    tData[4] = data;
    indx = 5;
  } else {
    tData[0] = 0x12;  // Write Data with 4-Byte Address instruction
    tData[1] = (Addr >> 24) & 0xFF;
    tData[2] = (Addr >> 16) & 0xFF;
    tData[3] = (Addr >> 8) & 0xFF;
    tData[4] = Addr & 0xFF;
    tData[5] = data;
    indx = 6;
  }

  // Ensure the chip is in write mode
  write_enable();  // Enable writing to the flash memory

  // Select the chip (pull CS low)
  spi_cs_low();

  // Send the write command and data
  SPI.transfer(tData, indx);

  // Deselect the chip (pull CS high)
  spi_cs_high();

  // Wait for the write to complete (10 ms delay is usually enough)
  delay(10);

  // Disable the write mode (optional, but good practice)
  write_disable();
}

void W25Q::Float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }

}

float W25Q::Bytes2Float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}

uint8_t tempBytes[4];
void W25Q::Write_NUM (uint32_t page, uint16_t offset, float data)
{
	Float2Bytes(tempBytes, data);

	/* Write using sector update function */
	Write(page, offset, 4, tempBytes);
}

float W25Q::Read_NUM (uint32_t page, uint16_t offset)
{
	uint8_t rData[4];
	Read(page, offset, 4, rData);
	return (Bytes2Float(rData));
}

void W25Q::Read_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
	uint8_t data8[size*4];
	uint32_t indx = 0;

	FastRead(page, offset, size*4, data8);

	for (uint32_t i=0; i<size; i++)
	{
		data[i] = (data8[indx++]) | (data8[indx++]<<8) | (data8[indx++]<<16) | (data8[indx++]<<24);
	}
}

void W25Q::Write_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
	uint8_t data8[size*4];
	uint32_t indx = 0;

	for (uint32_t i=0; i<size; i++)
	{
		data8[indx++] = data[i]&0xFF;   // extract LSB
		data8[indx++] = (data[i]>>8)&0xFF;
		data8[indx++] = (data[i]>>16)&0xFF;
		data8[indx++] = (data[i]>>24)&0xFF;
	}

	Write(page, offset, indx, data8);
}

DRESULT W25Q::disk_initialize() {
  // SPI initialization
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);  // CS pin is set HIGH by default (inactive)
  
  // SPI communication settings (clock speed and mode)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  // 1MHz, MSB first, mode0

  delay(1000);  // Initialization delay
  
  reset();  // Reset the device (assuming the reset function is implemented)
  get_JEDEC_ID();  // Get the JEDEC ID to identify the chip
  
  w25q_data.lock = 1;  // Lock the flash memory for exclusive access
  delay(100);  // Short delay for stability
  
  // Check JEDEC ID and set block count accordingly
  switch (w25q_data.jedec_id & 0x000000FF) {
    case 0x20: // w25q512
      w25q_data.blockCount = 1024;
      break;
    case 0x19: // w25q256
      w25q_data.blockCount = 512;
      break;
    case 0x18: // w25q128
      w25q_data.blockCount = 256;
      break;
    case 0x17: // w25q64
      w25q_data.blockCount = 128;
      break;
    case 0x16: // w25q32
      w25q_data.blockCount = 64;
      break;
    case 0x15: // w25q16
      w25q_data.blockCount = 32;
      break;
    case 0x14: // w25q80
      w25q_data.blockCount = 16;
      break;
    case 0x13: // w25q40
      w25q_data.blockCount = 8;
      break;
    case 0x12: // w25q20
      w25q_data.blockCount = 4;
      break;
    case 0x11: // w25q10
      w25q_data.blockCount = 2;
      break;
    default:
      w25q_data.lock = 0;
      return RES_ERROR;  // Return RES_ERROR if JEDEC ID is not recognized
  }
  
  // Set up memory sizes based on block count
  w25q_data.pageSize = 256;
  w25q_data.sectorSize = 0x1000;  // 4KB sector size
  w25q_data.sectorCount = w25q_data.blockCount * 16;
  w25q_data.pageCount = (w25q_data.sectorCount * w25q_data.sectorSize) / w25q_data.pageSize;
  w25q_data.blockSize = w25q_data.sectorSize * 16;
  w25q_data.capacityKB = (w25q_data.sectorCount * w25q_data.sectorSize) / 1024;
  
  // Get the UID from the flash memory (make sure this function is defined properly)
  getUID(w25q_data.uuid);  // Get UID into the w25q_data.uuid array
  
  // Continue with status register reads
  Read_Status_Register_1();
  Read_Status_Register_2();
  Read_Status_Register_3();
  
  w25q_data.lock = 0;  // Unlock the flash memory after initialization
  
  // Correct the STA_NOINIT flag manipulation (clear STA_NOINIT flag)
  w25q_data.Stat = static_cast<DRESULT>(w25q_data.Stat & ~STA_NOINIT);  // Mask STA_NOINIT correctly
  
  return RES_OK;  // Return RES_OK after successful initialization
}
