## 2025년 uc-rv32ima

RISC-V 에뮬레이터를 사용하여 다양한 MCU에서 리눅스를 실행합니다. 이 프로젝트는 CNLohr의 mini-rv32ima RISC-V
 에뮬레이터 코어를 사용하여 ESP32C3 등과 같은 다양한 MCU에서 리눅스를 실행하는 것입니다. 
 현재는 ESP32C3만 테스트되었습니다. 이론적으로 ESP 시리즈 SoC가 지원되지만, ESP32C3를 제외한 다른 SoC의 개발 
 보드는 없어 테스트하지 못했습니다.

## 목적
재미를 위해

CNLohr가 mini-rv32ima를 작성했을 때, 그는 ESP32C3에서 리눅스를 실행하고자 했다고 했고, 저도 그 아이디어에 
흥미를 느꼈습니다.

## ESP32C3에서 리눅스를 실행하려면 무엇이 부족한가?
RV32-IMC CPU 코어가 하나 필요합니다. 이는 리눅스 커널에서 'A' 확장자를 제거하는 패치를 적용하여 해결할 수 있습니다.

MMU가 없습니다. 이는 NOMMU를 사용하여 해결할 수 있습니다.

메모리가 부족합니다. 4MB 플래시와 400KB SRAM만 있습니다. 이는 SPI PSRAM 칩을 추가하여 해결할 수 있습니다. 그러나,

ESP32C3에서 PSRAM에서 직접 코드를 실행하는 방법을 ESP32S3처럼 동작시키는 방법을 찾지 못했습니다. 
이 방법을 아는 사람이 있다면 알려주시면 정말 감사하겠습니다;) 
그렇게 되면 ESP32C3에서 리눅스를 직접 실행할 수 있게 됩니다!

현재의 해결책은 RISC-V IMA 에뮬레이터를 사용하고, PSRAM을 에뮬레이터의 주 시스템 메모리로 사용하는 것입니다.

## 동작 방식
8MB SPI PSRAM 칩을 시스템 메모리로 사용합니다. 시작 시 PSRAM을 초기화하고, 리눅스 커널 이미지(초기 RAM 파일 시스템이 
포함되어 rootfs로 사용됨)와 장치 트리 바이너리를 플래시에서 PSRAM으로 로드한 후 부팅을 시작합니다.

성능을 개선하기 위해 간단한 캐시를 구현했습니다. 이를 통해 리눅스 부팅 중에 95.1% 캐시 적중률을 달성했습니다:

4KB 캐시
2-way 집합 연관
64B 캐시라인
LRU (Least Recently Used)

tvlad1234의 pico-rv32ima와의 차이점
ESP32C3의 uc-rv32ima에서 지원될 예정입니다.

## 8MB SPI PSRAM 하나만 필요

간단한 캐시 메커니즘이 구현되어 성능이 훨씬 향상됨

SD카드가 필요 없음

## 요구 사항
ESP32C3 개발 보드

8메가바이트(64Mbit) SPI PSRAM 칩(PSRAM64H 사용)과 이를 SOP8 어댑터 보드에 납땜한 후, 어댑터 보드를
 MCU 개발 보드와 듀폰 선으로 올바르게 연결

## 사용 방법
SPI PSRAM 칩 핀이 MCU 보드와 제대로 연결되었는지 확인합니다. 예를 들어, PSRAM이 ESP32C3의 아래 GPIO와 
연결되어야 하며, VCC와 GND 핀도 잊지 말고 연결해야 합니다:

MOSI: GPIO6

MISO: GPIO5

SCLK: GPIO4

CS1: GPIO9  // PSRAM64H (8MB)

CS2: GPIO10 // W25Q128 (16MB) 

## esp-idf 환경에서 uc-rv32ima를 빌드

esptool을 사용하여 보드의 플래시에 이미지를 작성한 후 보드를 리셋합니다:

esptool.py --chip esp32c3 --port /dev/ttyAMC0 write_flash 0x200000 Image 

1초 이내에 USB CDC 콘솔에 리눅스 커널 메시지가 출력되기 시작합니다. 리셋 버튼을 누른 후 리눅스 셸이
 실행되는 데 약 1분 20초가 걸립니다.

스크린샷 2025-04-02 16-26-31.png
(https://github.com/kangz543g/ESP32C3_RV32IMA_LINUX/blob/main/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202025-04-02%2016-26-31.png)

riscv_emufun (mini-rv32ima)
https://github.com/cnlohr/mini-rv32ima


