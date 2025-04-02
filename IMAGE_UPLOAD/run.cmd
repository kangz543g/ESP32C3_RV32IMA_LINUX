windows 10
esptool.py --chip esp32c3 --port COM22 write_flash 0x200000 Image

linux 
esptool.py --chip esp32c3 --port /dev/ttyACM0 write_flash 0x200000 Image
