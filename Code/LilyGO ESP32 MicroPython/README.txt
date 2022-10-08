Installing MicroPython firmware on 'LilyGO TTGO T-Display V1.1 ESP32 - with 1.14 inch TFT Display'
--------------------------------------------------------------------------------------------------
(from https://www.instructables.com/Installing-Loboris-lobo-Micropython-on-ESP32-With-/)
python -m esptool --chip esp32 --port COM10 --before default_reset --after no_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 bootloader.bin 0xf000 phy_init_data.bin 0x10000 MicroPython.bin 0x8000 partitions_mpy.bin

Uploading main.py and other files
---------------------------------
ampy -p COM10 put main.py
After reset the main.py will run.

Use PuTTY as a serial motitor.


