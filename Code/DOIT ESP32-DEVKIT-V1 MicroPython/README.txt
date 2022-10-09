Installing MicroPython firmware on 'DOIT ESP32-DEVKIT-V1'
--------------------------------------------------------------------------------------------------
(from https://docs.micropython.org/en/latest/esp32/tutorial/intro.html)
python -m esptool --chip esp32 --port COM9 write_flash -z 0x1000 esp32-idf4-20210202-v1.14.bin

Local editing main.py and other files and uploading
---------------------------------------------------
ampy -p COM9 put main.py
After reset the main.py will run.
Use PuTTY as a serial motitor.

Remote editing main.py and other files and uploading
----------------------------------------------------
Open Source IDE: https://thonny.org/


