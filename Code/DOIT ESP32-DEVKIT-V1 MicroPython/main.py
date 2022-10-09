import machine
from time import sleep_ms
import time
import bluetooth
from bluetooth import ESP32_BLE

print ("Starting Bluetooth BLE test!")
ble = ESP32_BLE("ESP32BLE")

while True:
    if bluetooth.is_ble_connected:
        ble.send('Hello')
    sleep_ms(1000)