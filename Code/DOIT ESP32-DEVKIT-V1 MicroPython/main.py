import esp32
import machine
from machine import Pin
from time import sleep_ms
import time
from machine import deepsleep
from bluetooth import ESP32_BLE
from servo import Servo
from ultrasonic import HCSR04

LED_BUILTIN_PIN = 2
US_TRIGGER_PIN = 25
US_ECHO_PIN = 26
SERVO_LEFT_PIN = 13
SERVO_RIGHT_PIN = 5
MOTION_PIN = 35
    
led_builtin = Pin(LED_BUILTIN_PIN, Pin.OUT)
led_builtin.value(1) # Turn onboard LED on
motion = Pin(MOTION_PIN, Pin.IN)
servo_left_pin = Pin(SERVO_LEFT_PIN)
servo_right_pin = Pin(SERVO_RIGHT_PIN)
my_servo_left = Servo(servo_left_pin)
my_servo_right = Servo(servo_right_pin)

us_sensor = HCSR04(trigger_pin=US_TRIGGER_PIN, echo_pin=US_ECHO_PIN, echo_timeout_us=100000)
esp32.wake_on_ext0(pin=motion, level = esp32.WAKEUP_ANY_HIGH)

print ("Starting MiniBot with MicroPython!")
#ble = ESP32_BLE("ESP32BLE")

start_time = time.ticks_ms()
while True:
#     if ble.is_ble_connected:
#         #ble.send('Hello')
#         msg = ble.receive()
#         if msg != "":
#             print (msg)
#         if msg == "f":
#             print ("forward")
#         elif msg == "b":
#             print ("backward")       
    current_time = time.ticks_ms()
    distance = us_sensor.distance_cm()
    distance = int(distance)
    print(distance)
    if distance > 20:
        my_servo_left.write_angle(180)
        my_servo_right.write_angle(180)
    else:
        my_servo_left.write_angle(0)
        my_servo_right.write_angle(0)
    time.sleep_ms(1000)
    if time.ticks_ms() - start_time > 10000:
        led_builtin.value(0) # Turn onboard LED off
        deepsleep()

