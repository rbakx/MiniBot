////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This is a demo program for the MiniBot using the 'LilyGO TTGO T-Display V1.1 ESP32 - with 1.14 inch TFT Display'
// board.
// The MiniBot can be 3D printed and is easy to assemble.
// For driving two servos are used, type TS90M Mini Servo - 1.6kg - Continuous.
// It drives forward and makes a turn when an obstacle is detected using an ultrasonic sensor.
// In addition a robot arm mounted on the front can be controlled with two additonal servos, type TS90M Mini Servo - 1.6kg.
// It can also be remote controlled via Bluetooth, using the 'Arduino Bluetooth Controller' app, see
// https://play.google.com/store/apps/details?id=com.appsvalley.bluetooth.arduinocontroller.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <NewPing.h>
#include "BluetoothSerial.h"
// For TFT display of the the 'LilyGO TTGO T-Display V1.1 ESP32 - with 1.14 inch TFT Display' to work, in the '.pio\libdeps\ttgo-t1\TFT_eSPI\User_Setup_Select.h':
// - Comment out the '//#include <User_Setup.h>'.
// - Uncomment the '#include <User_Setups/Setup25_TTGO_T_Display.h>'.
#include <TFT_eSPI.h> // TFT display library.
#include <Servo.h>

#define TFT_GREY 0x5AEB // New color, this line is just for demo purposes.

const int BUTTON_LEFT_PIN = 0;
const int BUTTON_RIGHT_PIN = 35;

const int SERVO_WHEEL_LEFT_PIN = 25;
const int SERVO_WHEEL_RIGHT_PIN = 26;
const int SERVO_ARM_1_PIN = 32;
const int SERVO_ARM_2_PIN = 33;
const int SERVO_GRIPPER_OPEN = 180;
const int SERVO_GRIPPER_CLOSED = 30;

const int ULTRASONIC_TRIG_PIN = 2;
const int ULTRASONIC_ECHO_PIN = 15;
const int ULTRASONIC_MAX_DISTANCE = 100;

const int ALLOWED_DISTANCE = 20;

const int MODE_RESET = 0;
const int MODE_AUTO = 1;
const int MODE_MANUAL = 2;

Servo servoWheelLeft;
Servo servoWheelRight;
Servo servoArm1;
Servo servoArm2;

TFT_eSPI tft = TFT_eSPI(135, 240);
NewPing ping(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_MAX_DISTANCE); // NewPing setup of pin and maximum distance.
BluetoothSerial SerialBT;

unsigned long backgroundColor = TFT_BLUE;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start of MiniBot demo program");

  pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);

  SerialBT.begin();

  tft.init();
  tft.setRotation(0);
  // Fill screen with random colour so we can see the effect of printing with and without
  // a background colour defined.
  tft.fillScreen(backgroundColor);

  // Set "cursor" at top left corner of display (0,0) and select font 2
  // (cursor will move to next line automatically during printing with 'tft.println'
  //  or stay on the line is there is room for the text with tft.print)
  tft.setCursor(0, 0, 2);
  // Set the font colour to be white with a black background, set text size multiplier to 1
  tft.setTextColor(TFT_WHITE, backgroundColor);
  tft.setTextSize(3);
  // We can now plot text on screen using the "print" class.
  tft.println("Hello World!");

  // Attach servo pins to servo objects.
  servoWheelLeft.attach(SERVO_WHEEL_LEFT_PIN);
  servoWheelRight.attach(SERVO_WHEEL_RIGHT_PIN);
  servoArm1.attach(SERVO_ARM_1_PIN);
  servoArm2.attach(SERVO_ARM_2_PIN);
}

void loop()
{
  static int mode = MODE_RESET;
  static unsigned long previousBackgroundColor = TFT_BLUE;
  static int previousButtonLeftState = HIGH;
  static int previousButtonRightState = HIGH;
  static int servoArm1Pos = 90;
  static int servoArm2Pos = 90;
  int distance;

  // put your main code here, to run repeatedly:
  distance = ping.ping_cm();
  // NewPing library returns 0 with false echo.
  if (distance == 0)
  {
    distance = ULTRASONIC_MAX_DISTANCE;
  }

  backgroundColor = distance < ALLOWED_DISTANCE ? TFT_RED : TFT_BLUE;

  // Only set background color when it changes to prevent flicker.
  if (backgroundColor != previousBackgroundColor)
  {
    tft.fillScreen(backgroundColor);
    tft.setTextColor(TFT_WHITE, backgroundColor);
    previousBackgroundColor = backgroundColor;
  }
  tft.setCursor(0, 0, 2);
  tft.println("Dist: \n" + String(distance) + " cm          ");

  // Switch between automatic mode and reset mode as soon as one of the onboard buttons is pressed.
  int buttonLeftState = digitalRead(BUTTON_LEFT_PIN);
  int buttonRightState = digitalRead(BUTTON_RIGHT_PIN);
  if (previousButtonLeftState == HIGH && previousButtonRightState == HIGH && (buttonLeftState == LOW || buttonRightState == LOW))
  {
    if (mode == MODE_RESET)
    {
      mode = MODE_AUTO;
    }
    else
    {
      mode = MODE_RESET;
    }
  }
  previousButtonLeftState = buttonLeftState;
  previousButtonRightState = buttonRightState;

  char cmd = '-';
  if (SerialBT.available())
  {
    // switch to manual mode as soon Bluetooth command comes in.
    mode = MODE_MANUAL;
    cmd = SerialBT.read();
    tft.setCursor(0, 100, 2);
    tft.println("Cmd: \n" + String(cmd));
  }

  if (mode == MODE_AUTO)
  {
    if (distance < ALLOWED_DISTANCE)
    {
      // Turn left or right randomly.
      if (random(2) == 0)
      {
        servoWheelLeft.write(0);
        servoWheelRight.write(0);
      }
      else
      {
        servoWheelLeft.write(180);
        servoWheelRight.write(180);
      }
      delay(700);
    }
    else
    {
      servoWheelLeft.write(180);
      servoWheelRight.write(0);
    }
  }
  else
  {
    switch (cmd)
    {
    case 'f':
      servoWheelLeft.write(180);
      servoWheelRight.write(0);
      delay(500);
      break;
    case 'b':
      servoWheelLeft.write(0);
      servoWheelRight.write(180);
      delay(500);
      break;
    case 'l':
      servoWheelLeft.write(0);
      servoWheelRight.write(0);
      break;
    case 'r':
      servoWheelLeft.write(180);
      servoWheelRight.write(180);
      break;
    case '1':
      servoArm1Pos -= 5;
      servoArm1Pos = max(0, min(180, servoArm1Pos));
      servoArm1.write(servoArm1Pos);
      break;
    case '2':
      servoArm1Pos += 5;
      servoArm1Pos = max(0, min(180, servoArm1Pos));
      servoArm1.write(servoArm1Pos);
      break;
    case '3':
      servoArm2Pos = SERVO_GRIPPER_OPEN;
      servoArm2Pos = max(0, min(180, servoArm2Pos));
      servoArm2.write(servoArm2Pos);
      break;
    case '4':
      servoArm2Pos = SERVO_GRIPPER_CLOSED;
      servoArm2Pos = max(0, min(180, servoArm2Pos));
      servoArm2.write(servoArm2Pos);
      break;
    case 'a':
      // switch back to automatic mode.
      mode = MODE_AUTO;
      break;
    default:
      servoWheelLeft.write(90);
      servoWheelRight.write(90);
      break;
    }
  }

  delay(100);
}