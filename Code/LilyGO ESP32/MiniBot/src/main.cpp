////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This is a demo program for the MiniBot using the 'LilyGO TTGO T-Display V1.1 ESP32 - with 1.14 inch TFT Display'
// board.
// The MiniBot can be 3D printed and is easy to assemble.
// For driving two servos are used, type TS90D Mini Servo - 1.6kg - Continuous.
// In addition a robot gripper mounted on the front can be controlled with two additonal servos,
// type TS90M Mini Servo - 1.6kg (metal gearing and no continuous rotation).
// Three ultrasonic sensors type HC-SR04 are supported which makes following an object possible.
// It has the following modes:
// MODE_RESET:
//     The MiniBot stands still waiting for another MODE, initiated via Bluetooth or by pressing a button.
// MODE_MANUAL:
//     The MiniBot can be controlled manually via Bluetooth using the 'Arduino Bluetooth Controller' app, see
//     https://play.google.com/store/apps/details?id=com.appsvalley.bluetooth.arduinocontroller.
// MODE_AUTO:
//     The MiniBot drives and avoids obstacles using one ultrasonic sensor.
// MODE_FOLLOW:
//     An object can be followed using three ultrasonic sensors.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
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
const int SERVO_GRIPPER_1_PIN = 32;
const int SERVO_GRIPPER_2_PIN = 33;
const int SERVO_GRIPPER_OPEN = 180;
const int SERVO_GRIPPER_CLOSED = 30;
const int SERVO_GRIPPER_OPEN_CLOSE_DELTA = 1;
const int SERVO_GRIPPER_UP_DOWN_DELTA = 1;

const int NOF_ULTRASONIC_SENSORS = 3;
// All ultrasonic sensors are triggered at once as their trigger pins are connected to the same GPIO pin.
const int ULTRASONIC_TRIG_PIN = 2;
const int ULTRASONIC_0_ECHO_PIN = 15;
const int ULTRASONIC_1_ECHO_PIN = 13;
const int ULTRASONIC_2_ECHO_PIN = 12;
const int ULTRASONIC_MAX_DISTANCE = 100;
const int ULTRASONIC_MAX_JUMP = 20;
const int ULTRASONIC_DELAY_MS = 100;

const int ALLOWED_DISTANCE = 20;

const int TURNTIME = 700;

const int MODE_RESET = 0;
const int MODE_MANUAL = 1;
const int MODE_AUTO = 2;
const int MODE_FOLLOW = 3;

Servo servoWheelLeft;
Servo servoWheelRight;
Servo servoGripperUpDown;
Servo servoGripperOpenClose;

TFT_eSPI tft = TFT_eSPI(135, 240);
BluetoothSerial SerialBT;

unsigned long backgroundColor = TFT_BLUE;

volatile int distanceMicroSeconds[NOF_ULTRASONIC_SENSORS] = {0, 0, 0};

void IsrEcho0()
{
  static unsigned long startTimeMicroSeconds;
  int level = digitalRead(ULTRASONIC_0_ECHO_PIN);
  if (level == HIGH)
  {
    startTimeMicroSeconds = micros();
  }
  else
  {
    distanceMicroSeconds[0] = micros() - startTimeMicroSeconds;
  }
}

void IsrEcho1()
{
  static unsigned long startTimeMicroSeconds;
  int level = digitalRead(ULTRASONIC_1_ECHO_PIN);
  if (level == HIGH)
  {
    startTimeMicroSeconds = micros();
  }
  else
  {
    distanceMicroSeconds[1] = micros() - startTimeMicroSeconds;
  }
}

void IsrEcho2()
{
  static unsigned long startTimeMicroSeconds;
  int level = digitalRead(ULTRASONIC_2_ECHO_PIN);
  if (level == HIGH)
  {
    startTimeMicroSeconds = micros();
  }
  else
  {
    distanceMicroSeconds[2] = micros() - startTimeMicroSeconds;
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start of MiniBot demo program");

  pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  // Ultrasonic echo pins to INPUT_PULLUP to prevent spurious interrupts when not all sensors are connected.
  pinMode(ULTRASONIC_0_ECHO_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_1_ECHO_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_2_ECHO_PIN, INPUT_PULLUP);
  attachInterrupt(ULTRASONIC_0_ECHO_PIN, IsrEcho0, CHANGE);
  attachInterrupt(ULTRASONIC_1_ECHO_PIN, IsrEcho1, CHANGE);
  attachInterrupt(ULTRASONIC_2_ECHO_PIN, IsrEcho2, CHANGE);

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
  tft.setTextSize(2);
  // We can now plot text on screen using the "print" class.
  tft.println("Hello World!");

  // Attach servo pins to servo objects.
  servoWheelLeft.attach(SERVO_WHEEL_LEFT_PIN);
  servoWheelRight.attach(SERVO_WHEEL_RIGHT_PIN);
  servoGripperUpDown.attach(SERVO_GRIPPER_1_PIN);
  servoGripperOpenClose.attach(SERVO_GRIPPER_2_PIN);
}

void loop()
{
  // put your main code here, to run repeatedly:
  static int mode = MODE_RESET;
  static unsigned long previousBackgroundColor = TFT_BLUE;
  static int previousButtonLeftState = HIGH;
  static int previousButtonRightState = HIGH;
  static int servoGripperUpDownPos = 90;
  static int servoGripperOpenClosePos = 90;
  static int ServoGripperUpDownDelta = 0;
  static int ServoGripperOpenCloseDelta = 0;
  static unsigned long previousMillis = 0;
  static unsigned long startTurnTime = 0;
  static bool turning = false;
  static int previousMeasuredDistance[NOF_ULTRASONIC_SENSORS] = {ULTRASONIC_MAX_DISTANCE, ULTRASONIC_MAX_DISTANCE, ULTRASONIC_MAX_DISTANCE};
  static int distance[NOF_ULTRASONIC_SENSORS] = {ULTRASONIC_MAX_DISTANCE, ULTRASONIC_MAX_DISTANCE, ULTRASONIC_MAX_DISTANCE};
  int measuredDistance[NOF_ULTRASONIC_SENSORS];

  // Read out ultrasonic sensors and trigger for the next read.
  // Multiple ultrasonic sensors are used which widens the FOV and makes it possible to approach a target or to follow an object.
  // In the FOLLOW mode only one object is involved and we have to determine which sensor has the object closest by to determine which direction to steer.
  // In this case triggering all sensors at the same time is the most accurate solution when the MiniBot is moving.
  // When more objects are involved and we want to get a distance map it is better to trigger the sensors one by one to prevent interference.
  // Reading the length of the echo pulse is done using an interrupt for each sensor.
  // The results is accurate distance measurements and no use of a delay (except for the short 10 µs trigger pulse). This in contrary to for example the NewPing library, which blocks waiting for the end of the echo pulse.
  //
  // According to the datasheet https://www.elecrow.com/download/HC_SR04%20Datasheet.pdf, he HC-SR04 ultrasonic sensor normally should have an echo pulse timeout of 38 ms if no object is in sight.
  // This means the echo signal will last maximum appr. 38 ms which corresponds to appr 13 meter back and forth or 6.5 meter distance.
  // However, not all HC-SR04 clones have this timeout. The echo pulse can last > 200 ms if no object is in sight and right after that give a false short echo pulse (< 1 ms) when triggered.
  // The result is false short distance readings when no object is in sight.
  // Therefore single distance measurement jumps > ULTRASONIC_MAX_JUMP are filtered out.

  // Wait for ULTRASONIC_DELAY_MS before triggering the ultrasonic sensors to prevent false echoes.
  if (millis() - previousMillis > ULTRASONIC_DELAY_MS)
  {
    for (int i = 0; i < NOF_ULTRASONIC_SENSORS; i++)
    {
      if (distanceMicroSeconds[i] != 0)
      {
        measuredDistance[i] = distanceMicroSeconds[i] / 58;

        if (measuredDistance[i] > ULTRASONIC_MAX_DISTANCE)
        {
          measuredDistance[i] = ULTRASONIC_MAX_DISTANCE;
        }
        if (abs(measuredDistance[i] - previousMeasuredDistance[i]) <= ULTRASONIC_MAX_JUMP)
        {
          distance[i] = measuredDistance[i];
        }
        previousMeasuredDistance[i] = measuredDistance[i];
        distanceMicroSeconds[i] = 0; // Indicate that this measurement has been handled. It will be set again by the ISR at the next echo pulse.
      }
    }
    // Trigger the ultrasonic sensors for the next measurement.
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    previousMillis = millis();
  }

  backgroundColor = distance[1] < ALLOWED_DISTANCE ? TFT_RED : TFT_BLUE;

  // Only set background color when it changes to prevent flicker.
  if (backgroundColor != previousBackgroundColor)
  {
    tft.fillScreen(backgroundColor);
    tft.setTextColor(TFT_WHITE, backgroundColor);
    previousBackgroundColor = backgroundColor;
  }
  tft.setCursor(0, 0, 2);
  tft.println("Distances:\n" + String(distance[0]) + " cm          ");
  tft.setCursor(0, tft.fontHeight() * 2, 2);
  tft.println(String(distance[1]) + " cm          ");
  tft.setCursor(0, tft.fontHeight() * 3, 2);
  tft.println(String(distance[2]) + " cm");

  // Switch between modes when one of the onboard buttons is pressed.
  int buttonLeftState = digitalRead(BUTTON_LEFT_PIN);
  int buttonRightState = digitalRead(BUTTON_RIGHT_PIN);
  if (previousButtonLeftState == HIGH && buttonLeftState == LOW)
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
  if (previousButtonRightState == HIGH && buttonRightState == LOW)
  {
    if (mode == MODE_RESET)
    {
      mode = MODE_FOLLOW;
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
    tft.setCursor(0, tft.fontHeight() * 4, 2);
    tft.println("Cmd: \n" + String(cmd));
  }

  if (mode == MODE_RESET)
  {
    servoWheelLeft.write(90);
    servoWheelRight.write(90);
    servoGripperUpDown.write(60);
    servoGripperOpenClose.write(90);
  }
  else if (mode == MODE_MANUAL)
  {
    switch (cmd)
    {
    case 'f':
      servoWheelLeft.write(180);
      servoWheelRight.write(0);
      break;
    case 'b':
      servoWheelLeft.write(0);
      servoWheelRight.write(180);
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
      ServoGripperUpDownDelta = SERVO_GRIPPER_UP_DOWN_DELTA;
      break;
    case '2':
      ServoGripperUpDownDelta = -SERVO_GRIPPER_UP_DOWN_DELTA;
      break;
    case '3':
      ServoGripperOpenCloseDelta = SERVO_GRIPPER_OPEN_CLOSE_DELTA;
      break;
    case '4':
      ServoGripperOpenCloseDelta = -SERVO_GRIPPER_OPEN_CLOSE_DELTA;
      break;
    case 'A':
      // switch back to automatic mode.
      mode = MODE_AUTO;
      break;
    case 'F':
      // switch back to target mode.
      mode = MODE_FOLLOW;
      break;
    case '0':
      // Forward, backward, left, write button release, stop motion.
      servoWheelLeft.write(90);
      servoWheelRight.write(90);
      break;
    case 'o':
      // Gripper button release, stop motion.
      ServoGripperUpDownDelta = 0;
      ServoGripperOpenCloseDelta = 0;
      break;
    default:
      break;
    }
    if (ServoGripperUpDownDelta != 0)
    {
      servoGripperUpDownPos += ServoGripperUpDownDelta;
      servoGripperUpDownPos = max(0, min(180, servoGripperUpDownPos));
      servoGripperUpDown.write(servoGripperUpDownPos);
    }
    if (ServoGripperOpenCloseDelta != 0)
    {
      servoGripperOpenClosePos += ServoGripperOpenCloseDelta;
      servoGripperOpenClosePos = max(0, min(180, servoGripperOpenClosePos));
      servoGripperOpenClose.write(servoGripperOpenClosePos);
    }
  }
  else if (mode == MODE_AUTO)
  {
    // Do not turn using a delay otherwise the distance measurement will not be updated.
    if (turning == false && distance[1] < ALLOWED_DISTANCE)
    {
      // Start the turn.
      turning = true;
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
      startTurnTime = millis();
    }
    else if (turning == true)
    {
      // Turning in progress, check if turn time has passed.
      if (millis() - startTurnTime > TURNTIME)
      {
        turning = false;
      }
    }
    else
    {
      // Straight forward.
      servoWheelLeft.write(180);
      servoWheelRight.write(0);
    }
  }
  else if (mode == MODE_FOLLOW)
  {
    if (distance[1] < ALLOWED_DISTANCE)
    {
      // Stop.
      servoWheelLeft.write(90);
      servoWheelRight.write(90);
    }
    else if (distance[1] > distance[0])
    {
      // Turn left.
      servoWheelLeft.write(90);
      servoWheelRight.write(70);
    }
    else if (distance[1] > distance[2])
    {
      // Turn right.
      servoWheelLeft.write(110);
      servoWheelRight.write(90);
    }
    else
    {
      // Straight forward.
      servoWheelLeft.write(180);
      servoWheelRight.write(0);
    }
  }
}