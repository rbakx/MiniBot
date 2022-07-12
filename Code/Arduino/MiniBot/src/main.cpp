////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// This is a demo program for the MiniBot using the 'Arduino Uno' board.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Servo.h>

const int SERVO_LEFT_PIN = 5;
const int SERVO_RIGHT_PIN = 6;

const int ULTRASONIC_TRIG_PIN = 2;
const int ULTRASONIC_ECHO_PIN = 3;

Servo servoLeft;
Servo servoRight;

long duration; // Variable for the duration of sound wave travel.
int distance;  // Variable for the distance measurement.

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start of MiniBot demo program!");

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); // Sets the ultrasonic trigger pin as an OUTPUT.
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);  // Sets the ultrasonic echo pin as an INPUT.

  servoLeft.attach(SERVO_LEFT_PIN);   // Attaches the servo to the servo object.
  servoRight.attach(SERVO_RIGHT_PIN); // Attaches the servo to the servo object.
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Clears the trigPin condition
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.println("Distance: " + String(distance) + " cm");

  if (distance < 20)
  {
    servoLeft.write(180);
    servoRight.write(180);
    delay(1000);
  }
  else
  {
    servoLeft.write(180);
    servoRight.write(50);
  }
}