#include <Arduino.h>
#include <Servo.h>


// SERVO motor pins
extern int SERVO_PIN;

extern int newAngle; // Initial servo angle

extern Servo myServo; // Create a servo object

void updateServo(String newTargetAngle);