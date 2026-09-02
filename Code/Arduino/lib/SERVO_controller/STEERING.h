#include <Arduino.h>
#include <Servo.h>


// SERVO motor pins
extern int SERVO_PIN;

extern double newAngle; // Initial servo angle

extern double oldAngle; // Previous servo angle

extern double BASE_ANGLE; // Previous servo angle

extern Servo myServo; // Create a servo object

void updateServo(String newTargetAngleFactor);