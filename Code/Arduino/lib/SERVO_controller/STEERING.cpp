#include <Arduino.h>
#include <STEERING.h>

int SERVO_PIN = 10;

int newAngle; // Initial servo angle

Servo myServo; // Create a servo object

void updateServo(String newTargetAngle) {
    newAngle = newTargetAngle.toInt();
    Serial.print("New Servo angle target: ");
    Serial.println(newAngle);

    myServo.write(newAngle);
}