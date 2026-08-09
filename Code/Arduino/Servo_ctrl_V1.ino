#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 10;

void setup() {
    Serial.begin(9600);

    myServo.attach(SERVO_PIN);
    myServo.write(90);
    delay(1000);
    myServo.write(0);
    delay(1000);
    myServo.write(180);
    delay(1000);
    myServo.write(90);


    Serial.println("Servo test ready.");
    Serial.println("Enter an angle from 0 to 180:");
}

void loop() {
    if (Serial.available() > 0) {

        int angle = Serial.parseInt();

        angle = constrain(angle, 0, 180);

        myServo.write(angle);

        Serial.print("Servo angle: ");
        Serial.println(angle);

        while (Serial.available()) {
            Serial.read();
        }
    }
}
