#include <Arduino.h>
#include <Servo.h>
#include <PID.h>
#include <STEERING.h>


String newRPM;
String newServoAngle;
String currentChallenge;


//function declarations:
void readSerialCommand();
void applyMotor(double cmd);

//setup() and loop() functions:
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  // PID pin and interrupt setup:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderAISR, CHANGE);

  // SERVO pin and interrupt setup:            // servo object to control a servo  
  myServo.attach(SERVO_PIN);  // sets pin 10 to servo controls
  myServo.write(90);


  lastTime = millis();
  Serial.println("Motor ready");
}

void loop() {

  Serial.println("before read serial");
  readSerialCommand();
  Serial.println("after read serial");
  if (newRPM != ""){

    updateRPM(newRPM);
    newRPM = "";

  }

  if (newServoAngle != "") {

    updateServo(newServoAngle);

    newServoAngle = "";
  }
  
  runPID();

}


//helper function definitions:

void readSerialCommand() {
  if (!Serial.available()) return;
  delay(100); // wait for the entire command to be received

  Serial.println("Serial command received");

  String command = Serial.readStringUntil('\n');

  // command format will be [new desired RPM value], [new servo angle value], [which challenge we are at] (1,2,3...)
  // *** empty brackets [] mean no change to that value, so if we want to change only the RPM, we would send [new RPM], [], []

  int firstComma = command.indexOf(',');
  int secondComma = command.indexOf(',', firstComma + 1);

  Serial.println("RPM values parsed");

  newRPM = command.substring(1, firstComma);

  int startSecond = command.indexOf('[', firstComma) + 1;
  int endSecond   = command.indexOf(']', startSecond);

  Serial.println("servo values parsed");
  newServoAngle = command.substring(startSecond, endSecond);

  int startThird = command.indexOf('[', secondComma) + 1;
  int endThird   = command.indexOf(']', startThird);
  currentChallenge = command.substring(startThird, endThird);
  Serial.println("Challenge values parsed");
}
