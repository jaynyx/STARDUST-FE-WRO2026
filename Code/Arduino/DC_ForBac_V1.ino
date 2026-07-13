// TB6612FNG pins

#define PWMA 5
#define AIN1 7
#define AIN2 8
#define STBY 9

// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

// ---------- Variables encodeur ----------
volatile long encoderCount = 0;
long currentCount;
long previousCount;
long deltaPulses;
double PPR_DC_ENGINE = 422.4;  //according to the data sheet of the DC and the gearbox ratio of 1:9,6

// ---------- Paramètres PID ----------
double Kp = 1.0;
double Ki = 0.2;
double Kd = 0.1;

double setpointRPM = 0;   // target  RPMS
double setpointPPST = 0;  // target pulse per sample time
double input = 0;
double output = 0;

double previousError = 0;
double integral = 0;

unsigned long lastTime = 0;
unsigned long now;
const unsigned long sampleTime = 20;  // ms (50 Hz)
double dt;

void setup() {

  Serial.begin(9600);

  // Motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);  // enable TB6612


  // Encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // x4 quadrature readings for better RPM resolution
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderBISR, CHANGE);

  lastTime = millis();

  Serial.println("Motor ready");
}


void loop() {  // pid loop


  if (Serial.available()) {  // gets the new RPM value or command input from the raspberry pi through serial communication ***all printing is for debugging purpous***

    String command = Serial.readStringUntil('\n');
    Serial.print("Command received: [");
    Serial.print(command);
    Serial.println("]");


    if (command.startsWith("RPM:")) {  // Raspberry pi will send string starting with "DESIRED RPM:"

      setpointRPM = command.substring(5).toInt();  // get the new RPM value to feed to pid controller

      RPMconversion();  // converts the given RPM in pulse per sample size

      String setpointRPMtxt = String(setpointRPM);
      Serial.print("New RPM setpoint: " + setpointRPMtxt + "\n");
      String setpointPPSTtxt = String(setpointPPST);
      Serial.print("New PPST setpoint: " + setpointPPSTtxt + "\n");

      Serial.println("entering pid");
    }
  }
  //applyMotor(255);
  pid();
  

  Serial.println(deltaPulses);
}

void pid() {

  now = millis();  // constant check for new time values

  if (now - lastTime >= sampleTime) {
    dt = (now - lastTime) / 1000.0;  // in seconds

    noInterrupts();
    currentCount = encoderCount;
    interrupts();

    deltaPulses = currentCount - previousCount;

    previousCount = currentCount;

    input = deltaPulses;

    // ---- Calcul PID ----
    double error = setpointPPST - input;               // P
    integral += error * dt;                            // I
    double derivative = (error - previousError) / dt;  // D

    output = Kp * error + Ki * integral + Kd * derivative;

    previousError = error;
    lastTime = now;

    // ---- Application à la sortie moteur ----
    applyMotor(output);

 /*   // Debug
    Serial.print("Position: ");
    Serial.print(input);
    Serial.print("  Erreur: ");
    Serial.print(error);
    Serial.print("  Sortie: ");
    Serial.println(output);
    */
  }
}





// ---------- Fonction moteur ----------
void applyMotor(double cmd) {
  cmd = constrain(cmd, -255, 255);

  if (cmd >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    cmd = -cmd;
  }

  analogWrite(PWMA, (int)cmd);
}

// ---------- Interruptions encodeur (quadrature x4) ----------
/* 
   since the clockwise sequence will be AB ---> 00 10 11 01, if A just changed and AB = 11, that means it went from 01 --> 11, 
   hence counter clockwise (we assume in clockwise rotation, A is before B) 
*/
void encoderAISR() {
  bool a = digitalRead(ENCODER_A);
  bool b = digitalRead(ENCODER_B);
  if (a == b) encoderCount++;
  else encoderCount--;
}

void encoderBISR() {
  bool a = digitalRead(ENCODER_A);
  bool b = digitalRead(ENCODER_B);
  if (a != b) encoderCount++;
  else encoderCount--;
}

void RPMconversion() {

  setpointPPST = (setpointRPM * PPR_DC_ENGINE * (sampleTime / 1000.0)) / 60.0;
  Serial.print(String(setpointPPST));
}
