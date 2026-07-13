//  DC Motor Speed Controller — TB6612FNG + Quadrature Encoder
//  PID speed control driven by RPM commands over Serial

// TB6612FNG motor driver pins
#define PWMA 5
#define AIN1 7
#define AIN2 8
#define STBY 9

// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

// Encoder variables
volatile long encoderCount = 0;
long currentCount = 0;
long previousCount = 0;
long deltaPulses = 0;

// Pulses per revolution: DC motor datasheet value with 1:9.6 gearbox ratio.
// Using x4 quadrature would give 422.4 PPR; if encoder channel B is broken,
// a simpler x2 scheme is used instead -> 422.4 / 2 = 211.2. This is still left to be tested...
double PPR_DC_ENGINE = 211.2;

// PID parameters
double Kp = 1.0;
double Ki = 0.6;
double Kd = 0.05;
double Kff = 6.0;

double setpointRPM = 0;   // target speed, RPM
double setpointPPST = 0;  // target pulses per sample time
double input = 0;         // measured pulses per sample time
double output = 0;        // PWM command sent to the motor

double previousError = 0;
double integral = 0;

const double OUTPUT_LIMIT = 255.0;
const double INTEGRAL_LIMIT = OUTPUT_LIMIT / 1.0; // clamps Ki * integral contribution to remove exponential infinity rize

// Timing
unsigned long lastTime = 0;
unsigned long now = 0;
const unsigned long sampleTime = 50;  // ms (20 Hz) — faster loop = faster response
double dt = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // enable TB6612

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderAISR, CHANGE);

  lastTime = millis();
  Serial.println("Motor ready");
}

//main loop
void loop() {
  readSerialCommand();
  runPID();
}

void readSerialCommand() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  Serial.print("Command received: [");
  Serial.print(command);
  Serial.println("]");

  if (command.startsWith("RPM:")) {
    setpointRPM = command.substring(4).toInt();
    RPMconversion();

    Serial.print("New RPM setpoint: ");
    Serial.println(setpointRPM);
    Serial.print("New PPST setpoint: ");
    Serial.println(setpointPPST);
    Serial.println("Entering PID");
  }
}

void runPID() {
  now = millis();
  if (now - lastTime < sampleTime) return;

  dt = (now - lastTime) / 1000.0;

  noInterrupts();
  currentCount = encoderCount;
  interrupts();

  deltaPulses = currentCount - previousCount;
  previousCount = currentCount;
  input = deltaPulses;

  double error = setpointPPST - input;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT / Ki, INTEGRAL_LIMIT / Ki);  // anti-windup

  double derivative = (error - previousError) / dt;

  output = Kff * setpointPPST + Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
  lastTime = now;

  applyMotor(output);

  // ---- Debug ----
  Serial.print("Pulses: ");
  Serial.print(input);
  Serial.print("  Error: ");
  Serial.print(error);
  Serial.print("  Output: ");
  Serial.println(output);
}

void applyMotor(double cmd) {
  cmd = constrain(cmd, -OUTPUT_LIMIT, OUTPUT_LIMIT);

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

void encoderAISR() {
  if (setpointRPM > 0) {
    encoderCount++;
  } else if (setpointRPM < 0) {
    encoderCount--;
  }
}

void RPMconversion() {
  setpointPPST = (setpointRPM * PPR_DC_ENGINE * (sampleTime / 1000.0)) / 60.0;
  Serial.print("Computed PPST: ");
  Serial.println(setpointPPST);
}
