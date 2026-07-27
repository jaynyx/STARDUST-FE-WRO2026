//  DC Motor Speed Controller — TB6612FNG + Encoder
//  PID speed control driven by RPM commands over Serial
//  Simplified: single anti-windup mechanism, derivative-on-measurement,
//  no output slew limiter.

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

// Encoder resolution
double PPR_DC_ENGINE = 211.2;

// ---- PID parameters ----
double Kp = 0.8; // still need to be set through testing
double Ki = 0.00;
double Kd = 0.00;

double setpointRPM = 0;  // desired RPM
double currentRPM = 0;   // measured RPM
double output = 0;       // PWM output

double previousMeasurement = 0; // for derivative-on-measurement
double integral = 0;
double error = 0;

double P, I, D, derivative;

// Limits
const double OUTPUT_LIMIT = 255.0;
const double INTEGRAL_LIMIT = 200.0; // tune: max contribution I can add, in PWM units, is Ki*INTEGRAL_LIMIT

// Timing
unsigned long lastTime = 0;
unsigned long now = 0;
const unsigned long sampleTime = 20;  // ms
double dt = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderAISR, CHANGE);

  lastTime = millis();
  Serial.println("Motor ready");
}

void loop() {
  readSerialCommand();
  runPID();
}

void readSerialCommand() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');

  if (command.startsWith("RPM:")) {       // parses new RPM setpoint commands
    setpointRPM = command.substring(4).toInt();
    Serial.print("New RPM target: ");
    Serial.println(setpointRPM);
  }
}

void runPID() {
  now = millis();
  if (now - lastTime < sampleTime) return;
  dt = (now - lastTime) / 1000.0;

  // Read encoder safely
  noInterrupts();
  currentCount = encoderCount;
  interrupts();

  deltaPulses = currentCount - previousCount;
  previousCount = currentCount;

  currentRPM = (deltaPulses / PPR_DC_ENGINE) * (60.0 / dt);  // PPST conversion to RPMS

  error = setpointRPM - currentRPM;

  // --- Proportional ---
  P = Kp * error;

  // --- Derivative on measurement ---
  derivative = -(currentRPM - previousMeasurement) / dt;
  D = Kd * derivative;
  previousMeasurement = currentRPM;

  // --- Integral, with single anti-windup rule: ---
  // only accumulate if doing so wouldn't push the output past its limit.
  double unclamppedOutput = P + Ki * (integral + error * dt) + D;
  if (unclamppedOutput < OUTPUT_LIMIT && unclamppedOutput > -OUTPUT_LIMIT) {
    integral += error * dt;
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);        // keeps integral from reaching infiniti
  }
  I = Ki * integral;

  output = constrain(P + I + D, -OUTPUT_LIMIT, OUTPUT_LIMIT);

  Serial.print("RPM=");
  Serial.print(currentRPM);
  Serial.print(" Error=");
  Serial.print(error);
  Serial.print(" P=");
  Serial.print(P);
  Serial.print(" I=");
  Serial.print(I);
  Serial.print(" D=");
  Serial.print(D);
  Serial.print(" PWM=");
  Serial.println(output);

  lastTime = now;
  applyMotor(output);
}

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

// Encoder interrupt
void encoderAISR() {
  if (setpointRPM >= 0)
    encoderCount++;
  else
    encoderCount--;
}
