#include <PID.h>


// Encoder variables
volatile long encoderCount = 0;
long currentCount = 0;
long previousCount = 0;
long deltaPulses = 0;

// Encoder resolution
double PPR_DC_ENGINE = 211.2;

// ---- PID parameters ----
double Kp = 0.4; // still need to be set through testing
double Ki = 0.95;
double Kd = 0.00;
double Kf = 0.5;      // introduction of feed-forward for faster accelerations closer to the target

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
const unsigned long sampleTime = 20;  // ms  *needs some tuning through testing 
double dt = 0;





void runPID() {
  now = millis();
  if (now - lastTime < sampleTime) return;
  dt = (now - lastTime) / 1000.0;

  // Read encoder safely
  noInterrupts();
  currentCount = encoderCount;
  interrupts();

  deltaPulses = currentCount - previousCount;
  //Serial.print("deltaPulse:");
  //Serial.print(deltaPulses);
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

  //output = constrain(P + I + D, -OUTPUT_LIMIT, OUTPUT_LIMIT);             // see if no bounds could create issue
  output = Kf * setpointRPM + P + I + D;

  /*
  Serial.print(" RPM=");
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
  Serial.println(output);*/

  lastTime = now;
  applyMotor(output);
}


// Encoder interrupt
void encoderAISR() {
  if (setpointRPM >= 0)
    encoderCount++;
  else
    encoderCount--;
}

void updateRPM(String newTargetRPM) {
  setpointRPM = newTargetRPM.toInt();
  Serial.print("New RPM target: ");
  Serial.println(setpointRPM);
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