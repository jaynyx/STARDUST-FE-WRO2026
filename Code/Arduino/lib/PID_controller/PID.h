//  DC Motor Speed Controller — TB6612FNG + Encoder
//  PID speed control driven by RPM commands over Serial
//  Simplified: single anti-windup mechanism, derivative-on-measurement,
//  no output slew limiter.
#include <Arduino.h>
#ifndef PID_h
#define PID_h

// TB6612FNG DC motor driver pins
#define PWMA 5
#define AIN1 7
#define AIN2 8
#define STBY 9

// DC Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

// Encoder variables
extern volatile long encoderCount;
extern long currentCount;
extern long previousCount;
extern long deltaPulses;

// Encoder resolution
extern double PPR_DC_ENGINE;

// ---- PID parameters ----
extern double Kp; // still need to be set through testing
extern double Ki;
extern double Kd;
extern double Kf;     // introduction of feed-forward for faster accelerations closer to the target

extern double setpointRPM;  // desired RPM
extern double currentRPM;   // measured RPM
extern double output;      // PWM output

extern double previousMeasurement; // for derivative-on-measurement
extern double integral;
extern double error;

extern double P, I, D, derivative;

// Limits
extern const double OUTPUT_LIMIT;
extern const double INTEGRAL_LIMIT; // tune: max contribution I can add, in PWM units, is Ki*INTEGRAL_LIMIT

// Timing
extern unsigned long lastTime;
extern unsigned long now;
extern const unsigned long sampleTime;  // ms  *needs some tuning through testing 
extern double dt;

#endif


void runPID();

void encoderAISR();

void updateRPM(String newTargetRPM);

void applyMotor(double cmd);