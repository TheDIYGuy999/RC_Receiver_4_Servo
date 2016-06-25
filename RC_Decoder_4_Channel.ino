/* 4 channel RC signal converter for caterpillar vehicles. This device converts the RC signal from a standard
  receiver and sends it to two DRV8833 DC motor drivers for the two caterpillar motors & two additional motors

  Pro Micro 5V / 16MHz or 3.3V / 8MHz (also Pro Mini with board rev. 1.1  or above)

*/
//
// =======================================================================================================
// LIRBARIES & TABS
// =======================================================================================================
//
#include <PWMFrequency.h> // https://github.com/kiwisincebirth/Arduino/tree/master/libraries/PWMFrequency
#include <DRV8833.h> // https://github.com/TheDIYGuy999/DRV8833


#include "curves.h" // load nonlinear steering correction arrays

//
// =======================================================================================================
// BUILD OPTIONS (comment out unneeded options)
// =======================================================================================================
//

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

// RC pins
#define RC_IN_1 A0 // Throttle or left caterpillar in direct mode (Channel 1)
#define RC_IN_2 A1 // Steering or right caterpillar in direct mode (Channel 2)
#define RC_IN_3 A2 // Channel 3
#define RC_IN_4 A3 // Channel 4

// Motor 1 (left caterpillar)
#define MOTOR_1_PWM 9
#define MOTOR_1_DIR 8

// Motor 2 (right caterpillar)
#define MOTOR_2_PWM 10
#define MOTOR_2_DIR 16

// Motor 3
#define MOTOR_3_PWM 5
#define MOTOR_3_DIR 3

// Motor 4
#define MOTOR_4_PWM 6
#define MOTOR_4_DIR 4

// Mode pins
#define MODE_PIN 2
#define MODE_PIN2 7

// Define global variables

// How many channels?
const int channels = 4;

// RC signal adjustment
const int servoMax = 1900; // 1800 us = 1.8ms (2ms equals 45° in theory)
const int servoMin = 1100; // 1200 us = 1.2ms (1ms equals -45° in theory)
const int servoNeutralMax = 1600; // 1600 us = 1.6ms
const int servoNeutralMin = 1400; // 1400 us = 1.4ms

// The range of the motor control signals
const int scaleMin = -100;
const int scaleMax = 100;

// RC signal impulse duration (in microseconds)
int pulse[5];

// RC channel offsets (in microseconds)
int offset[5];

// Steering overlay %
int steeringFactorLeft;
int steeringFactorRight;

// PWM value for each motor driver
int pwm[5];

// Initialize DRV8833 H-Bridge
// NOTE: The first pin must always be PWM capable, the second only, if the last parameter is set to "true"
// SYNTAX: IN1, IN2, min. input value, max. input value, neutral position width
// invert rotation direction, true = both pins are PWM capable
DRV8833 Motor1(MOTOR_1_PWM, MOTOR_1_DIR, scaleMin, scaleMax, 20, false, false);
DRV8833 Motor2(MOTOR_2_PWM, MOTOR_2_DIR, scaleMin, scaleMax, 20, false, false);
DRV8833 Motor3(MOTOR_3_PWM, MOTOR_3_DIR, scaleMin, scaleMax, 20, false, false);
DRV8833 Motor4(MOTOR_4_PWM, MOTOR_4_DIR, scaleMin, scaleMax, 20, false, false);

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

  // Serial initialization----------------------------------
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Set pinmodes
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN2, INPUT_PULLUP);


  // wait for RC receiver to initialize
  delay(1000);

  // Define PWM frequencies
  setPWMPrescaler(MOTOR_1_PWM, 1); // 123Hz = 256,  492Hz = 64, 3936Hz = 8, 31488Hz = 1
  setPWMPrescaler(MOTOR_2_PWM, 1);
  setPWMPrescaler(MOTOR_3_PWM, 1);
  setPWMPrescaler(MOTOR_4_PWM, 1);

  // Read RC data,
  readRC();

  // wait 1s,
  delay(1000);

  // then compute the RC channel offsets
  computeOffset();
}

//
// =======================================================================================================
// READ RC SERVO CHANNELS
// =======================================================================================================
//

void readRC() {
  static unsigned long previousMillis;
  // NOTE: If we want to use the ramptimes in the driveMotors() function, we don't want to block the MCU
  // every single clock cycle with thepulseIn() functions. So we only read the pulse signals every 30ms!
  // Otherwise, the delay time is zero. This provides a more direct vehicle response.
  if (millis() - previousMillis >= 0) { // put the desired time in ms in here
    // Read signals from RC receiver
    pulse[1] = pulseIn(RC_IN_1, HIGH, 100000); // 1 - 2 ms pulse length = -45° to 45° servo angle
    pulse[2] = pulseIn(RC_IN_2, HIGH, 100000); // 100000 = 0.1s timeout
    pulse[3] = pulseIn(RC_IN_3, HIGH, 100000); // 100000 = 0.1s timeout
    pulse[4] = pulseIn(RC_IN_4, HIGH, 100000); // 100000 = 0.1s timeout

    // center channels, if no valid impulse length was detected!
    if (pulse[1] == 0) pulse[1] = (servoNeutralMin + servoNeutralMax)  / 2;
    if (pulse[2] == 0) pulse[2] = (servoNeutralMin + servoNeutralMax)  / 2;
    if (pulse[3] == 0) pulse[3] = (servoNeutralMin + servoNeutralMax)  / 2;
    if (pulse[4] == 0) pulse[4] = (servoNeutralMin + servoNeutralMax)  / 2;

    previousMillis = millis();
  }
}

//
// =======================================================================================================
// COMPUTE RC CHANNEL OFFSET (Channel auto zero detection during setup)
// =======================================================================================================
//

void computeOffset() {
  for (int i = 1; i <= channels; i++) {
    offset[i] = (servoNeutralMin + servoNeutralMax) / 2 - pulse[i];
  }
#ifdef DEBUG
  Serial.print(offset[1]);
  Serial.print("\t");
  Serial.print(offset[2]);
  Serial.print("\t");
  Serial.print(offset[3]);
  Serial.print("\t");
  Serial.println(offset[4]);
#endif
}

//
// =======================================================================================================
// RC CHANNEL OFFSET
// =======================================================================================================
//

void channelOffset() {
  for (int i = 1; i <= channels; i++) {
    pulse[i] = pulse[i] + offset[i];
  }
}

//
// =======================================================================================================
// "STEERING" MOTOR DRIVING FUNCTION (Throttle & Steering signals)
// =======================================================================================================
//

void driveMotorsSteering() {

  int steeringFactorLeft2;
  int steeringFactorRight2;

  // Compute steering overlay:
  // The steering signal is channel 1 = pulse[1]
  // 100% = wheel spins with 100% of the requested speed forward
  // -100% = wheel spins with 100% of the requested speed backward
  if (pulse[1] <= servoNeutralMin) {
    steeringFactorLeft = map(pulse[1], servoMin, servoNeutralMin, 0, 100);
    steeringFactorLeft = constrain(steeringFactorLeft, 0, 100);
  }
  else {
    steeringFactorLeft = 100;
  }

  if (pulse[1] >= servoNeutralMax) {
    steeringFactorRight = map(pulse[1], servoMax, servoNeutralMax, 0, 100);
    steeringFactorRight = constrain(steeringFactorRight, 0, 100);
  }
  else {
    steeringFactorRight = 100;
  }



  // If the "Semi Caterpillar" jumper isn't present:
  if (digitalRead(MODE_PIN2)) {
    steeringFactorLeft2 = pgm_read_word(&curveFull[steeringFactorLeft]);
    steeringFactorRight2 = pgm_read_word(&curveFull[steeringFactorRight]);
  }
  else {
    steeringFactorLeft2 = pgm_read_word(&curveSemi[steeringFactorLeft]);
    steeringFactorRight2 = pgm_read_word(&curveSemi[steeringFactorRight]);
  }


  // Nonlinear steering correction


#ifdef DEBUG
  Serial.print(steeringFactorLeft);
  Serial.print("\t");
  Serial.print(steeringFactorRight);
  Serial.print("\t");
  Serial.print(steeringFactorLeft2);
  Serial.print("\t");
  Serial.print(steeringFactorRight2);
  Serial.print("\t");
  Serial.print(pwm[1]);
  Serial.print("\t");
  Serial.println(pwm[2]);
#endif

  // Caterpillar motors
  // The throttle signal (for both caterpillars) is channel 3 = pulse[3]
  pwm[1] = map(pulse[3], servoMin, servoMax, scaleMax, scaleMin) * steeringFactorRight2 / 100;
  pwm[2] = map(pulse[3], servoMin, servoMax, scaleMax, scaleMin) * steeringFactorLeft2 / 100;

  // Additional motors
  pwm[3] = map(pulse[2], servoMin, servoMax, scaleMax, scaleMin);
  pwm[4] = map(pulse[4], servoMin, servoMax, scaleMax, scaleMin);
}


//
// =======================================================================================================
// "DIRECT" MOTOR DRIVING FUNCTION (Throttle L & R signals)
// =======================================================================================================
//

void driveMotorsDirect() {

  pwm[1] = map(pulse[3], servoMin, servoMax, scaleMax, scaleMin); // Left caterpillar
  pwm[2] = map(pulse[2], servoMin, servoMax, scaleMax, scaleMin); // Right caterpillar
  pwm[3] = map(pulse[1], servoMin, servoMax, scaleMax, scaleMin);
  pwm[4] = map(pulse[4], servoMin, servoMax, scaleMax, scaleMin);
}

//
// =======================================================================================================
// WRITE MOTOR OUTPUTS
// =======================================================================================================
//

void driveMotors() {

  // Clamp values between scaleMin and scaleMax!
  for (int i = 1; i <= channels; i++) {
    pwm[i] = constrain(pwm[i], scaleMin, scaleMax);
  }

  // SYNTAX: Input value, max PWM, ramptime in ms per 1 PWM increment
  // false = brake inactive, false = brake in neutral position inactive
  Motor1.drive(pwm[1], 255, 0, false, false); // Left cateripllar
  Motor2.drive(pwm[2], 255, 0, false, false); // Right cateripllar
  Motor3.drive(pwm[3], 255, 0, false, false);
  Motor4.drive(pwm[4], 255, 0, false, false);
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {

  // Read RC servo channels
  readRC();

  channelOffset();

  if (digitalRead(MODE_PIN) && digitalRead(MODE_PIN2)) { // Note: HIGH = no Jumper!
    // Drive motors in direct mode, if no jumper is set
    driveMotorsDirect();
  } else {
    // Drive motors in steering mode, if jumper is set
    driveMotorsSteering();
  }

  // Write motor outputs
  driveMotors();
}
