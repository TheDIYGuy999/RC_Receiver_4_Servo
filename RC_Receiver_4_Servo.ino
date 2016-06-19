// 4 Channel Receiver with 4 standard servo outputs
// 3.3V, 8MHz Pro Mini, 2.4GHz NRF24L01 radio module

//
// =======================================================================================================
// BUILD OPTIONS (comment out unneeded options)
// =======================================================================================================
//

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

//
// =======================================================================================================
// INCLUDE LIRBARIES
// =======================================================================================================
//

// Libraries
#include <SPI.h>
#include <RF24.h> // Installed via Tools > Board > Boards Manager > Type RF24
#include <printf.h>
#include <Servo.h>
#include <SimpleTimer.h> // https://github.com/jfturcot/SimpleTimer
#include <statusLED.h> // https://github.com/TheDIYGuy999/statusLED

#include "readVCC.h"

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

// Vehicle address
int vehicleNumber = 2; // This number must be unique for each vehicle!
const int maxVehicleNumber = 5;

// the ID number of the used "radio pipe" must match with the selected ID on the transmitter!
const uint64_t pipeIn[maxVehicleNumber] = { 0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL };

// The size of this struct should not exceed 32 bytes
struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  boolean mode1 = false; // Speed limitation
  boolean mode2 = false;
};

RcData data;

// Hardware configuration: Set up nRF24L01 radio on hardware SPI bus & pins 7 (CE) & 8 (CSN)
RF24 radio(7, 8);

// Battery
boolean batteryOK = false;

// Create Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Status LED objects
statusLED battLED(false);

// Timer
SimpleTimer timer;

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
  printf_begin();
  delay(3000);
#endif

  // LED setup
  battLED.begin(2);

  // Radio setup
  radio.begin();
  radio.setChannel(1);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);                  // Ensure autoACK is enabled
  radio.setRetries(1, 5);                  // 1x250us delay (blocking!!), max. 5 retries
  radio.setCRCLength(RF24_CRC_8);          // Use 8-bit CRC for performance

#ifdef DEBUG
  radio.printDetails();
  delay(3000);
#endif

  radio.openReadingPipe(1, pipeIn[vehicleNumber - 1]);
  radio.startListening();

  // Servo pins
  servo1.attach(A0);
  servo2.attach(A1);
  servo3.attach(A2);
  servo4.attach(A3);

  // All axis to neutral position
  data.axis1 = 50;
  data.axis2 = 50;
  data.axis3 = 50;
  data.axis4 = 50;

  timer.setInterval(100, checkBattery); // Check battery voltage every 100ms
}

//
// =======================================================================================================
// LED
// =======================================================================================================
//

void led() {

  // Red LED (ON = battery empty, blinking = OK
  if (batteryOK) {
    battLED.flash(140, 150, 500, vehicleNumber); // ON, OFF, PAUSE, PULSES
  } else {
    battLED.off(); // Always ON = battery low voltage
  }
}

//
// =======================================================================================================
// READ RADIO DATA
// =======================================================================================================
//

void readRadio() {

  static unsigned long lastRecvTime = 0;

  if (radio.available()) {
    radio.read(&data, sizeof(RcData));
    lastRecvTime = millis();
#ifdef DEBUG
    Serial.print(data.axis1);
    Serial.print("\t");
    Serial.print(data.axis2);
    Serial.print("\t");
    Serial.print(data.axis3);
    Serial.print("\t");
    Serial.print(data.axis4);
    Serial.println("\t");
#endif
  }

  if (millis() - lastRecvTime > 1000) { // bring all servos to their middle position, if no RC signal is received for 1s!
    data.axis1 = 50; // Aileron (Steering for car)
    data.axis2 = 50; // Elevator
    data.axis3 = 50; // Throttle
    data.axis4 = 50; // Rudder
#ifdef DEBUG
    Serial.println("No Radio Available - Check Transmitter!");
#endif
  }
}

//
// =======================================================================================================
// WRITE SERVO POSITIONS
// =======================================================================================================
//

void writeServos() {
  servo1.write(map(data.axis1, 100, 0, 45, 135) ); // 45 - 135°
  servo2.write(map(data.axis2, 100, 0, 45, 135) ); // 45 - 135°
  servo3.write(map(data.axis3, 100, 0, 45, 135) ); // 45 - 135°
  servo4.write(map(data.axis4, 100, 0, 45, 135) ); // 45 - 135°
}


//
// =======================================================================================================
// CHECK BATTERY VOLTAGE
// =======================================================================================================
//

void checkBattery() {
  float batteryVolt = readVcc() / 1000.0 ;

  if (batteryVolt >= 3.0) {
    batteryOK = true;
#ifdef DEBUG
    Serial.print(batteryVolt);
    Serial.println(" V. Battery OK");
#endif
  } else {
    batteryOK = false;
#ifdef DEBUG
    Serial.print(batteryVolt);
    Serial.println(" V. Battery empty!");
#endif
  }
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {

  // Timer
  timer.run();

  // Read radio data from transmitter
  readRadio();

  // Write the servo positions
  writeServos();

  // LED
  led();
}

