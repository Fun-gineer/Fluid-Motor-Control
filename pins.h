/*
Define names of pins the hardware is connected to

In your code insert
   
   #include "pins.h"

*/

#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

//include EEPROM library for reading and storing position value when powered down
#define indicator_address 5 //address to indicate if previous value has been stored or not

//include I2C library and define I2C communication pins
#define I2C_ADDR 0x27 //0x27 for LCD comm
#define ADC4 //SDA pin
#define ADC5 //SCL pin

/* Digital Pins to turn the motor on/off 
 IN1  IN2   FUNCTION  ( nSLEEP = PB1, IN1 = PD3, PB2, IN2 = PD4, PD5)
  H   PWM   Forward, slow decay //Used in this routine
 PWM   H    Reverse, slow decay //Used in this routine
 PWM   L    Forward, fast decay
  L   PWM   Reverse, fast decay 
  L    L    Coast, fast decay   //Used in this routine
  H    H    Brake, slow decay   //Used in this routine */

//HBridge sleep pin
#define nSLEEP 6 // pin 1 of H-Bridge; HIGH=on, LOW=sleep  ---PD6
#define nFAULT 5 // pin 8 of H-Bridge; low input for fault     ---PD5 (labeled 0 on board schematic)

//LDO power pin for encoders
#define nEncoder 13 // Enable pin of 5V LDO LOW=sleep  --PB5

//motor1-A JP3-A //device motor
#define HBridgeAPin1 3  // PWM connected to H-Bridge Pin 16 ----PD3 to +anode MotorA-1
#define HBridgeAPin2 11 // PWM connected to H-Bridge Pin 15 ----PB3 to -cathode MotorA-2

//motor1-B JP3-B //pump momtor
#define HBridgeBPin1 9  // PWM connected to H-Bridge Pin 9 ----PB1 to +anode MotorB-1
#define HBridgeBPin2 10 // PWM connected to H-Bridge Pin 10 ----PB2 to -cathode MotorB-2

// Analog Pins for absolute position sensors
#define ADC0 A0// analog input motor1-A JP4-A
#define ADC1 A1// analog input motor1-B JP4-B

//Digital pins for Maxon encoder input motor1-B JP6 (must be on port D pins 2-7)
#define encoderBPinA 2 //Channel A encoder input pin (interrupt 0)    ----PD2
#define encoderBPinB 4 //Channel B encoder input pin (need not be interrupt pin)     ----PD4

#define PD7 7 // extra digital pin
#define PB0 8 // extra digital pin

#define PB4 12 // extra digital pin
#define PB5 13 //Electrocoalescence output pin

#define limitPin1 12 // Limit Switch digital input pin

#endif
