#ifndef _RCM_H_
#define _RCM_H_

#include <Arduino.h>
#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0
#include <JMotor.h> //https://github.com/joshua-8/JMotor

#ifndef RCM_HARDWARE_VERSION
#define port1Pin 32
#define port2Pin 33
#define port3Pin 25
#define port4Pin 26
#define port5Pin 27
//          PWM_CH, PIN
#define port1 8, port1Pin
#define port2 7, port2Pin
#define port3 6, port3Pin
#define port4 5, port4Pin
#define port5 4, port5Pin
//           PWM_CH, EN_PIN, PIN1, PIN2
#define portA 3, 23, 22, 19
#define portB 1, 5, 18, 21
#define portC 2, 17, 16, 12
#define portD 0, 13, 4, 14

#define inport1 39
#define inport2 34
#define inport3 35

#define ONBOARD_LED 2
#define batMonitorPin 36

void setupMotors() { }

#elif RCM_HARDWARE_VERSION == 10 // rcmByte_1

#include <FastLED.h>
#include <TMC7300.h>

CRGB RSL_leds[1] = { CRGB(0, 0, 0) };

#define RSL RSL_leds[0]

inline void setRSL(CRGB color, boolean show = true)
{
    if (RSL != color) {
        RSL = color;
        if (show)
            FastLED.show();
    }
}

CRGB RSLcolor = CRGB(250, 45, 0); // orange

#define batMonitorPin A1
#define uartPin A0

#define motorsEnablePin RX

#define buttonPin 0

#define port1Pin SCK
#define port2Pin MISO
#define port3Pin MOSI
#define port4Pin A2
#define port5Pin A3
#define port6Pin SDA
#define port7Pin SCL
#define port8Pin TX
//          PWM_CH, PIN
#define port1 0, port1Pin
#define port2 1, port2Pin
#define port3 2, port3Pin
#define port4 3, port4Pin
#define port5 4, port5Pin
#define port6 5, port6Pin
#define port7 6, port7Pin
#define port8 7, port8Pin

//            chip address, motor address
#ifndef MOTOR_DRIVER_BAUD
#define MOTOR_DRIVER_BAUD 110000
#endif

#define portAB uartPin, 0, MOTOR_DRIVER_BAUD
#define portCD uartPin, 2, MOTOR_DRIVER_BAUD
#define portEF uartPin, 3, MOTOR_DRIVER_BAUD
#define portGH uartPin, 1, MOTOR_DRIVER_BAUD

TMC7300IC TMC7300_AB = TMC7300IC(portAB);
TMC7300IC TMC7300_CD = TMC7300IC(portCD);
TMC7300IC TMC7300_EF = TMC7300IC(portEF);
TMC7300IC TMC7300_GH = TMC7300IC(portGH);

#define portA TMC7300_AB, 0
#define portB TMC7300_AB, 1
#define portC TMC7300_CD, 0
#define portD TMC7300_CD, 1
#define portE TMC7300_EF, 0
#define portF TMC7300_EF, 1
#define portG TMC7300_GH, 0
#define portH TMC7300_GH, 1

JMotorDriverTMC7300 motorDriverA = JMotorDriverTMC7300(TMC7300_AB, 0);
JMotorDriverTMC7300 motorDriverB = JMotorDriverTMC7300(TMC7300_AB, 1);
JMotorDriverTMC7300 motorDriverC = JMotorDriverTMC7300(TMC7300_CD, 0);
JMotorDriverTMC7300 motorDriverD = JMotorDriverTMC7300(TMC7300_CD, 1);
JMotorDriverTMC7300 motorDriverE = JMotorDriverTMC7300(TMC7300_EF, 1);
JMotorDriverTMC7300 motorDriverF = JMotorDriverTMC7300(TMC7300_EF, 0);
JMotorDriverTMC7300 motorDriverG = JMotorDriverTMC7300(TMC7300_GH, 1);
JMotorDriverTMC7300 motorDriverH = JMotorDriverTMC7300(TMC7300_GH, 0);

void setupMotors()
{
    pinMode(motorsEnablePin, OUTPUT);
    digitalWrite(motorsEnablePin, LOW);
    TMC7300_AB.begin();
    TMC7300_CD.begin();
    TMC7300_EF.begin();
    TMC7300_GH.begin();
    digitalWrite(motorsEnablePin, HIGH);
}

#endif // RCM_HARDWARE_VERSION

boolean enabled = false;
boolean wasEnabled = false;

#endif
