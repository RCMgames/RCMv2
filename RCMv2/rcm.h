#ifndef _RCM_H_
#define _RCM_H_

#include <Arduino.h>
#include <JMotor.h> //https://github.com/joshua-8/JMotor

#define RCM_ORIGINAL 1
#define RCM_BYTE_V2 2
#define RCM_NIBBLE_V1 3
#define RCM_4_V1 4
#define RCM_D1_V1 5
#define ALFREDO_NOU2_NO_VOLTAGE_MONITOR 6
#define ALFREDO_NOU2_WITH_VOLTAGE_MONITOR 7
#define ALFREDO_NOU3 8

#define RCM_COMM_EWD 1
#define RCM_COMM_ROS 2

#if RCM_HARDWARE_VERSION == RCM_ORIGINAL
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

#ifndef OVERRIDE_DEFAULT_VOLTAGE_COMP
const int adcUnitsPerVolt = 440; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, adcUnitsPerVolt);
#endif

void setupMotors() { }

#ifndef EWDmaxWifiSendBufSize
#define EWDmaxWifiSendBufSize 41
#endif
#ifndef EWDmaxWifiRecvBufSize
#define EWDmaxWifiRecvBufSize 41
#endif

#elif RCM_HARDWARE_VERSION == RCM_D1_V1

#define port1Pin D2
#define port2Pin D1
//          PWM_CH, PIN
#define port1 port1Pin
#define port2 port2Pin
//           PIN1, PIN2
#define portA D0, D7
#define portB D5, D6

#define ONBOARD_LED D4
#define batMonitorPin A0

#ifndef OVERRIDE_DEFAULT_VOLTAGE_COMP
const int adcUnitsPerVolt = 111; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, adcUnitsPerVolt);
#endif

void setupMotors() { }

#ifndef EWDmaxWifiSendBufSize
#define EWDmaxWifiSendBufSize 41
#endif
#ifndef EWDmaxWifiRecvBufSize
#define EWDmaxWifiRecvBufSize 41
#endif

#elif RCM_HARDWARE_VERSION == RCM_4_V1

#define port1Pin 14
#define port2Pin 27
#define port3Pin 5
#define port4Pin 19
#define port5Pin 33
#define port6Pin 32
#define port7Pin 23
//          PWM_CH, PIN
#define port1 0, port1Pin
#define port2 1, port2Pin
#define port3 2, port3Pin
#define port4 3, port4Pin
#define port5 4, port5Pin
#define port6 5, port6Pin
#define port7 6, port7Pin
//           PWM_CH, PIN1, PIN2
#define portA 8, 4, 16
#define portB 9, 13, 12
#define portC 10, 17, 18
#define portD 11, 26, 25
#define portE 12, port5Pin, port4Pin
#define portF 13, port6Pin, port7Pin

#define inport1 35
#define inport2 34
#define inport3 39

#define ONBOARD_LED 2
#define batMonitorPin 36

#ifndef OVERRIDE_DEFAULT_VOLTAGE_COMP
const int adcUnitsPerVolt = 310; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, adcUnitsPerVolt);
#endif

void setupMotors() { }

#ifndef EWDmaxWifiSendBufSize
#define EWDmaxWifiSendBufSize 41
#endif
#ifndef EWDmaxWifiRecvBufSize
#define EWDmaxWifiRecvBufSize 41
#endif
#elif RCM_HARDWARE_VERSION == RCM_BYTE_V2

#include <FastLED.h>
#include <TMC7300.h>

CRGB RSL_leds[1] = { CRGB(0, 0, 0) };

#define RSL_LED RSL_leds[0]

inline void setRSL(CRGB color, boolean show = true)
{
    if (RSL_LED != color) {
        RSL_LED = color;
        if (show)
            FastLED.show();
    }
}

CRGB RSLcolor = CRGB(250, 45, 0); // orange

#define batMonitorPin RX
#define uartPin MISO

#define motorsEnablePin SCK

#define buttonPin 0

#define port1Pin A0
#define port2Pin MOSI
#define port3Pin A1
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
#define MOTOR_DRIVER_BAUD 100000
#endif

#define portAB uartPin, 1, MOTOR_DRIVER_BAUD
#define portCD uartPin, 3, MOTOR_DRIVER_BAUD
#define portEF uartPin, 2, MOTOR_DRIVER_BAUD
#define portGH uartPin, 0, MOTOR_DRIVER_BAUD

TMC7300IC TMC7300_AB = TMC7300IC(portAB);
TMC7300IC TMC7300_CD = TMC7300IC(portCD);
TMC7300IC TMC7300_EF = TMC7300IC(portEF);
TMC7300IC TMC7300_GH = TMC7300IC(portGH);

#define portA TMC7300_AB, 1
#define portB TMC7300_AB, 0
#define portC TMC7300_CD, 1
#define portD TMC7300_CD, 0
#define portE TMC7300_EF, 1
#define portF TMC7300_EF, 0
#define portG TMC7300_GH, 1
#define portH TMC7300_GH, 0

JMotorDriverTMC7300 motorDriverA = JMotorDriverTMC7300(portA);
JMotorDriverTMC7300 motorDriverB = JMotorDriverTMC7300(portB);
JMotorDriverTMC7300 motorDriverC = JMotorDriverTMC7300(portC);
JMotorDriverTMC7300 motorDriverD = JMotorDriverTMC7300(portD);
JMotorDriverTMC7300 motorDriverE = JMotorDriverTMC7300(portE);
JMotorDriverTMC7300 motorDriverF = JMotorDriverTMC7300(portF);
JMotorDriverTMC7300 motorDriverG = JMotorDriverTMC7300(portG);
JMotorDriverTMC7300 motorDriverH = JMotorDriverTMC7300(portH);

void setupMotors()
{
    pinMode(motorsEnablePin, OUTPUT);
    digitalWrite(motorsEnablePin, LOW);
    TMC7300_AB.begin();
    TMC7300_CD.begin();
    TMC7300_EF.begin();
    TMC7300_GH.begin();
#if RCM_HARDWARE_VERSION == RCM_BYTE_V2 || RCM_HARDWARE_VERSION == RCM_NIBBLE_V1
#ifdef RCM_BYTE_DO_NOT_USE_SAFE_DISABLE
    digitalWrite(motorsEnablePin, HIGH); // if not using safe disable, set the enable pin to enabled
#endif
#endif
}

#ifndef OVERRIDE_DEFAULT_VOLTAGE_COMP
const int adcUnitsPerVolt = 310; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, adcUnitsPerVolt);
#endif

#elif RCM_HARDWARE_VERSION == RCM_NIBBLE_V1

#include <FastLED.h>
#include <TMC7300.h>

CRGB RSL_leds[1] = { CRGB(0, 0, 0) };

#define RSL_LED RSL_leds[0]

inline void setRSL(CRGB color, boolean show = true)
{
    if (RSL_LED != color) {
        RSL_LED = color;
        if (show)
            FastLED.show();
    }
}

CRGB RSLcolor = CRGB(250, 45, 0); // orange

#define batMonitorPin A3
#define uartPin MISO

#define motorsEnablePin MOSI

#define buttonPin 0

#define auxPin1 RX
#define auxPin2 SCK

#define port1Pin A0
#define port2Pin A1
#define port3Pin A2
#define port4Pin TX
//          PWM_CH, PIN
#define port1 0, port1Pin
#define port2 1, port2Pin
#define port3 2, port3Pin
#define port4 3, port4Pin

//            chip address, motor address
#ifndef MOTOR_DRIVER_BAUD
#define MOTOR_DRIVER_BAUD 100000
#endif

#define portAB uartPin, 0, MOTOR_DRIVER_BAUD
#define portCD uartPin, 1, MOTOR_DRIVER_BAUD

TMC7300IC TMC7300_AB = TMC7300IC(portAB);
TMC7300IC TMC7300_CD = TMC7300IC(portCD);

#define portA TMC7300_AB, 1
#define portB TMC7300_AB, 0
#define portC TMC7300_CD, 1
#define portD TMC7300_CD, 0

JMotorDriverTMC7300 motorDriverA = JMotorDriverTMC7300(portA);
JMotorDriverTMC7300 motorDriverB = JMotorDriverTMC7300(portB);
JMotorDriverTMC7300 motorDriverC = JMotorDriverTMC7300(portC);
JMotorDriverTMC7300 motorDriverD = JMotorDriverTMC7300(portD);

void setupMotors()
{
    pinMode(motorsEnablePin, OUTPUT);
    digitalWrite(motorsEnablePin, LOW);
    TMC7300_AB.begin();
    TMC7300_CD.begin();
    digitalWrite(motorsEnablePin, HIGH);
}
#ifndef OVERRIDE_DEFAULT_VOLTAGE_COMP
const int adcUnitsPerVolt = 310; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, adcUnitsPerVolt);
#endif

#elif RCM_HARDWARE_VERSION == ALFREDO_NOU2_NO_VOLTAGE_MONITOR || RCM_HARDWARE_VERSION == ALFREDO_NOU2_WITH_VOLTAGE_MONITOR
// https://github.com/AlfredoSystems/Alfredo-NoU2/blob/master/Alfredo_NoU2.h
#define MOTOR1_A 13
#define MOTOR1_B 12
#define MOTOR2_A 27
#define MOTOR2_B 33
#define MOTOR3_A 32
#define MOTOR3_B 18
#define MOTOR4_A 19
#define MOTOR4_B 23
#define MOTOR5_A 15
#define MOTOR5_B 14
#define MOTOR6_A 22
#define MOTOR6_B 21

#define SERVO1_PIN 16
#define SERVO2_PIN 17
#define SERVO3_PIN 25
#define SERVO4_PIN 26

#define RSL_PIN 2 // Same as built-in LED

// PWM Channels
#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1
#define MOTOR3_CHANNEL 2
#define MOTOR4_CHANNEL 3
#define MOTOR5_CHANNEL 4
#define MOTOR6_CHANNEL 5
#define SERVO1_CHANNEL 6
#define SERVO2_CHANNEL 7
#define SERVO3_CHANNEL 8
#define SERVO4_CHANNEL 9

#define port1Pin SERVO1_PIN
#define port2Pin SERVO2_PIN
#define port3Pin SERVO3_PIN
#define port4Pin SERVO4_PIN
//          PWM_CH, PIN
#define port1 SERVO1_CHANNEL, port1Pin
#define port2 SERVO2_CHANNEL, port2Pin
#define port3 SERVO3_CHANNEL, port3Pin
#define port4 SERVO4_CHANNEL, port4Pin
//           PWM_CH, EN_PIN, PIN1, PIN2
#define portA MOTOR1_CHANNEL, MOTOR1_B, MOTOR1_A
#define portB MOTOR2_CHANNEL, MOTOR2_B, MOTOR2_A
#define portC MOTOR3_CHANNEL, MOTOR3_B, MOTOR3_A
#define portD MOTOR4_CHANNEL, MOTOR4_B, MOTOR4_A
#define portE MOTOR5_CHANNEL, MOTOR5_B, MOTOR5_A
#define portF MOTOR6_CHANNEL, MOTOR6_B, MOTOR6_A

#define ONBOARD_LED RSL_PIN

#if RCM_HARDWARE_VERSION == ALFREDO_NOU2_WITH_VOLTAGE_MONITOR
#define batMonitorPin 36
#endif

#ifndef OVERRIDE_DEFAULT_VOLTAGE_COMP
#if RCM_HARDWARE_VERSION == ALFREDO_NOU2_NO_VOLTAGE_MONITOR
JVoltageCompConst voltageComp = JVoltageCompConst(10);
#elif RCM_HARDWARE_VERSION == ALFREDO_NOU2_WITH_VOLTAGE_MONITOR
const int adcUnitsPerVolt = 310; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, adcUnitsPerVolt);
#endif
#endif

void setupMotors() { }

#ifndef EWDmaxWifiSendBufSize
#define EWDmaxWifiSendBufSize 41
#endif
#ifndef EWDmaxWifiRecvBufSize
#define EWDmaxWifiRecvBufSize 41
#endif

#elif RCM_HARDWARE_VERSION == ALFREDO_NOU3

#else
void setupMotors() { }
#endif // RCM_HARDWARE_VERSION

boolean enabled = false;
boolean wasEnabled = false;

#if RCM_COMM_METHOD == RCM_COMM_EWD

#ifndef EWDmaxWifiSendBufSize
#define EWDmaxWifiSendBufSize 200
#endif
#ifndef EWDmaxWifiRecvBufSize
#define EWDmaxWifiRecvBufSize 200
#endif

#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0

#elif RCM_COMM_METHOD == RCM_COMM_ROS

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

unsigned long lastEnableSentMillis = 0;
boolean ROSCheckFail = false;

#include "rcmros.h"

#endif

#endif
