//   This example program controls all 8 motors and 8 servos from an RCM BYTE. Use with the "byter" config file of https://github.com/RCMgames/rcmds-new
//   https://github.com/rcmgames/RCMv2
//   for information about the electronics, see the link at the top of this page: https://github.com/RCMgames

#define RCM_HARDWARE_VERSION 10 // uncomment if you have an RCMByte board

#include "rcm.h" //defines pins

const int dacUnitsPerVolt = 440; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);
// set up motors and anything else you need here
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain
JMotorDriverEsp32Servo servoDriver1 = JMotorDriverEsp32Servo(port1);
JMotorDriverEsp32Servo servoDriver2 = JMotorDriverEsp32Servo(port2);
JMotorDriverEsp32Servo servoDriver3 = JMotorDriverEsp32Servo(port3);
JMotorDriverEsp32Servo servoDriver4 = JMotorDriverEsp32Servo(port4);
JMotorDriverEsp32Servo servoDriver5 = JMotorDriverEsp32Servo(port5);
JMotorDriverEsp32Servo servoDriver6 = JMotorDriverEsp32Servo(port6);
JMotorDriverEsp32Servo servoDriver7 = JMotorDriverEsp32Servo(port7);
JMotorDriverEsp32Servo servoDriver8 = JMotorDriverEsp32Servo(port8);

float mA = 0;
float mB = 0;
float mC = 0;
float mD = 0;
float mE = 0;
float mF = 0;
float mG = 0;
float mH = 0;

float s1 = 0;
float s2 = 0;
float s3 = 0;
float s4 = 0;
float s5 = 0;
float s6 = 0;
float s7 = 0;
float s8 = 0;

void Enabled()
{
    // code to run while enabled, put your main code here
    motorDriverA.set(mA);
    motorDriverB.set(mB);
    motorDriverC.set(mC);
    motorDriverD.set(mD);
    motorDriverE.set(mE);
    motorDriverF.set(mF);
    motorDriverG.set(mG);
    motorDriverH.set(mH);

    servoDriver1.set(s1);
    servoDriver2.set(s2);
    servoDriver3.set(s3);
    servoDriver4.set(s4);
    servoDriver5.set(s5);
    servoDriver6.set(s6);
    servoDriver7.set(s7);
    servoDriver8.set(s8);
}

void Enable()
{
    // turn on outputs
    motorDriverA.enable();
    motorDriverB.enable();
    motorDriverC.enable();
    motorDriverD.enable();
    motorDriverE.enable();
    motorDriverF.enable();
    motorDriverG.enable();
    motorDriverH.enable();

    servoDriver1.enable();
    servoDriver2.enable();
    servoDriver3.enable();
    servoDriver4.enable();
    servoDriver5.enable();
    servoDriver6.enable();
    servoDriver7.enable();
    servoDriver8.enable();
}

void Disable()
{
    // shut off all outputs
    motorDriverA.disable();
    motorDriverB.disable();
    motorDriverC.disable();
    motorDriverD.disable();
    motorDriverE.disable();
    motorDriverF.disable();
    motorDriverG.disable();
    motorDriverH.disable();

    servoDriver1.disable();
    servoDriver2.disable();
    servoDriver3.disable();
    servoDriver4.disable();
    servoDriver5.disable();
    servoDriver6.disable();
    servoDriver7.disable();
    servoDriver8.disable();
}

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you shouldn't also call leftMotorController.run())

    delay(1);
}
void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    mA = EWD::recvFl();
    mB = EWD::recvFl();
    mC = EWD::recvFl();
    mD = EWD::recvFl();
    mE = EWD::recvFl();
    mF = EWD::recvFl();
    mG = EWD::recvFl();
    mH = EWD::recvFl();

    s1 = EWD::recvFl();
    s2 = EWD::recvFl();
    s3 = EWD::recvFl();
    s4 = EWD::recvFl();
    s5 = EWD::recvFl();
    s6 = EWD::recvFl();
    s7 = EWD::recvFl();
    s8 = EWD::recvFl();
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "router";
    EWD::routerPassword = "password";
    EWD::routerPort = 25210;

    // EWD::mode = EWD::Mode::createAP;
    // EWD::APName = "rcm0";
    // EWD::APPassword = "rcmPassword";
    // EWD::APPort = 25210;
}

#include "rcmutil.h"
