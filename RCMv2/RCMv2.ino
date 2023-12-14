//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information about the electronics, see the link at the top of this page: https://github.com/RCMgames

#define RCM_HARDWARE_VERSION 10 // uncomment if you have an RCMByte board

#include "rcm.h" //defines pins

const int dacUnitsPerVolt = 380; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);
// set up motors and anything else you need here
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain

void Enabled()
{
    // code to run while enabled, put your main code here
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
    digitalWrite(motorsEnablePin, HIGH);
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
    digitalWrite(motorsEnablePin, LOW); // to be safe
}

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you shouldn't also call leftMotorController.run())

    // turns on three motors at a time, cycling through all motors
    int i = (millis() % 8000) / 1000;
    RSLcolor = i == 0 ? CRGB(255, 255, 255) : CRGB(255, 0, 255);
    motorDriverA.set((i <= 2) * motorDriverA.getMaxRange());
    motorDriverB.set((i >= 1 && i <= 3) * motorDriverB.getMaxRange());
    motorDriverC.set((i >= 2 && i <= 4) * motorDriverC.getMaxRange());
    motorDriverD.set((i >= 3 && i <= 5) * motorDriverD.getMaxRange());
    motorDriverE.set((i >= 4 && i <= 6) * motorDriverE.getMaxRange());
    motorDriverF.set((i >= 5) * motorDriverF.getMaxRange());
    motorDriverG.set((i >= 6 || i <= 0) * motorDriverG.getMaxRange());
    motorDriverH.set((i >= 7 || i <= 1) * motorDriverH.getMaxRange());

    delay(1);
}

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
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
