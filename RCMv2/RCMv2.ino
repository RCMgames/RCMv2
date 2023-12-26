//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information about the electronics, see the link at the top of this page: https://github.com/RCMgames

//    #define RCM_HARDWARE_VERSION 10 // uncomment if you have an RCMByte board

#include "rcm.h" //defines pins

const int dacUnitsPerVolt = 440; // increasing this number decreases the calculated voltage
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
}

void Disable()
{
    // turn off outputs
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
