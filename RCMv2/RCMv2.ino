//   https://github.com/rcmgames/RCMv2
#include "rcm.h" //defines pins
#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0
#include <JMotor.h> //https://github.com/joshua-8/JMotor

// set up motors and anything else you need here
const int dacUnitsPerVolt = 380;
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);

// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain

void configWifi()
{

    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "router";
    EWD::routerPassword = "password";
    EWD::routerPort = 25210;

    // EWD::mode = EWD::Mode::createAP;
    // EWD::APName = "rcm0";
    // EWD::APPassword = "rcmpassword";
    // EWD::APPort = 25210;
}

void Enabled()
{
    // code to run while enabled
}

void Enable()
{
    // turn on outputs
}

void Disable()
{
    // shut off all outputs
}

void PowerOn()
{
    // runs once on robot startup, set pin modes
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions can be put here
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

void setup()
{ // don't edit
    Serial.begin(115200);
    PowerOn();
    pinMode(ONBOARD_LED, OUTPUT);
    Disable();
    configWifi();
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);
}

void loop()
{ // don't edit
    EWD::runWifiCommunication();
    if (!EWD::wifiConnected || EWD::timedOut()) {
        enabled = false;
    }
    Always();
    if (enabled && !wasEnabled) {
        Enable();
    }
    if (!enabled && wasEnabled) {
        Disable();
    }
    if (enabled) {
        Enabled();
        digitalWrite(ONBOARD_LED, millis() % 500 < 250); // flash, enabled
    } else {
        if (!EWD::wifiConnected)
            digitalWrite(ONBOARD_LED, millis() % 1000 <= 100); // short flash, wifi connection fail
        else if (EWD::timedOut())
            digitalWrite(ONBOARD_LED, millis() % 1000 >= 100); // long flash, no driver station connected
        else
            digitalWrite(ONBOARD_LED, HIGH); // on, disabled
    }
    wasEnabled = enabled;
}
