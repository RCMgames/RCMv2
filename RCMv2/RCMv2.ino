//   https://github.com/rcmgames/RCMv2
#include "rcm.h" //defines pins
#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0
#include <JMotor.h> //https://github.com/joshua-8/JMotor

// set up motors and anything else you need here

float speed = 0;
float turn = 0;

float lSpeed = 0;
float rSpeed = 0;

const int dacUnitsPerVolt = 380;
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);

// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain
JMotorDriverEsp32L293 leftDriver = JMotorDriverEsp32L293(portA); // left motor is connected to port A
// motor_stop_voltage, motor_stop_speed, motor_start_voltage, motor_start_speed, motor_high_voltage, motor_high_speed, start_boost_time
JMotorCompStandardConfig leftMotorConfig = JMotorCompStandardConfig(1.9, .553, 3.2, 1.24, 4.6, 1.89, 100);
JMotorCompStandard leftMotorCompensator = JMotorCompStandard(voltageComp, leftMotorConfig, 100);
JMotorControllerOpen lMotor = JMotorControllerOpen(leftDriver, leftMotorCompensator);

JMotorDriverEsp32L293 rightDriver
    = JMotorDriverEsp32L293(portD); // right motor is connected to port D
// motor_stop_voltage, motor_stop_speed, motor_start_voltage, motor_start_speed, motor_high_voltage, motor_high_speed, start_boost_time
JMotorCompStandardConfig rightMotorConfig = JMotorCompStandardConfig(1.9, .553, 3.2, 1.24, 4.6, 1.89, 100);
JMotorCompStandard rightMotorCompensator = JMotorCompStandard(voltageComp, rightMotorConfig, 100);
JMotorControllerOpen rMotor = JMotorControllerOpen(rightDriver, rightMotorCompensator);

JDrivetrainTwoSide drivetrain = JDrivetrainTwoSide(lMotor, rMotor, 0.15); // width
JDrivetrainControllerBasic drive = JDrivetrainControllerBasic(drivetrain, { INFINITY, INFINITY, INFINITY }, { .05, 0, 1 }, { 0, 0, 0 });

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
    drive.moveVel({ speed * drive.getMaxVel().x, 0, turn * drive.getMaxVel().theta });
}

void Enable()
{
    // turn on outputs
    drive.enable();
}

void Disable()
{
    // shut off all outputs
    drive.disable();
}

void PowerOn()
{
    // runs once on robot startup, set pin modes
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions can be put here
    drive.run();
    delay(1);
}

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    speed = EWD::recvFl();
    EWD::recvFl();
    turn = -EWD::recvFl(); // inverted to translate from standard rcmDS reference frame to JMotor/ROS standard
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
