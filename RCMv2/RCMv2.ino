//   https://github.com/rcmgames/RCMv2
#include "rcm.h" //defines pins
#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0
#include <JMotor.h> //https://github.com/joshua-8/JMotor

// set up motors and anything else you need here

JTwoDTransform input;
JTwoDTransform output;

const int dacUnitsPerVolt = 380;
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);

// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain
JMotorDriverEsp32L293 leftDriver = JMotorDriverEsp32L293(portA); // left motor is connected to port A
// motor_stop_voltage, motor_stop_speed, motor_start_voltage, motor_start_speed, motor_high_voltage, motor_high_speed, start_boost_time
JMotorCompStandardConfig leftMotorConfig = JMotorCompStandardConfig(1.9, .553, 3.2, 1.24, 4.6, 1.89, 100);
JMotorCompStandard leftMotorCompensator = JMotorCompStandard(voltageComp, leftMotorConfig, 100);
JMotorControllerOpen lMotor = JMotorControllerOpen(leftDriver, leftMotorCompensator, INFINITY, INFINITY, 50000);

JMotorDriverEsp32L293 rightDriver
    = JMotorDriverEsp32L293(portD); // right motor is connected to port D
// motor_stop_voltage, motor_stop_speed, motor_start_voltage, motor_start_speed, motor_high_voltage, motor_high_speed, start_boost_time
JMotorCompStandardConfig rightMotorConfig = JMotorCompStandardConfig(1.9, .553, 3.2, 1.24, 4.6, 1.89, 100);
JMotorCompStandard rightMotorCompensator = JMotorCompStandard(voltageComp, rightMotorConfig, 100);
JMotorControllerOpen rMotor = JMotorControllerOpen(rightDriver, rightMotorCompensator, INFINITY, INFINITY, 50000);

JDrivetrainTwoSide drivetrain = JDrivetrainTwoSide(lMotor, rMotor, 0.15); // width
JDrivetrainControllerBasic drive = JDrivetrainControllerBasic(drivetrain, { INFINITY, INFINITY, INFINITY }, { .05, 0, 1 }, { .01, .01, .01 }, false);

float servoVal = 0;

JMotorDriverEsp32Servo servoDriver = JMotorDriverEsp32Servo(port3);
//    JServoController(JMotorDriverServo& _servo, bool _reverse = false, float velLimit = INFINITY, float accelLimit = INFINITY, unsigned long _disableTimeout = 0, float _minAngleLimit = 0, float _maxAngleLimit = 180, float _pos = 90, float _minSetAngle = 0, float _maxSetAngle = 180, int minServoVal = 544, int maxServoVal = 2400, bool _preventGoingWrongWay = true, bool _preventGoingTooFast = true, float _stoppingDecelLimit = INFINITY)
JServoController servo = JServoController(servoDriver, false, 100, 25, 100);

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
    output = JDeadzoneRemover::calculate(input, { 0, 0, 0 }, drive.getMaxVel());
    output = JCurvatureDrive::calculate(true, output, 0.25);
    drive.moveVel(output);

    servo.setPosTarget(servoVal);
}

void Enable()
{
    // turn on outputs
    drive.enable();
    servo.enable();
}

void Disable()
{
    // shut off all outputs
    drive.disable();
    servo.disable();
}

void PowerOn()
{
    // runs once on robot startup, set pin modes
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions can be put here
    drive.run();
    servo.run();
    delay(1);
}

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    input = { EWD::recvFl(), -EWD::recvFl(), -EWD::recvFl() };
    EWD::recvFl();
    servoVal = (EWD::recvFl() + 1.0) * 90.0;
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
