//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information about the electronics, see the link at the top of this page: https://github.com/RCMgames

// #define RCM_HARDWARE_VERSION 10 // uncomment if you have an RCMByte board

#include "rcm.h" //defines pins

const int dacUnitsPerVolt = 440; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);
// set up motors and anything else you need here
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain
JMotorDriverEsp32L293 flMotorDriver = JMotorDriverEsp32L293(portA, true, false, false, 8000, 12);
JMotorDriverEsp32L293 frMotorDriver = JMotorDriverEsp32L293(portB, true, false, false, 8000, 12);
JMotorDriverEsp32L293 blMotorDriver = JMotorDriverEsp32L293(portC, true, false, false, 8000, 12);
JMotorDriverEsp32L293 brMotorDriver = JMotorDriverEsp32L293(portD, true, false, false, 8000, 12);
JMotorCompBasic motorCompensator = JMotorCompBasic(voltageComp, 1.7, 0.5); // volts per rps, min rps
JMotorControllerOpen flMotor = JMotorControllerOpen(flMotorDriver, motorCompensator, INFINITY, INFINITY, 10);
JMotorControllerOpen frMotor = JMotorControllerOpen(frMotorDriver, motorCompensator, INFINITY, INFINITY, 10);
JMotorControllerOpen blMotor = JMotorControllerOpen(blMotorDriver, motorCompensator, INFINITY, INFINITY, 10);
JMotorControllerOpen brMotor = JMotorControllerOpen(brMotorDriver, motorCompensator, INFINITY, INFINITY, 10);
JDrivetrainMecanum drivetrain = JDrivetrainMecanum(frMotor, flMotor, blMotor, brMotor, { 1, 1, 1 });

JTwoDTransform velCmd = { 0, 0, 0 };

void Enabled()
{
    drivetrain.setVel(velCmd);
    // code to run while enabled, put your main code here
}

void Enable()
{
    // turn on outputs
    drivetrain.enable();
}

void Disable()
{
    // turn off outputs
    drivetrain.disable();
}

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you shouldn't also call leftMotorController.run())
    drivetrain.run();
    delay(1);
}

#ifndef RCM_ROS
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

    EWD::mode = EWD::Mode::createAP;
    EWD::APName = "rcm0";
    EWD::APPassword = "rcmPassword";
    EWD::APPort = 25210;
}
#else ////////////// ignore everything below this line unless you're using ROS mode/////////////////////////////////////////////
void ROSWifiSettings()
{
    // SSID, password, IP, port (on a computer run: sudo docker run -it --rm --net=host microros/micro-ros-agent:iron udp4 --port 8888 )
    set_microros_wifi_transports("router", "password", "10.38.54.221", 8888);
    nodeName = "rcm_robot";
    // numSubscribers = 10;uiik
}

#include <example_interfaces/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
// and lots of other message types are available (see file available_ros2_types)
#include <geometry_msgs/msg/twist.h>

// declare publishers
declarePub(battery, std_msgs__msg__Float32);

// declare subscribers and write callback functions
declareSubAndCallback(cmd_vel, geometry_msgs__msg__Twist);
velCmd.x = cmd_velMsg->linear.x;
velCmd.y = cmd_velMsg->linear.y;
velCmd.theta = cmd_velMsg->angular.z;
} // end of callback

void ROSbegin()
{
    // create publishers
    createPublisher(battery, std_msgs__msg__Float32);
    batteryMsg.data = 0;

    addSub(cmd_vel, geometry_msgs__msg__Twist);
}

void ROSrun()
{
    // you can put publishers here
    batteryMsg.data = voltageComp.getSupplyVoltage();
    publish(battery);

    rosSpin(10);
}
#endif

#include "rcmutil.h"
