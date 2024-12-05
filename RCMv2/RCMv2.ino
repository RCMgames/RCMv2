//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information see this page: https://github.com/RCMgames

/**
UNCOMMENT ONE OF THE FOLLOWING LINES DEPENDING ON WHAT HARDWARE YOU ARE USING
Remember to also choose the "environment" for your microcontroller in PlatformIO
*/
// #define RCM_HARDWARE_VERSION RCM_ORIGINAL // versions 1, 2, 3, and 3.1 of the original RCM hardware // https://github.com/RCMgames/RCM_hardware_documentation_and_user_guide
// #define RCM_HARDWARE_VERSION RCM_4_V1 // version 1 of the RCM 4 // https://github.com/RCMgames/RCM-Hardware-V4
// #define RCM_HARDWARE_VERSION RCM_BYTE_V2 // version 2 of the RCM BYTE // https://github.com/RCMgames/RCM-Hardware-BYTE
// #define RCM_HARDWARE_VERSION RCM_NIBBLE_V1 // version 1 of the RCM Nibble // https://github.com/RCMgames/RCM-Hardware-Nibble
// #define RCM_HARDWARE_VERSION RCM_D1_V1 // version 1 of the RCM D1 // https://github.com/RCMgames/RCM-Hardware-D1
// #define RCM_HARDWARE_VERSION ALFREDO_NOU2_NO_VOLTAGE_MONITOR // voltageComp will always report 10 volts https://www.alfredosys.com/products/alfredo-nou2/
// #define RCM_HARDWARE_VERSION ALFREDO_NOU2_WITH_VOLTAGE_MONITOR // modified to add resistors VIN-30k-D36-10k-GND https://www.alfredosys.com/products/alfredo-nou2/
#define RCM_HARDWARE_VERSION ALFREDO_NOU3 // https://www.alfredosys.com/products/alfredo-nou3/

/**
uncomment one of the following lines depending on which communication method you want to use
*/
#define RCM_COMM_METHOD RCM_COMM_EWD // use the normal communication method for RCM robots
// #define RCM_COMM_METHOD RCM_COMM_ROS // use the ROS communication method

#include "rcm.h" //defines pins

// set up motors and anything else you need here
// See this page for how to set up servos and motors for each type of RCM board:
// https://github.com/RCMgames/useful-code/tree/main/boards
// See this page for information about how to set up a robot's drivetrain using the JMotor library
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain

// all the servo drivers
JMotorDriverEsp32Servo servo1Driver = JMotorDriverEsp32Servo(servo1port);
JMotorDriverEsp32Servo servo2Driver = JMotorDriverEsp32Servo(servo2port);
JMotorDriverEsp32Servo servo3Driver = JMotorDriverEsp32Servo(servo3port);
JMotorDriverEsp32Servo servo4Driver = JMotorDriverEsp32Servo(servo4port);
JMotorDriverEsp32Servo servo5Driver = JMotorDriverEsp32Servo(servo5port);
JMotorDriverEsp32Servo servo6Driver = JMotorDriverEsp32Servo(servo6port);

// all the motor drivers
JMotorDriverPCA9685HBridge motor1Driver = JMotorDriverPCA9685HBridge(motor1port);
JMotorDriverPCA9685HBridge motor2Driver = JMotorDriverPCA9685HBridge(motor2port);
JMotorDriverPCA9685HBridge motor3Driver = JMotorDriverPCA9685HBridge(motor3port);
JMotorDriverPCA9685HBridge motor4Driver = JMotorDriverPCA9685HBridge(motor4port);
JMotorDriverPCA9685HBridge motor5Driver = JMotorDriverPCA9685HBridge(motor5port);
JMotorDriverPCA9685HBridge motor6Driver = JMotorDriverPCA9685HBridge(motor6port);
JMotorDriverPCA9685HBridge motor7Driver = JMotorDriverPCA9685HBridge(motor7port);
JMotorDriverPCA9685HBridge motor8Driver = JMotorDriverPCA9685HBridge(motor8port);

// variables for the drivers
float servo1Val = 0;
float servo2Val = 0;
float servo3Val = 0;
float servo4Val = 0;
float servo5Val = 0;
float servo6Val = 0;

float motor1Val = 0;
float motor2Val = 0;
float motor3Val = 0;
float motor4Val = 0;
float motor5Val = 0;
float motor6Val = 0;
float motor7Val = 0;
float motor8Val = 0;

void Enabled()
{
    // code to run while enabled, put your main code here
    // set all the motor drivers (you can put this in Enabled())
    servo1Driver.set(servo1Val);
    servo2Driver.set(servo2Val);
    servo3Driver.set(servo3Val);
    servo4Driver.set(servo4Val);
    servo5Driver.set(servo5Val);
    servo6Driver.set(servo6Val);

    motor1Driver.set(motor1Val);
    motor2Driver.set(motor2Val);
    motor3Driver.set(motor3Val);
    motor4Driver.set(motor4Val);
    motor5Driver.set(motor5Val);
    motor6Driver.set(motor6Val);
    motor7Driver.set(motor7Val);
    motor8Driver.set(motor8Val);

}

void Enable()
{
    // turn on outputs
    // enable all the motor drivers (you can put this in Enable())
    servo1Driver.enable();
    servo2Driver.enable();
    servo3Driver.enable();
    servo4Driver.enable();
    servo5Driver.enable();
    servo6Driver.enable();

    motor1Driver.enable();
    motor2Driver.enable();
    motor3Driver.enable();
    motor4Driver.enable();
    motor5Driver.enable();
    motor6Driver.enable();
    motor7Driver.enable();
    motor8Driver.enable();

}

void Disable()
{
    // turn off outputs
    // disable all the motor drivers (you can put this in Disable())
    servo1Driver.disable();
    servo2Driver.disable();
    servo3Driver.disable();
    servo4Driver.disable();
    servo5Driver.disable();
    servo6Driver.disable();

    motor1Driver.disable();
    motor2Driver.disable();
    motor3Driver.disable();
    motor4Driver.disable();
    motor5Driver.disable();
    motor6Driver.disable();
    motor7Driver.disable();
    motor8Driver.disable();

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

#if RCM_COMM_METHOD == RCM_COMM_EWD
void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    // receive values for all the variables (you can put this in WifiDataToParse())
    servo1Val = EWD::recvFl();
    servo2Val = EWD::recvFl();
    servo3Val = EWD::recvFl();
    servo4Val = EWD::recvFl();
    servo5Val = EWD::recvFl();
    servo6Val = EWD::recvFl();

    motor1Val = EWD::recvFl();
    motor2Val = EWD::recvFl();
    motor3Val = EWD::recvFl();
    motor4Val = EWD::recvFl();
    motor5Val = EWD::recvFl();
    motor6Val = EWD::recvFl();
    motor7Val = EWD::recvFl();
    motor8Val = EWD::recvFl();

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
#elif RCM_COMM_METHOD == RCM_COMM_ROS ////////////// ignore everything below this line unless you're using ROS mode /////////////////////////////////////////////
void ROSWifiSettings()
{
    // SSID, password, IP, port (on a computer run: sudo docker run -it --rm --net=host microros/micro-ros-agent:iron udp4 --port 8888 )
    set_microros_wifi_transports("router", "password", "10.25.21.1", 8888); // doesn't complete until it connects to the wifi network
    nodeName = "rcm_robot";
    // numSubscribers = 10; //change max number of subscribers
}

#include <example_interfaces/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
// and lots of other message types are available (see file available_ros2_types)
// #include <geometry_msgs/msg/twist.h>

// declare publishers
declarePub(battery, std_msgs__msg__Float32);

// // declare subscribers and write callback functions
// declareSubAndCallback(cmd_vel, geometry_msgs__msg__Twist);
// velCmd.x = cmd_velMsg->linear.x;
// velCmd.y = cmd_velMsg->linear.y;
// velCmd.theta = cmd_velMsg->angular.z;
// } // end of callback

void ROSbegin()
{
    // create publishers
    createPublisher(battery, std_msgs__msg__Float32, "/rcm/battery");
    batteryMsg.data = 0;

    // add subscribers
    // addSub(cmd_vel, geometry_msgs__msg__Twist, "/cmd_vel");
}

void ROSrun()
{
    rosSpin(1);
    // you can add more publishers here
    batteryMsg.data = voltageComp.getSupplyVoltage();
    publish(battery);
}
#endif

#include "rcmutil.h"
