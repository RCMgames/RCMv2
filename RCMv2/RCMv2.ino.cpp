# 1 "C:\\Users\\Joshua\\AppData\\Local\\Temp\\tmp4sficqqa"
#include <Arduino.h>
# 1 "C:/Users/Joshua/Desktop/RCMv2/RCMv2/RCMv2.ino"
# 9 "C:/Users/Joshua/Desktop/RCMv2/RCMv2/RCMv2.ino"
#define RCM_HARDWARE_VERSION RCM_ORIGINAL







#define RCM_COMM_METHOD RCM_COMM_EWD


#include "rcm.h"
void Enabled();
void Enable();
void Disable();
void PowerOn();
void Always();
void WifiDataToParse();
void WifiDataToSend();
void configWifi();
void ROSWifiSettings();
void ROSbegin();
void ROSrun();
#line 28 "C:/Users/Joshua/Desktop/RCMv2/RCMv2/RCMv2.ino"
void Enabled()
{

}

void Enable()
{

}

void Disable()
{

}

void PowerOn()
{

}

void Always()
{


    delay(1);
}

#if RCM_COMM_METHOD == RCM_COMM_EWD
void WifiDataToParse()
{
    enabled = EWD::recvBl();

}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());

}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "router";
    EWD::routerPassword = "password";
    EWD::routerPort = 25210;





}
#elif RCM_COMM_METHOD == RCM_COMM_ROS
void ROSWifiSettings()
{

    set_microros_wifi_transports("router", "password", "10.25.21.1", 8888);
    nodeName = "rcm_robot";

}

#include <example_interfaces/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>




declarePub(battery, std_msgs__msg__Float32);
# 105 "C:/Users/Joshua/Desktop/RCMv2/RCMv2/RCMv2.ino"
void ROSbegin()
{

    createPublisher(battery, std_msgs__msg__Float32, "/rcm/battery");
    batteryMsg.data = 0;



}

void ROSrun()
{
    rosSpin(1);

    batteryMsg.data = voltageComp.getSupplyVoltage();
    publish(battery);
}
#endif

#include "rcmutil.h"