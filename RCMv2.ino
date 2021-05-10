//   https://github.com/rcmgames/RCMv2
#include <ESP32_easy_wifi_data.h>
#include "rcm.h"

void configWifi() { //see https://github.com/joshua-8/ESP32_easy_wifi_data/blob/master/examples/fullExample/fullExample.ino
  EWD::routerName = " "; //name of the wifi network you want to connect to
  EWD::routerPass = " "; //password for your wifi network (enter "-open-network-" if the network has no password) (default: -open-network-)
  EWD::wifiPort = 25220; //what port the esp32 communicates on if connected to a wifi network (default: 25210)
}

void Enabled() { //code to run while enabled
}

void Enable() { //turn on outputs
}

void Disable() { //shut off all outputs
}

void PowerOn() { //runs once on robot startup, set pinmodes

}

void Always() { //always runs if void loop is running, don't control outputs here

}

void WifiDataToParse() {
  enabled = EWD::recvBl();
  //add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)

}
void WifiDataToSend() {
  //EWD::sendFl(batteryVoltage);
  //add data to send here:

}

void setup() {
  PowerOn();
  pinMode(ONBOARD_LED, OUTPUT);
  Disable();
  configWifi();
  EWD::setupWifi(WifiDataToParse, WifiDataToSend);
}

void loop() {
  EWD::runWifiCommunication();
  if (!EWD::notTimedOut()) {
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
    digitalWrite(ONBOARD_LED, millis() % 500 < 250);
  } else {
    digitalWrite(ONBOARD_LED, HIGH);
  }
  wasEnabled = enabled;
}
