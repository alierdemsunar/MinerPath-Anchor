//
// Created by Mert Demirbağ on 8.12.2022.
//



#include "Network.h"
#include "EthernetUDP.h"
#include "Arduino.h"

Network::Network(boolean DEVELOPMENT_MODE){
    this->DEVELOPMENT_MODE = DEVELOPMENT_MODE;
}

void Network::setup(byte ANCHOR_MAC_ADDRESS[6]){
    EthernetClass::begin(ANCHOR_MAC_ADDRESS);

    if(DEVELOPMENT_MODE) {
        while (!Serial) {}
        Serial.println("[?] Trying to activate ethernet module...");
        if (EthernetClass::begin(ANCHOR_MAC_ADDRESS) == 0) {
            Serial.println("[X] Failed to configure ethernet MAC address!");
            if (EthernetClass::hardwareStatus() == EthernetNoHardware) {
                Serial.println("[X] Network module was not found!");
            } else if (EthernetClass::linkStatus() == LinkOFF) {
                Serial.println("[X] Network cable is not connected!");
            }
        } else {
            Serial.print("[!] MinerPath-Anchor MAC: ");Serial.print(ANCHOR_MAC_ADDRESS[0], HEX);Serial.print(":");Serial.print(ANCHOR_MAC_ADDRESS[1], HEX);Serial.print(":");Serial.print(ANCHOR_MAC_ADDRESS[2], HEX);Serial.print(":");Serial.print(ANCHOR_MAC_ADDRESS[3], HEX);Serial.print(":");Serial.print(ANCHOR_MAC_ADDRESS[4], HEX);Serial.print(":");Serial.println(ANCHOR_MAC_ADDRESS[5], HEX);
            Serial.print("[!] MinerPath-Anchor IP: ");Serial.print(EthernetClass::localIP());Serial.print("/");Serial.print(EthernetClass::subnetMask());Serial.print("/");Serial.println(EthernetClass::gatewayIP());
            Serial.print("[!] MinerPath-Anchor DNS: ");Serial.println(EthernetClass::dnsServerIP());
            Serial.println("[✓] Network module activated.");
        }
    }
}


