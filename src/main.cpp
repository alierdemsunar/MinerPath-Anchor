#include <Arduino.h>
#include <EthernetUDP.h>
#include <Terminal.h>
#include "UWBMain.h"

//***UNIQUE PARAMETERS***//
#if defined(ESP8266)
const uint8_t PIN_RST = 5; // reset pin
const uint8_t PIN_IRQ = 4; // irq pin
const uint8_t PIN_SS = 15; // spi select pin
#else
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin
#endif


char TERMINAL_DOMAIN[32] = "terminal.minerpath.local"; // Terminal domain address
byte ANCHOR_MAC_ADDRESS[6] = {0x90, 0x92, 0xBE, 0xEF, 0xFE, 0xEF}; // Anchor MAC address
const boolean DEVELOPMENT_MODE = true; // Development mode switch

//***ANCHOR PARAMETERS***//
IPAddress TERMINAL_IP(0, 0, 0, 0); // Terminal IP address
unsigned int TERMINAL_PORT = 6792; // Terminal Port
unsigned int LOCAL_PORT = 6792; // Local Port
EthernetUDP UDP_PACKET; // EthernetUDP Class

//CONFIGURATIONS
device_configuration_t DEFAULT_CONFIG = {
        false,
        true,
        true,
        true,
        false,
        SFDMode::STANDARD_SFD,
        Channel::CHANNEL_5,
        DataRate::RATE_850KBPS,
        PulseFrequency::FREQ_16MHZ,
        PreambleLength::LEN_256,
        PreambleCode::CODE_3
};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
        false,
        false,
        true,
        false,
        false,
        false,
        false,
        false
};


void setup() {
    Serial.begin(115200);

    EthernetClass::begin(ANCHOR_MAC_ADDRESS);
    //***DEVELOPMENT ONLY CODE START***//
    if(DEVELOPMENT_MODE) {
        while (!Serial) {}
        Serial.println("[?] Trying to activate ethernet module...");
        if (EthernetClass::begin(ANCHOR_MAC_ADDRESS) == 0) {
            Serial.println("[X] Failed to configure ethernet MAC address!");
            if (EthernetClass::hardwareStatus() == EthernetNoHardware) {
                Serial.println("[X] Ethernet module was not found!");
            } else if (EthernetClass::linkStatus() == LinkOFF) {
                Serial.println("[X] Ethernet cable is not connected!");
            }
        } else {
            Serial.print("[!] MinerPath-Anchor MAC: ");Serial.print(ANCHOR_MAC_ADDRESS[0], HEX);Serial.print(":");Serial.print(ANCHOR_MAC_ADDRESS[1], HEX);Serial.print(":");Serial.print(ANCHOR_MAC_ADDRESS[2], HEX);Serial.print(":");Serial.print(ANCHOR_MAC_ADDRESS[3], HEX);Serial.print(":");Serial.print(ANCHOR_MAC_ADDRESS[4], HEX);Serial.print(":");Serial.println(ANCHOR_MAC_ADDRESS[5], HEX);
            Serial.print("[!] MinerPath-Anchor IP: ");Serial.print(EthernetClass::localIP());Serial.print("/");Serial.print(EthernetClass::subnetMask());Serial.print("/");Serial.println(EthernetClass::gatewayIP());
            Serial.print("[!] MinerPath-Anchor DNS: ");Serial.println(EthernetClass::dnsServerIP());
            Serial.println("[✓] Ethernet module activated.");
        }
    }
    //***DEVELOPMENT ONLY CODE END***//
    TERMINAL_IP = Terminal::terminalDNSLookup(TERMINAL_DOMAIN, TERMINAL_IP, DEVELOPMENT_MODE);
    UDP_PACKET.begin(LOCAL_PORT);
    // DEBUG monitoring
    // initialize the driver
    UWBMain::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    UWBMain::setDeviceAddress(5);
    UWBMain::setNetworkId(10);
    Serial.println(F("Committed configuration ..."));
    // wait a bit
    delay(1000);
}

void loop() {
    UDP_PACKET.beginPacket(TERMINAL_IP, TERMINAL_PORT);
    UDP_PACKET.write("Anchor IP: ");UDP_PACKET.write(EthernetClass::localIP());
    UDP_PACKET.endPacket();
    delay(1000);
    char msg[128];
    UWBMain::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    UWBMain::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    UWBMain::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    //Terminal::foo();
}