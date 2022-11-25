#include <Arduino.h>
#include <EthernetUDP.h>
#include <Dns.h>
#include <Terminal.h>
//***UNIQUE PARAMETERS***//
char TERMINAL_DOMAIN[32] = "terminal.minerpath.local"; // Terminaal domain address
byte ANCHOR_MAC_ADDRESS[6] = {0x90, 0x92, 0xBE, 0xEF, 0xFE, 0xEF}; // Anchor MAC address
const boolean DEVELOPMENT_MODE = true; // Development mode switch

//***ANCHOR PARAMETERS***//
IPAddress TERMINAL_IP(0, 0, 0, 0); // Terminaal IP address
unsigned int TERMINAL_PORT = 6792; // Terminaal Port
unsigned int LOCAL_PORT = 6792; // Local Port
EthernetUDP UDP_PACKET; // EthernetUDP Class

//***TERMINAL DNS LOOKUP FUNCTION***//







void setup() {
    //***DEVELOPMENT ONLY CODE START***//
    if(DEVELOPMENT_MODE) {
        Serial.begin(115200);
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
}

void loop() {
    UDP_PACKET.beginPacket(TERMINAL_IP, TERMINAL_PORT);
    UDP_PACKET.write("Anchor IP: ");UDP_PACKET.write(EthernetClass::localIP());
    UDP_PACKET.endPacket();
    delay(1000);
    //Terminal::foo();
}