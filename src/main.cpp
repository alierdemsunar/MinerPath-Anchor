#include <Arduino.h>
#include <Terminal.h>
#include <Forwarder.h>
#include <Network.h>

//***UNIQUE PARAMETERS***//
byte ANCHOR_MAC_ADDRESS[6] = {0x90, 0x92, 0xBE, 0xEF, 0xFE, 0xEF}; // Anchor MAC address
const boolean DEVELOPMENT_MODE = true; // Development mode switch

//***COMMON PARAMETERS***//
const boolean DEVELOPMENT_MODE = false; // Development mode switch
char TERMINAL_DOMAIN[32] = "terminal.minerpath.local"; // Gateway domain address
unsigned int LOCAL_PORT = 6792; // Local Port
unsigned int TERMINAL_PORT = 6792; // Gateway Port
Forwarder* forwarder= new Forwarder();
Network* network = new Network(DEVELOPMENT_MODE);

void setup() {
    if(DEVELOPMENT_MODE) {
        Serial.begin(115200);
    }
    network->setup(ANCHOR_MAC_ADDRESS);
    network->TERMINAL_IP = Terminal::terminalDNSLookup(TERMINAL_DOMAIN, network->TERMINAL_IP, DEVELOPMENT_MODE);
    forwarder->setup(LOCAL_PORT);
}

void loop() {
    UDP_PACKET.beginPacket(TERMINAL_IP, TERMINAL_PORT);
    UDP_PACKET.write("Anchor IP: ");UDP_PACKET.write(EthernetClass::localIP());
    UDP_PACKET.endPacket();
    delay(1000);
}