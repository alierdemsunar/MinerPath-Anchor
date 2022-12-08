//
// Created by Mert Demirbağ on 8.12.2022.
//

#include <Forwarder.h>
#include <EthernetUDP.h>


void Forwarder::setup(unsigned int LOCAL_PORT){
    UDP_PACKET.begin(LOCAL_PORT);
};

void Forwarder::sendPacket(IPAddress TERMINAL_IP, unsigned int TERMINAL_PORT, String packet){
    UDP_PACKET.beginPacket(TERMINAL_IP, TERMINAL_PORT);
    UDP_PACKET.print(packet);
    UDP_PACKET.endPacket();
}