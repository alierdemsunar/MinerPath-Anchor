//
// Created by Mert Demirbağ on 8.12.2022.
//

#ifndef MINERPATH_ANCHOR_FORWARDER_H
#define MINERPATH_ANCHOR_FORWARDER_H

#include <EthernetUDP.h>

class Forwarder {
public:
    void sendPacket(IPAddress TERMINAL_IP, unsigned int TERMINAL_PORT, String packet);
    void setup(unsigned int LOCAL_PORT);
private:
    EthernetUDP UDP_PACKET;
};


#endif //MINERPATH_ANCHOR_FORWARDER_H


 // EthernetUDP Class




