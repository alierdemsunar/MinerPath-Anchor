//
// Created by Mert Demirbağ on 8.12.2022.
//

#ifndef MINERPATH_ANCHOR_NETWORK_H
#define MINERPATH_ANCHOR_NETWORK_H
#include "Arduino.h"
#include <Dns.h>

class Network {
public:
    Network(boolean DEVELOPMENT_MODE);
    void setup(byte ANCHOR_MAC_ADDRESS[6]);
    IPAddress TERMINAL_IP = IPAddress(0, 0, 0, 0);
private:
   boolean DEVELOPMENT_MODE;
    // MinerPath IP address
};


#endif //MINERPATH_ANCHOR_NETWORK_H
