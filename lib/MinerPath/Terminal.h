//
// Created by Ali Erdem Sunar on 25.11.2022.
//

#ifndef MINERPATH_ANCHOR_TERMINAL_H
#define MINERPATH_ANCHOR_TERMINAL_H

#include <Dns.h>

class Terminal {
public:
    static IPAddress terminalDNSLookup(const char *DOMAIN, IPAddress IP, bool DEVELOPMENT_MODE);
};


#endif //MINERPATH_ANCHOR_TERMINAL_H
