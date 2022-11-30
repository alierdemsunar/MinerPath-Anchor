//
// Created by Ali Erdem Sunar on 25.11.2022.
//

#include <Terminal.h>
#include <Dns.h>

IPAddress Terminal::terminalDNSLookup(const char *DOMAIN, IPAddress IP, bool DEVELOPMENT_MODE) {
    DNSClient DNS_CLIENT;
    DNS_CLIENT.begin(EthernetClass::dnsServerIP());
    DNS_CLIENT.getHostByName(DOMAIN, IP);
    //***DEVELOPMENT ONLY CODE START***//
    if (DEVELOPMENT_MODE) {
        Serial.println("[?] Trying to find terminal IP via DNS...");
        if (DNS_CLIENT.getHostByName(DOMAIN, IP) == 1) {
            Serial.print("[!] Terminal IP: ");
            Serial.print(IP);
            Serial.print(" (");
            Serial.print(DOMAIN);
            Serial.println(")");
        } else {
            Serial.print("[X] IP of \"");
            Serial.print(DOMAIN);
            Serial.println("\" terminal was not found via DNS!");
        }
    }
    //***DEVELOPMENT ONLY CODE END***//
    if (DNS_CLIENT.getHostByName(DOMAIN, IP) == 1) {
        return IP;
    }
    else
        return IPAddress(0,0,0,0);
}
