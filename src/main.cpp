#include <Arduino.h>
#include <Terminal.h>
#include <Forwarder.h>
#include <Network.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

//***UNIQUE PARAMETERS START***//
byte ANCHOR_MAC_ADDRESS[6] = {0x90, 0x92, 0xBE, 0xEF, 0xFE, 0xEF}; // Anchor MAC address
byte eui[8] = {0x77, 0x77, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x1C};
//***UNIQUE PARAMETERS END***//

//***COMMON PARAMETERS START***//
const boolean DEVELOPMENT_MODE = true; // Development mode switch
char TERMINAL_DOMAIN[32] = "terminal.minerpath.local"; // Gateway domain address
unsigned int LOCAL_PORT = 6792; // Local Port
unsigned int TERMINAL_PORT = 6792; // Gateway Port
Forwarder *forwarder = new Forwarder(); // Forwarder Starter
Network *network = new Network(DEVELOPMENT_MODE); // Network Starter
//***COMMON PARAMETERS END***//

// connection pins
const uint8_t PIN_RST = 3; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = 48; // spi select pin
// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define BLINK 4
#define BLINK_REPORT 5
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;

uint64_t timeComputedRange;
// last computed range/time
// data buffer
#define LEN_DATA 33
#define MAX_DEVICE 10
byte data[LEN_DATA];
byte msgNum;
uint64_t receivedTagEUI[MAX_DEVICE][2];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
uint32_t receivedTagCount = 0;
uint64_t currentTag = 0;
float samplingRate = 0;

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

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
        true,
        true,
        true,
        false,
        true
};

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void receiver() {
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually
    DW1000Ng::startReceive();
}

void resetInactive() {
    // anchor listens for POLL
    expectedMsgId = POLL;
    receiver();
    noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
}

void transmitPollAck() {
    data[33] = msgNum;
    data[0] = POLL_ACK;
    // write EUI to data
    for (int i = 0; i < 8; i++) {
        DW1000NgUtils::writeValueToBytes(data + 24 + i, eui[i], 1);
    }
    DW1000NgUtils::writeValueToBytes(data + 16, currentTag, 8);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitBlinkReport() {
    data[0] = BLINK_REPORT;
    // write EUI to data
    for (int i = 0; i < 8; i++) {
        DW1000NgUtils::writeValueToBytes(data + 24 + i, eui[i], 1);
    }
    DW1000NgUtils::writeValueToBytes(data + 16, currentTag, 8);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRangeReport(float curRange) {
    data[33] = msgNum;
    data[0] = RANGE_REPORT;
    // write EUI to data
    for (int i = 0; i < 8; i++) {
        DW1000NgUtils::writeValueToBytes(data + 24 + i, eui[i], 1);
    }
    // write final ranging result
    memcpy(data + 1, &curRange, 4);
    DW1000NgUtils::writeValueToBytes(data + 16, currentTag, 8);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRangeFailed() {
    data[0] = RANGE_FAILED;

    // write EUI to data
    for (int i = 0; i < 8; i++) {
        DW1000NgUtils::writeValueToBytes(data + 24 + i, eui[i], 1);
    }
    DW1000NgUtils::writeValueToBytes(data + 24, currentTag, 8);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}


void setup() {
//***MINERPATH SETUP START***//
    if (DEVELOPMENT_MODE) {
        Serial.begin(115200);
    }
    network->setup(ANCHOR_MAC_ADDRESS);
    network->TERMINAL_IP = Terminal::terminalDNSLookup(TERMINAL_DOMAIN, network->TERMINAL_IP, DEVELOPMENT_MODE);
    forwarder->setup(LOCAL_PORT);
//***MINERPATH SETUP END***//

    // DEBUG monitoring
    Serial.println(F("### DW1000Ng-arduino-ranging-anchor ###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

    DW1000Ng::setDeviceAddress(1);

    DW1000Ng::setAntennaDelay(16436);

    DW1000Ng::setEUI(eui);

    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: ");
    Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: ");
    Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: ");
    Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: ");
    Serial.println(msg);
    // attach callback for (successfully) sent and received messages
    DW1000Ng::attachSentHandler(handleSent);
    DW1000Ng::attachReceivedHandler(handleReceived);
    // anchor starts in receiving mode, awaiting a ranging poll message

    receiver();
    noteActivity();
    // for first time ranging frequency computation
    rangingCountPeriod = millis();

    //
    int i;
    for (i = 0; i < MAX_DEVICE; i++) {
        receivedTagEUI[i][0] = 0;
        receivedTagEUI[i][1] = 0;
    }
}


void loop() {
    //forwarder->sendPacket(network->TERMINAL_IP,TERMINAL_PORT,"Deneme22");

    //TCP
    int32_t curMillis = millis();
    int i;
    for (i = 0; i < MAX_DEVICE; i++) {
        if (curMillis - receivedTagEUI[i][1] > 300) {
            receivedTagEUI[i][0] = 0;
            receivedTagEUI[i][1] = 0;
        }
    }
    if (!sentAck && !receivedAck) {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {
            resetInactive();
        }
        return;
    }
    // continue on any success confirmation
    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL_ACK) {
            timePollAckSent = DW1000Ng::getTransmitTimestamp();
            noteActivity();
        }
        DW1000Ng::startReceive();
    }
    if (receivedAck) {
        receivedAck = false;
        //add to array
        // get message and parse
        DW1000Ng::getReceivedData(data, LEN_DATA);

        byte msgId = data[0];


        if (msgId != expectedMsgId) {
            // unexpected message, start over again (except if already POLL)
            protocolFailed = true;
        }

        if (msgId == BLINK) {
            // on POLL we (re-)start, so no protocol failure
            protocolFailed = false;


            for (i = 0; i < MAX_DEVICE; i++) {
                if (DW1000NgUtils::bytesAsValue(data + 16, 8) == receivedTagEUI[i][0]) {
                    return;
                }
            }
            currentTag = DW1000NgUtils::bytesAsValue(data + 16, 8);
            receivedTagCount++;
            if(receivedTagCount >= MAX_DEVICE)
                receivedTagCount = 0;
            receivedTagEUI[receivedTagCount][0] = DW1000NgUtils::bytesAsValue(data + 16, 8);
            receivedTagEUI[receivedTagCount][1] = curMillis;

            expectedMsgId = POLL;
            transmitBlinkReport();
            noteActivity();
        } else if (msgId == POLL) {
            msgNum = data[33];
            // on POLL we (re-)start, so no protocol failure
            protocolFailed = false;
            if (currentTag != DW1000NgUtils::bytesAsValue(data + 16, 8) ||
                DW1000NgUtils::bytesAsValue(eui, 8) != DW1000NgUtils::bytesAsValue(data + 24, 8)) {
                transmitRangeFailed();
                return;
            }
            expectedMsgId = RANGE;


            timePollReceived = DW1000Ng::getReceiveTimestamp();
            transmitPollAck();
            noteActivity();
        } else if (msgId == RANGE) {

            expectedMsgId = POLL;

            if (currentTag != DW1000NgUtils::bytesAsValue(data + 16, 8) ||
                DW1000NgUtils::bytesAsValue(eui, 8) != DW1000NgUtils::bytesAsValue(data + 24, 8) ||
                msgNum != data[33]) {
                transmitRangeFailed();
                return;
            }
            timeRangeReceived = DW1000Ng::getReceiveTimestamp();

            if (!protocolFailed) {
                timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
                timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
                timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
                // (re-)compute range as two-way ranging is done
                double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                                          timePollReceived,
                                                                          timePollAckSent,
                                                                          timePollAckReceived,
                                                                          timeRangeSent,
                                                                          timeRangeReceived);
                /* Apply simple bias correction */
                distance = DW1000NgRanging::correctRange(distance);
                if (distance < 0) {
                    transmitRangeFailed();
                    return;
                }
                String rangeString = "Range: ";
                rangeString += distance;
                rangeString += " m";
                rangeString += "\t RX power: ";
                rangeString += DW1000Ng::getReceivePower();
                rangeString += " dBm";
                rangeString += "\t Sampling: ";
                rangeString += samplingRate;
                rangeString += " Hz";
                char ts[128];
                sprintf(ts, "%u", timeRangeSent);
                rangeString += "\t Time Range Sent: ";
                rangeString += ts;
                char msg[128];
                sprintf(msg, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                        data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23]);
                rangeString += "\t Tag EUI: ";
                rangeString += msg;


                Serial.println(rangeString);


                //Serial.print("FP power is [dBm]: "); Serial.print(DW1000Ng::getFirstPathPower());
                //Serial.print("RX power is [dBm]: "); Serial.println(DW1000Ng::getReceivePower());
                //Serial.print("Receive quality: "); Serial.println(DW1000Ng::getReceiveQuality());
                // update sampling rate (each second)


                transmitRangeReport(distance * DISTANCE_OF_RADIO_INV);
                expectedMsgId = BLINK;
                successRangingCount++;
                if (curMillis - rangingCountPeriod > 100) {
                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                    rangingCountPeriod = curMillis;
                    successRangingCount = 0;
                }
            } else {
                transmitRangeFailed();
            }

            noteActivity();
        }
    }

}