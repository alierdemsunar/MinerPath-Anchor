//
// Created by Ali Erdem Sunar on 30.11.2022.
//

#ifndef MINERPATH_ANCHOR_UWBCONFIGURATION_H
#define MINERPATH_ANCHOR_UWBCONFIGURATION_H


#include <Arduino.h>
#include "UWBConstant.h"

#define DW1000NG_PRINTABLE true
/**
 * Adds debug functionalities
 *
*/
#define DW1000NG_DEBUG false

/**
 * Optimizes code for the DWM1000 Module
 */
#define DWM1000_OPTIMIZED false

/**
 * Printable DW1000NgDeviceConfiguration about: rom:2494 byte ; ram 256 byte
 * This option is needed because compiler can not optimize unused codes from inheritanced methods
 * Some examples or debug code use this
 * Set false if you do not need it and have to save some space
 */
#define DW1000NGCONFIGURATION_H_PRINTABLE false

typedef struct device_configuration_t {
    boolean extendedFrameLength;
    boolean receiverAutoReenable;
    boolean smartPower;
    boolean frameCheck;
    boolean nlos;
    SFDMode sfd;
    Channel channel;
    DataRate dataRate;
    PulseFrequency pulseFreq;
    PreambleLength preambleLen;
    PreambleCode preaCode;
} device_configuration_t;

typedef struct interrupt_configuration_t {
    boolean interruptOnSent;
    boolean interruptOnReceived;
    boolean interruptOnReceiveFailed;
    boolean interruptOnReceiveTimeout;
    boolean interruptOnReceiveTimestampAvailable;
    boolean interruptOnAutomaticAcknowledgeTrigger;
} interrupt_configuration_t;

typedef struct frame_filtering_configuration_t {
    boolean behaveAsCoordinator;
    boolean allowBeacon;
    boolean allowData;
    boolean allowAcknowledgement;
    boolean allowMacCommand;
    boolean allowAllReserved;
    boolean allowReservedFour;
    boolean allowReservedFive;
} frame_filtering_configuration_t;

typedef struct sleep_configuration_t {
    boolean onWakeUpRunADC;
    boolean onWakeUpReceive;
    boolean onWakeUpLoadEUI;
    boolean onWakeUpLoadL64Param;
    boolean preserveSleep;
    boolean enableSLP;
    boolean enableWakePIN;
    boolean enableWakeSPI;
} sleep_configuration_t;


#endif //MINERPATH_ANCHOR_UWBCONFIGURATION_H
