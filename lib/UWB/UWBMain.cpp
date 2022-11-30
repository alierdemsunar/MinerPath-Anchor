//
// Created by Ali Erdem Sunar on 30.11.2022.
//

#include "UWBMain.h"

#include <stdlib.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "UWBConstant.h"
#include "UWBConfiguration.h"
#include "UWBRegister.h"
#include "UWBPorting.h"
#include "UWBUtil.h"
//compile options #include "DW1000NgCompileOptions.hpp"

namespace UWBMain {
    namespace {

        /* ########################### PRIVATE VARIABLES ################################# */

        /* SPI select pin and interrupt pin*/
        uint8_t _ss = 0xff;
        uint8_t _irq = 0xff;
        uint8_t _rst = 0xff;

        /* IRQ callbacks */
        void (*_handleSent)(void) = nullptr;

        void (*_handleError)(void) = nullptr;

        void (*_handleReceived)(void) = nullptr;

        void (*_handleReceiveFailed)(void) = nullptr;

        void (*_handleReceiveTimeout)(void) = nullptr;

        void (*_handleReceiveTimestampAvailable)(void) = nullptr;

        /* registers */
        byte _syscfg[LEN_SYS_CFG];
        byte _sysctrl[LEN_SYS_CTRL];
        byte _sysstatus[LEN_SYS_STATUS];
        byte _txfctrl[LEN_TX_FCTRL];
        byte _sysmask[LEN_SYS_MASK];
        byte _chanctrl[LEN_CHAN_CTRL];
        byte _networkAndAddress[LEN_PANADR];

        /* Temperature and Voltage monitoring */
        byte _vmeas3v3 = 0;
        byte _tmeas23C = 0;

        /* Driver Internal State Trackers */
        byte _extendedFrameLength;
        PacSize _pacSize;
        PulseFrequency _pulseFrequency;
        DataRate _dataRate;
        PreambleLength _preambleLength;
        PreambleCode _preambleCode;
        Channel _channel;
        boolean _smartPower;
        boolean _frameCheck;
        boolean _debounceClockEnabled = false;
        boolean _nlos = false;
        boolean _standardSFD = true;
        boolean _autoTXPower = true;
        boolean _autoTCPGDelay = true;
        boolean _wait4resp = false;
        uint16_t _antennaTxDelay = 0;
        uint16_t _antennaRxDelay = 0;

    }
    /*
		* Read bytes from the DW1000. Number of bytes depend on register length.
		* @param[in] cmd
		* 		The register address (see Chapter 7 in the DW1000Ng user manual).
		* @param[in] offset
		*		The number of bytes expected to be received.
		* @param[out] data
		*		The data array to be read into.
		* @param[in] data_size
		*
     * */
    void _readBytesFromRegister(byte cmd, uint16_t offset, byte data[], uint16_t data_size) {
        byte header[3];
        uint8_t headerLen = 1;

        // build SPI header
        if(offset == NO_SUB) {
            header[0] = READ | cmd;
        } else {
            header[0] = READ_SUB | cmd;
            if(offset < 128) {
                header[1] = (byte)offset;
                headerLen++;
            } else {
                header[1] = RW_SUB_EXT | (byte)offset;
                header[2] = (byte)(offset >> 7);
                headerLen += 2;
            }
        }

        UWBPorting::readFromSPI(_ss, headerLen, header, data_size, data);
    }

    void _writeBytesToRegister(byte cmd, uint16_t offset, byte data[], uint16_t data_size) {
        byte header[3];
        uint8_t headerLen = 1;

        // TODO proper error handling: address out of bounds
        // build SPI header
        if(offset == NO_SUB) {
            header[0] = WRITE | cmd;
        } else {
            header[0] = WRITE_SUB | cmd;
            if(offset < 128) {
                header[1] = (byte)offset;
                headerLen++;
            } else {
                header[1] = RW_SUB_EXT | (byte)offset;
                header[2] = (byte)(offset >> 7);
                headerLen += 2;
            }
        }

        UWBPorting::writeToSPI(_ss, headerLen, header, data_size, data);
    }
    void _writeValueToRegister(byte cmd, uint16_t offset, uint32_t data, uint16_t data_size) {
        byte dataBytes[data_size];
        UWBUtil::writeValueToBytes(dataBytes, data, data_size);
        _writeBytesToRegister(cmd, offset, dataBytes, data_size);
    }

    void _readSystemEventStatusRegister() {
        _readBytesFromRegister(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
    }
    boolean _isClockProblem() {
        return (UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, CLKPLL_LL_BIT) ||
                UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RFPLL_LL_BIT));
    }
    boolean _isTransmitDone() {
        return UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, TXFRS_BIT);
    }
    boolean _isReceiveTimestampAvailable() {
        return UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT);
    }
    boolean _isReceiveFailed() {
        return (UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RXPHE_BIT) ||
                UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RXFCE_BIT) ||
                UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RXRFSL_BIT) ||
                UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, LDEERR_BIT));
    }
    boolean _isReceiveTimeout() {
        return (UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RXRFTO_BIT) ||
                UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RXPTO_BIT) ||
                UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RXSFDTO_BIT));
    }
    void _resetReceiver() {
        /* Set to 0 only bit 28 */
        _writeValueToRegister(PMSC, PMSC_SOFTRESET_SUB, 0xE0, LEN_PMSC_SOFTRESET);
        /* Set SOFTRESET to all ones */
        _writeValueToRegister(PMSC, PMSC_SOFTRESET_SUB, 0xF0, LEN_PMSC_SOFTRESET);
    }

    void _clearTransmitStatus() {
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, AAT_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, TXFRB_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, TXPRS_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, TXPHS_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, TXFRS_BIT, true);
        _writeBytesToRegister(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
    }
    void _clearReceiveFailedStatus() {
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXPHE_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXFCE_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXRFSL_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, AFFREJ_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, LDEERR_BIT, true);
        _writeBytesToRegister(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
    }
    void _clearReceiveTimestampAvailableStatus() {
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT, true);
        _writeBytesToRegister(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
    }

    void forceTRxOff() {
        memset(_sysctrl, 0, LEN_SYS_CTRL);
        UWBUtil::setBit(_sysctrl, LEN_SYS_CTRL, TRXOFF_BIT, true);
        _writeBytesToRegister(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
    }

#if defined(ESP8266)
    void ICACHE_RAM_ATTR interruptServiceRoutine() {
#else
    void interruptServiceRoutine() {
#endif		// read current status and handle via callbacks
        _readSystemEventStatusRegister();
        if(_isClockProblem() /* TODO and others */ && _handleError != 0) {
            (*_handleError)();
        }
        if(_isTransmitDone()) {
            _clearTransmitStatus();
            if(_handleSent != nullptr)
                (*_handleSent)();
        }
        if(_isReceiveTimestampAvailable()) {
            _clearReceiveTimestampAvailableStatus();
            if(_handleReceiveTimestampAvailable != nullptr)
                (*_handleReceiveTimestampAvailable)();
        }
        if(_isReceiveFailed()) {
            _clearReceiveFailedStatus();
            forceTRxOff();
            _resetReceiver();
            if(_handleReceiveFailed != nullptr)
                (*_handleReceiveFailed)();
        } else if(_isReceiveTimeout()) {
            _clearReceiveTimeoutStatus();
            forceTRxOff();
            _resetReceiver();
            if(_handleReceiveTimeout != nullptr)
                (*_handleReceiveTimeout)();
        } else if(_isReceiveDone()) {
            _clearReceiveStatus();
            if(_handleReceived != nullptr)
                (*_handleReceived)();
        }
    }

    void initialize(uint8_t ss, uint8_t irq, uint8_t rst, SPIClass&spi) {
        // generous initial init/wake-up-idle delay
        delay(5);
        _ss = ss;
        _irq = irq;
        _rst = rst;

        if(rst != 0xff) {
            // DW1000 data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
            pinMode(_rst, INPUT);
        }

        UWBPorting::SPIinit(spi);
        // pin and basic member setup
        // attach interrupt
        // TODO throw error if pin is not a interrupt pin
        if(_irq != 0xff)
            attachInterrupt(digitalPinToInterrupt(_irq), interruptServiceRoutine, RISING);
        UWBPorting::SPIselect(_ss, _irq);
        // reset chip (either soft or hard)
        reset();

        UWBPorting::setSPIspeed(SPIClock::SLOW);
        _enableClock(SYS_XTI_CLOCK);
        delay(5);

        // Configure the CPLL lock detect
        _writeBitToRegister(EXT_SYNC, EC_CTRL_SUB, LEN_EC_CTRL, PLLLDT_BIT, true);

        // Configure XTAL trim
        _fsxtalt();

        // load LDE micro-code
        _manageLDE();

        // read the temp and vbat readings from OTP that were recorded during production test
        // see 6.3.1 OTP memory map
        byte buf_otp[4];
        _readBytesOTP(0x008, buf_otp); // the stored 3.3 V reading
        _vmeas3v3 = buf_otp[0];
        _readBytesOTP(0x009, buf_otp); // the stored 23C reading
        _tmeas23C = buf_otp[0];

        _enableClock(SYS_AUTO_CLOCK);
        delay(5);
        UWBPorting::setSPIspeed(SPIClock::FAST);

        _readNetworkIdAndDeviceAddress();
        _readSystemConfigurationRegister();
        _readChannelControlRegister();
        _readTransmitFrameControlRegister();
        _readSystemEventMaskRegister();

        /* Cleared AON:CFG1(0x2C:0x0A) for proper operation of deepSleep */
        _writeValueToRegister(AON, AON_CFG1_SUB, 0x00, LEN_AON_CFG1);

    }
}