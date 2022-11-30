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
        if (offset == NO_SUB) {
            header[0] = READ | cmd;
        } else {
            header[0] = READ_SUB | cmd;
            if (offset < 128) {
                header[1] = (byte) offset;
                headerLen++;
            } else {
                header[1] = RW_SUB_EXT | (byte) offset;
                header[2] = (byte) (offset >> 7);
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
        if (offset == NO_SUB) {
            header[0] = WRITE | cmd;
        } else {
            header[0] = WRITE_SUB | cmd;
            if (offset < 128) {
                header[1] = (byte) offset;
                headerLen++;
            } else {
                header[1] = RW_SUB_EXT | (byte) offset;
                header[2] = (byte) (offset >> 7);
                headerLen += 2;
            }
        }

        UWBPorting::writeToSPI(_ss, headerLen, header, data_size, data);
    }

    void _writeSingleByteToRegister(byte cmd, uint16_t offset, byte data) {
        _writeBytesToRegister(cmd, offset, &data, 1); // 1 as data_size because writes a single byte
    }

    void _writeBitToRegister(byte bitRegister, uint16_t RegisterOffset, uint16_t bitRegister_LEN, uint16_t selectedBit,
                             boolean value) {
        uint16_t idx;
        uint8_t bitPosition;

        idx = selectedBit / 8;
        if (idx >= bitRegister_LEN) {
            return; // TODO proper error handling: out of bounds
        }
        byte targetByte;
        memset(&targetByte, 0, 1);
        bitPosition = selectedBit % 8;
        _readBytesFromRegister(bitRegister, RegisterOffset + idx, &targetByte, 1);

        value ? bitSet(targetByte, bitPosition) : bitClear(targetByte, bitPosition);

        if (RegisterOffset == NO_SUB)
            RegisterOffset = 0x00;

        _writeBytesToRegister(bitRegister, RegisterOffset + idx, &targetByte, 1);
    }

    void _writeValueToRegister(byte cmd, uint16_t offset, uint32_t data, uint16_t data_size) {
        byte dataBytes[data_size];
        UWBUtil::writeValueToBytes(dataBytes, data, data_size);
        _writeBytesToRegister(cmd, offset, dataBytes, data_size);
    }

    void _writeNetworkIdAndDeviceAddress() {
        _writeBytesToRegister(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
    }

    void _writeSystemConfigurationRegister() {
        _writeBytesToRegister(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
    }


    void _readSystemEventStatusRegister() {
        _readBytesFromRegister(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
    }

    void _readSystemConfigurationRegister() {
        _readBytesFromRegister(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
    }

    void _readNetworkIdAndDeviceAddress() {
        _readBytesFromRegister(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
    }

    void _readSystemEventMaskRegister() {
        _readBytesFromRegister(SYS_MASK, NO_SUB, _sysmask, LEN_SYS_MASK);
    }

    void _readChannelControlRegister() {
        _readBytesFromRegister(CHAN_CTRL, NO_SUB, _chanctrl, LEN_CHAN_CTRL);
    }

    void _readTransmitFrameControlRegister() {
        _readBytesFromRegister(TX_FCTRL, NO_SUB, _txfctrl, LEN_TX_FCTRL);
    }

    void _readBytesOTP(uint16_t address, byte data[]) {
        byte addressBytes[LEN_OTP_ADDR];

        // p60 - 6.3.3 Reading a value from OTP memory
        // bytes of address
        addressBytes[0] = (address & 0xFF);
        addressBytes[1] = ((address >> 8) & 0xFF);
        // set address
        _writeBytesToRegister(OTP_IF, OTP_ADDR_SUB, addressBytes, LEN_OTP_ADDR);
        // switch into read mode
        _writeSingleByteToRegister(OTP_IF, OTP_CTRL_SUB, 0x03); // OTPRDEN | OTPREAD
        _writeSingleByteToRegister(OTP_IF, OTP_CTRL_SUB, 0x01); // OTPRDEN
        // read value/block - 4 bytes
        _readBytesFromRegister(OTP_IF, OTP_RDAT_SUB, data, LEN_OTP_RDAT);
        // end read mode
        _writeSingleByteToRegister(OTP_IF, OTP_CTRL_SUB, 0x00);
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

    boolean _isReceiveDone() {
        if (_frameCheck) {
            return (UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RXFCG_BIT) &&
                    UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RXDFR_BIT));
        }
        return UWBUtil::getBit(_sysstatus, LEN_SYS_STATUS, RXDFR_BIT);
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

    void _clearReceiveTimeoutStatus() {
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXRFTO_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXPTO_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXSFDTO_BIT, true);
        _writeBytesToRegister(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
    }

    void _clearReceiveStatus() {
        // clear latched RX bits (i.e. write 1 to clear)
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXDFR_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXFCG_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXPRD_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXSFDD_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, RXPHD_BIT, true);
        UWBUtil::setBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT, true);
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

    void _enableClock(byte clock) {
        byte pmscctrl0[LEN_PMSC_CTRL0];
        memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
        _readBytesFromRegister(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
        if (clock == SYS_AUTO_CLOCK) {
            pmscctrl0[0] = SYS_AUTO_CLOCK;
            pmscctrl0[1] &= 0xFE;
        } else if (clock == SYS_XTI_CLOCK) {
            pmscctrl0[0] &= 0xFC;
            pmscctrl0[0] |= SYS_XTI_CLOCK;
        } else if (clock == SYS_PLL_CLOCK) {
            pmscctrl0[0] &= 0xFC;
            pmscctrl0[0] |= SYS_PLL_CLOCK;
        } else if (clock == TX_PLL_CLOCK) {
            pmscctrl0[0] &= 0xCF;
            pmscctrl0[0] |= TX_PLL_CLOCK;
        } else if (clock == LDE_CLOCK) {
            pmscctrl0[0] = SYS_XTI_CLOCK;
            pmscctrl0[1] = 0x03;
        } else {
            // TODO deliver proper warning
        }
        _writeBytesToRegister(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
    }

    void _disableSequencing() {
        _enableClock(SYS_XTI_CLOCK);
        byte zero[2];
        UWBUtil::writeValueToBytes(zero, 0x0000, 2);
        _writeBytesToRegister(PMSC, PMSC_CTRL1_SUB, zero, 2); // To re-enable write 0xE7
    }

    void getPrintableDeviceIdentifier(char msgBuffer[]) {
        byte data[LEN_DEV_ID];
        _readBytesFromRegister(DEV_ID, NO_SUB, data, LEN_DEV_ID);
        sprintf(msgBuffer, "%02X - model: %d, version: %d, revision: %d",
                (uint16_t) ((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F, data[0] & 0x0F);
    }

    void getPrintableExtendedUniqueIdentifier(char msgBuffer[]) {
        byte data[LEN_EUI];
        _readBytesFromRegister(EUI, NO_SUB, data, LEN_EUI);
        sprintf(msgBuffer, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0]);
    }
    void getPrintableNetworkIdAndShortAddress(char msgBuffer[]) {
        byte data[LEN_PANADR];
        _readBytesFromRegister(PANADR, NO_SUB, data, LEN_PANADR);
        sprintf(msgBuffer, "PAN: %02X, Short Address: %02X",
                (uint16_t)((data[3] << 8) | data[2]), (uint16_t)((data[1] << 8) | data[0]));
    }

#if defined(ESP8266)
    void ICACHE_RAM_ATTR interruptServiceRoutine() {
#else

    void interruptServiceRoutine() {
#endif        // read current status and handle via callbacks
        _readSystemEventStatusRegister();
        if (_isClockProblem() /* TODO and others */ && _handleError != 0) {
            (*_handleError)();
        }
        if (_isTransmitDone()) {
            _clearTransmitStatus();
            if (_handleSent != nullptr)
                (*_handleSent)();
        }
        if (_isReceiveTimestampAvailable()) {
            _clearReceiveTimestampAvailableStatus();
            if (_handleReceiveTimestampAvailable != nullptr)
                (*_handleReceiveTimestampAvailable)();
        }
        if (_isReceiveFailed()) {
            _clearReceiveFailedStatus();
            forceTRxOff();
            _resetReceiver();
            if (_handleReceiveFailed != nullptr)
                (*_handleReceiveFailed)();
        } else if (_isReceiveTimeout()) {
            _clearReceiveTimeoutStatus();
            forceTRxOff();
            _resetReceiver();
            if (_handleReceiveTimeout != nullptr)
                (*_handleReceiveTimeout)();
        } else if (_isReceiveDone()) {
            _clearReceiveStatus();
            if (_handleReceived != nullptr)
                (*_handleReceived)();
        }
    }

    void _manageLDE() {
        // transfer any ldo tune values
        byte ldoTune[LEN_OTP_RDAT];
        uint16_t LDOTUNE_ADDRESS = 0x04;
        _readBytesOTP(LDOTUNE_ADDRESS, ldoTune); // TODO #define
        if (ldoTune[0] != 0) {
            // TODO tuning available, copy over to RAM: use OTP_LDO bit
        }
        // tell the chip to load the LDE microcode
        // TODO remove clock-related code (PMSC_CTRL) as handled separately
        byte pmscctrl0[LEN_PMSC_CTRL0];
        byte otpctrl[LEN_OTP_CTRL];
        memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
        memset(otpctrl, 0, LEN_OTP_CTRL);
        _readBytesFromRegister(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
        _readBytesFromRegister(OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
        pmscctrl0[0] = 0x01;
        pmscctrl0[1] = 0x03;
        otpctrl[0] = 0x00;
        otpctrl[1] = 0x80;
        _writeBytesToRegister(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
        // uCode
        _enableClock(LDE_CLOCK);
        delay(5);
        _writeBytesToRegister(OTP_IF, OTP_CTRL_SUB, otpctrl, 2);
        delay(1);
        _enableClock(SYS_AUTO_CLOCK);
        delay(5);
        pmscctrl0[0] = 0x00;
        pmscctrl0[1] &= 0x02;
        _writeBytesToRegister(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
    }

    void reset() {
        if (_rst == 0xff) { /* Fallback to Software Reset */
            softwareReset();
        } else {
            // DW1000Ng data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
            pinMode(_rst, OUTPUT);
            digitalWrite(_rst, LOW);
            delay(2);  // DW1000Ng data sheet v2.08 §5.6.1 page 20: nominal 50ns, to be safe take more time
            pinMode(_rst, INPUT);
            delay(5); // dw1000Ng data sheet v1.2 page 5: nominal 3 ms, to be safe take more time
        }
    }

    void softwareReset() {
        UWBPorting::setSPIspeed(SPIClock::SLOW);

        /* Disable sequencing and go to state "INIT" - (a) Sets SYSCLKS to 01 */
        _disableSequencing();
        /* Clear AON and WakeUp configuration */
        _writeValueToRegister(AON, AON_WCFG_SUB, 0x00, LEN_AON_WCFG);
        _writeValueToRegister(AON, AON_CFG0_SUB, 0x00, LEN_AON_CFG0);
        // TODO change this with uploadToAON
        _writeValueToRegister(AON, AON_CTRL_SUB, 0x00, LEN_AON_CTRL);
        _writeValueToRegister(AON, AON_CTRL_SUB, 0x02, LEN_AON_CTRL);
        /* (b) Clear SOFTRESET to all zero’s */
        _writeValueToRegister(PMSC, PMSC_SOFTRESET_SUB, 0x00, LEN_PMSC_SOFTRESET);
        delay(1);
        /* (c) Set SOFTRESET to all ones */
        _writeValueToRegister(PMSC, PMSC_SOFTRESET_SUB, 0xF0, LEN_PMSC_SOFTRESET);
    }

    void _fsxtalt() {
        byte fsxtalt[LEN_FS_XTALT];
        byte buf_otp[4];
        _readBytesOTP(0x01E, buf_otp); //0x01E -> byte[0]=XTAL_Trim
        if (buf_otp[0] == 0) {
            // No trim value available from OTP, use midrange value of 0x10
            UWBUtil::writeValueToBytes(fsxtalt, ((0x10 & 0x1F) | 0x60), LEN_FS_XTALT);
        } else {
            UWBUtil::writeValueToBytes(fsxtalt, ((buf_otp[0] & 0x1F) | 0x60), LEN_FS_XTALT);
        }
        // write configuration back to chip
        _writeBytesToRegister(FS_CTRL, FS_XTALT_SUB, fsxtalt, LEN_FS_XTALT);
    }

    void initialize(uint8_t ss, uint8_t irq, uint8_t rst, SPIClass &spi) {
        // generous initial init/wake-up-idle delay
        delay(5);
        _ss = ss;
        _irq = irq;
        _rst = rst;

        if (rst != 0xff) {
            // DW1000 data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
            pinMode(_rst, INPUT);
        }

        UWBPorting::SPIinit(spi);
        // pin and basic member setup
        // attach interrupt
        // TODO throw error if pin is not a interrupt pin
        if (_irq != 0xff)
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

    void setDeviceAddress(uint16_t val) {
        _networkAndAddress[0] = (byte) (val & 0xFF);
        _networkAndAddress[1] = (byte) ((val >> 8) & 0xFF);
        _writeNetworkIdAndDeviceAddress();
    }

    void setNetworkId(uint16_t val) {
        _networkAndAddress[2] = (byte) (val & 0xFF);
        _networkAndAddress[3] = (byte) ((val >> 8) & 0xFF);
        _writeNetworkIdAndDeviceAddress();
    }

    void setEUI(char eui[]) {
        byte eui_byte[LEN_EUI];
        UWBUtil::convertToByte(eui, eui_byte);
        setEUI(eui_byte);
    }

    void setEUI(byte eui[]) {
        //we reverse the address->
        char msgBuffer[128];
        sprintf(msgBuffer, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                eui[7], eui[6], eui[5], eui[4], eui[3], eui[2], eui[1], eui[0]);
        byte reverseEUI[8];
        uint8_t size = 8;
        for (uint8_t i = 0; i < size; i++) {
            *(reverseEUI + i) = *(eui + size - i - 1);
        }
        _writeBytesToRegister(EUI, NO_SUB, reverseEUI, LEN_EUI);
    }

    void _useExtendedFrameLength(boolean val) {
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, PHR_MODE_0_BIT, val);
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, PHR_MODE_1_BIT, val);
    }

    void _setReceiverAutoReenable(boolean val) {
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, RXAUTR_BIT, val);
    }

    void _useFrameCheck(boolean val) {
        _frameCheck = val;
    }
    void _agctune1() {
        byte agctune1[LEN_AGC_TUNE1];
        if(_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
            UWBUtil::writeValueToBytes(agctune1, 0x8870, LEN_AGC_TUNE1);
        } else if(_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
            UWBUtil::writeValueToBytes(agctune1, 0x889B, LEN_AGC_TUNE1);
        } else {
            // TODO proper error/warning handling
        }
        _writeBytesToRegister(AGC_TUNE, AGC_TUNE1_SUB, agctune1, LEN_AGC_TUNE1);
    }

    /* AGC_TUNE2 - reg:0x23, sub-reg:0x0C, table 25 */
    void _agctune2() {
        byte agctune2[LEN_AGC_TUNE2];
        UWBUtil::writeValueToBytes(agctune2, 0x2502A907L, LEN_AGC_TUNE2);
        _writeBytesToRegister(AGC_TUNE, AGC_TUNE2_SUB, agctune2, LEN_AGC_TUNE2);
    }

    /* AGC_TUNE3 - reg:0x23, sub-reg:0x12, table 26 */
    void _agctune3() {
        byte agctune3[LEN_AGC_TUNE3];
        UWBUtil::writeValueToBytes(agctune3, 0x0035, LEN_AGC_TUNE3);
        _writeBytesToRegister(AGC_TUNE, AGC_TUNE3_SUB, agctune3, LEN_AGC_TUNE3);
    }

    /* DRX_TUNE0b - reg:0x27, sub-reg:0x02, table 30 */
    void _drxtune0b() {
        byte drxtune0b[LEN_DRX_TUNE0b];
        if(_dataRate == DataRate::RATE_110KBPS) {
            if(!_standardSFD) {
                UWBUtil::writeValueToBytes(drxtune0b, 0x0016, LEN_DRX_TUNE0b);
            } else {
                UWBUtil::writeValueToBytes(drxtune0b, 0x000A, LEN_DRX_TUNE0b);
            }
        } else if(_dataRate == DataRate::RATE_850KBPS) {
            if(!_standardSFD) {
                UWBUtil::writeValueToBytes(drxtune0b, 0x0006, LEN_DRX_TUNE0b);
            } else {
                UWBUtil::writeValueToBytes(drxtune0b, 0x0001, LEN_DRX_TUNE0b);
            }
        } else if(_dataRate == DataRate::RATE_6800KBPS) {
            if(!_standardSFD) {
                UWBUtil::writeValueToBytes(drxtune0b, 0x0002, LEN_DRX_TUNE0b);
            } else {
                UWBUtil::writeValueToBytes(drxtune0b, 0x0001, LEN_DRX_TUNE0b);
            }
        } else {
            // TODO proper error/warning handling
        }
        _writeBytesToRegister(DRX_TUNE, DRX_TUNE0b_SUB, drxtune0b, LEN_DRX_TUNE0b);
    }

    /* DRX_TUNE1a - reg:0x27, sub-reg:0x04, table 31 */
    void _drxtune1a() {
        byte drxtune1a[LEN_DRX_TUNE1a];
        if(_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
            UWBUtil::writeValueToBytes(drxtune1a, 0x0087, LEN_DRX_TUNE1a);
        } else if(_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
            UWBUtil::writeValueToBytes(drxtune1a, 0x008D, LEN_DRX_TUNE1a);
        } else {
            // TODO proper error/warning handling
        }
        _writeBytesToRegister(DRX_TUNE, DRX_TUNE1a_SUB, drxtune1a, LEN_DRX_TUNE1a);
    }

    /* DRX_TUNE1b - reg:0x27, sub-reg:0x06, table 32 */
    void _drxtune1b() {
        byte drxtune1b[LEN_DRX_TUNE1b];
        if(_preambleLength == PreambleLength::LEN_1536 || _preambleLength == PreambleLength::LEN_2048 ||
           _preambleLength == PreambleLength::LEN_4096) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(drxtune1b, 0x0064, LEN_DRX_TUNE1b);
            } else {
                // TODO proper error/warning handling
            }
        } else if(_preambleLength != PreambleLength::LEN_64) {
            if(_dataRate == DataRate::RATE_850KBPS || _dataRate == DataRate::RATE_6800KBPS) {
                UWBUtil::writeValueToBytes(drxtune1b, 0x0020, LEN_DRX_TUNE1b);
            } else {
                // TODO proper error/warning handling
            }
        } else {
            if(_dataRate == DataRate::RATE_6800KBPS) {
                UWBUtil::writeValueToBytes(drxtune1b, 0x0010, LEN_DRX_TUNE1b);
            } else {
                // TODO proper error/warning handling
            }
        }
        _writeBytesToRegister(DRX_TUNE, DRX_TUNE1b_SUB, drxtune1b, LEN_DRX_TUNE1b);
    }

    /* DRX_TUNE2 - reg:0x27, sub-reg:0x08, table 33 */
    void _drxtune2() {
        byte drxtune2[LEN_DRX_TUNE2];
        if(_pacSize == PacSize::SIZE_8) {
            if(_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
                UWBUtil::writeValueToBytes(drxtune2, 0x311A002DL, LEN_DRX_TUNE2);
            } else if(_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
                UWBUtil::writeValueToBytes(drxtune2, 0x313B006BL, LEN_DRX_TUNE2);
            } else {
                // TODO proper error/warning handling
            }
        } else if(_pacSize == PacSize::SIZE_16) {
            if(_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
                UWBUtil::writeValueToBytes(drxtune2, 0x331A0052L, LEN_DRX_TUNE2);
            } else if(_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
                UWBUtil::writeValueToBytes(drxtune2, 0x333B00BEL, LEN_DRX_TUNE2);
            } else {
                // TODO proper error/warning handling
            }
        } else if(_pacSize == PacSize::SIZE_32) {
            if(_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
                UWBUtil::writeValueToBytes(drxtune2, 0x351A009AL, LEN_DRX_TUNE2);
            } else if(_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
                UWBUtil::writeValueToBytes(drxtune2, 0x353B015EL, LEN_DRX_TUNE2);
            } else {
                // TODO proper error/warning handling
            }
        } else if(_pacSize == PacSize::SIZE_64) {
            if(_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
                UWBUtil::writeValueToBytes(drxtune2, 0x371A011DL, LEN_DRX_TUNE2);
            } else if(_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
                UWBUtil::writeValueToBytes(drxtune2, 0x373B0296L, LEN_DRX_TUNE2);
            } else {
                // TODO proper error/warning handling
            }
        } else {
            // TODO proper error/warning handling
        }
        _writeBytesToRegister(DRX_TUNE, DRX_TUNE2_SUB, drxtune2, LEN_DRX_TUNE2);
    }

    /* DRX_TUNE4H - reg:0x27, sub-reg:0x26, table 34 */
    void _drxtune4H() {
        byte drxtune4H[LEN_DRX_TUNE4H];
        if(_preambleLength == PreambleLength::LEN_64) {
            UWBUtil::writeValueToBytes(drxtune4H, 0x0010, LEN_DRX_TUNE4H);
        } else {
            UWBUtil::writeValueToBytes(drxtune4H, 0x0028, LEN_DRX_TUNE4H);
        }
        _writeBytesToRegister(DRX_TUNE, DRX_TUNE4H_SUB, drxtune4H, LEN_DRX_TUNE4H);
    }

    void _ldecfg1() {
        byte ldecfg1[LEN_LDE_CFG1];
        _nlos == true ? UWBUtil::writeValueToBytes(ldecfg1, 0x7, LEN_LDE_CFG1) : UWBUtil::writeValueToBytes(ldecfg1,
                                                                                                            0xD,
                                                                                                            LEN_LDE_CFG1);
        _writeBytesToRegister(LDE_IF, LDE_CFG1_SUB, ldecfg1, LEN_LDE_CFG1);
    }

    /* LDE_CFG2 - reg 0x2E, sub-reg:0x1806, table 50 */
    void _ldecfg2() {
        byte ldecfg2[LEN_LDE_CFG2];
        if (_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
            _nlos == true ? UWBUtil::writeValueToBytes(ldecfg2, 0x0003, LEN_LDE_CFG2) : UWBUtil::writeValueToBytes(
                    ldecfg2, 0x1607, LEN_LDE_CFG2);
        } else if (_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
            UWBUtil::writeValueToBytes(ldecfg2, 0x0607, LEN_LDE_CFG2);
        } else {
            // TODO proper error/warning handling
        }
        _writeBytesToRegister(LDE_IF, LDE_CFG2_SUB, ldecfg2, LEN_LDE_CFG2);
    }

    void _lderepc() {
        byte lderepc[LEN_LDE_REPC];
        if(_preambleCode == PreambleCode::CODE_1 || _preambleCode == PreambleCode::CODE_2) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x5998, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_3 || _preambleCode == PreambleCode::CODE_8) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x51EA, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_4) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x428E, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_5) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x451E, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_6) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x2E14, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_7) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x8000, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_9) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x28F4, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_10 || _preambleCode == PreambleCode::CODE_17) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x3332, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_11) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x3AE0, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_12) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x3D70, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_18 || _preambleCode == PreambleCode::CODE_19) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x35C2, LEN_LDE_REPC);
            }
        } else if(_preambleCode == PreambleCode::CODE_20) {
            if(_dataRate == DataRate::RATE_110KBPS) {
                UWBUtil::writeValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LEN_LDE_REPC);
            } else {
                UWBUtil::writeValueToBytes(lderepc, 0x47AE, LEN_LDE_REPC);
            }
        } else {
            // TODO proper error/warning handling
        }

        _writeBytesToRegister(LDE_IF, LDE_REPC_SUB, lderepc, LEN_LDE_REPC);
    }

    void _rfrxctrlh() {
        byte rfrxctrlh[LEN_RF_RXCTRLH];
        if(_channel != Channel::CHANNEL_4 && _channel != Channel::CHANNEL_7) {
            UWBUtil::writeValueToBytes(rfrxctrlh, 0xD8, LEN_RF_RXCTRLH);
        } else {
            UWBUtil::writeValueToBytes(rfrxctrlh, 0xBC, LEN_RF_RXCTRLH);
        }
        _writeBytesToRegister(RF_CONF, RF_RXCTRLH_SUB, rfrxctrlh, LEN_RF_RXCTRLH);
    }

    /* RX_TXCTRL - reg:0x28, sub-reg:0x0C */
    void _rftxctrl() {
        byte rftxctrl[LEN_RF_TXCTRL];
        if(_channel == Channel::CHANNEL_1) {
            UWBUtil::writeValueToBytes(rftxctrl, 0x00005C40L, LEN_RF_TXCTRL);
        } else if(_channel == Channel::CHANNEL_2) {
            UWBUtil::writeValueToBytes(rftxctrl, 0x00045CA0L, LEN_RF_TXCTRL);
        } else if(_channel == Channel::CHANNEL_3) {
            UWBUtil::writeValueToBytes(rftxctrl, 0x00086CC0L, LEN_RF_TXCTRL);
        } else if(_channel == Channel::CHANNEL_4) {
            UWBUtil::writeValueToBytes(rftxctrl, 0x00045C80L, LEN_RF_TXCTRL);
        } else if(_channel == Channel::CHANNEL_5) {
            UWBUtil::writeValueToBytes(rftxctrl, 0x001E3FE0L, LEN_RF_TXCTRL);
        } else if(_channel == Channel::CHANNEL_7) {
            UWBUtil::writeValueToBytes(rftxctrl, 0x001E7DE0L, LEN_RF_TXCTRL);
        } else {
            // TODO proper error/warning handling
        }
        _writeBytesToRegister(RF_CONF, RF_TXCTRL_SUB, rftxctrl, LEN_RF_TXCTRL);
    }

    /* TC_PGDELAY - reg:0x2A, sub-reg:0x0B, table 40 */
    void _tcpgdelaytune() {
        byte tcpgdelay[LEN_TC_PGDELAY];
        if(_channel == Channel::CHANNEL_1) {
            UWBUtil::writeValueToBytes(tcpgdelay, 0xC9, LEN_TC_PGDELAY);
        } else if(_channel == Channel::CHANNEL_2) {
            UWBUtil::writeValueToBytes(tcpgdelay, 0xC2, LEN_TC_PGDELAY);
        } else if(_channel == Channel::CHANNEL_3) {
            UWBUtil::writeValueToBytes(tcpgdelay, 0xC5, LEN_TC_PGDELAY);
        } else if(_channel == Channel::CHANNEL_4) {
            UWBUtil::writeValueToBytes(tcpgdelay, 0x95, LEN_TC_PGDELAY);
        } else if(_channel == Channel::CHANNEL_5) {
            UWBUtil::writeValueToBytes(tcpgdelay, 0xB5, LEN_TC_PGDELAY);
        } else if(_channel == Channel::CHANNEL_7) {
            UWBUtil::writeValueToBytes(tcpgdelay, 0x93, LEN_TC_PGDELAY);
        } else {
            // TODO proper error/warning handling
        }
        _writeBytesToRegister(TX_CAL, TC_PGDELAY_SUB, tcpgdelay, LEN_TC_PGDELAY);
    }

    // FS_PLLCFG and FS_PLLTUNE - reg:0x2B, sub-reg:0x07-0x0B, tables 43-44
    void _fspll() {
        byte fspllcfg[LEN_FS_PLLCFG];
        byte fsplltune[LEN_FS_PLLTUNE];
        if(_channel == Channel::CHANNEL_1) {
            UWBUtil::writeValueToBytes(fspllcfg, 0x09000407L, LEN_FS_PLLCFG);
            UWBUtil::writeValueToBytes(fsplltune, 0x1E, LEN_FS_PLLTUNE);
        } else if(_channel == Channel::CHANNEL_2 || _channel == Channel::CHANNEL_4) {
            UWBUtil::writeValueToBytes(fspllcfg, 0x08400508L, LEN_FS_PLLCFG);
            UWBUtil::writeValueToBytes(fsplltune, 0x26, LEN_FS_PLLTUNE);
        } else if(_channel == Channel::CHANNEL_3) {
            UWBUtil::writeValueToBytes(fspllcfg, 0x08401009L, LEN_FS_PLLCFG);
            UWBUtil::writeValueToBytes(fsplltune, 0x56, LEN_FS_PLLTUNE);
        } else if(_channel == Channel::CHANNEL_5 || _channel == Channel::CHANNEL_7) {
            UWBUtil::writeValueToBytes(fspllcfg, 0x0800041DL, LEN_FS_PLLCFG);
            UWBUtil::writeValueToBytes(fsplltune, 0xBE, LEN_FS_PLLTUNE);
        } else {
            // TODO proper error/warning handling
        }
        _writeBytesToRegister(FS_CTRL, FS_PLLTUNE_SUB, fsplltune, LEN_FS_PLLTUNE);
        _writeBytesToRegister(FS_CTRL, FS_PLLCFG_SUB, fspllcfg, LEN_FS_PLLCFG);
    }


    void _setNlosOptimization(boolean val) {
        _nlos = val;
        if (_nlos) {
            _ldecfg1();
            _ldecfg2();
        }
    }

    void _txpowertune() {
        byte txpower[LEN_TX_POWER];
        if (_channel == Channel::CHANNEL_1 || _channel == Channel::CHANNEL_2) {
            if (_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x1B153555L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x15355575L, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x55555555L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x75757575L, LEN_TX_POWER);
#endif
                }
            } else if (_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x0D072747L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x07274767L, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x47474747L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x67676767L, LEN_TX_POWER);
#endif
                }
            } else {
                // TODO proper error/warning handling
            }
        } else if (_channel == Channel::CHANNEL_3) {
            if (_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x150F2F4FL, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x0F2F4F6FL, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x4F4F4F4FL, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x6F6F6F6FL, LEN_TX_POWER);
#endif
                }
            } else if (_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x0B2B4B6BL, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x2B4B6B8BL, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x6B6B6B6BL, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x8B8B8B8BL, LEN_TX_POWER);
#endif
                }
            } else {
                // TODO proper error/warning handling
            }
        } else if (_channel == Channel::CHANNEL_4) {
            if (_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x1F1F1F3FL, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x1F1F3F5FL, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x3F3F3F3FL, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x5F5F5F5FL, LEN_TX_POWER);
#endif
                }
            } else if (_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x1A3A5A7AL, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x3A5A7A9AL, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x7A7A7A7AL, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x9A9A9A9AL, LEN_TX_POWER);
#endif
                }
            } else {
                // TODO proper error/warning handling
            }
        } else if (_channel == Channel::CHANNEL_5) {
            if (_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x140E0828L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x0E082848L, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x28282828L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x48484848L, LEN_TX_POWER);
#endif
                }
            } else if (_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x05254565L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x25456585L, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x65656565L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x85858585L, LEN_TX_POWER);
#endif
                }
            } else {
                // TODO proper error/warning handling
            }
        } else if (_channel == Channel::CHANNEL_7) {
            if (_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x12325272L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x32527292L, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x72727272L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x92929292L, LEN_TX_POWER);
#endif
                }
            } else if (_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
                if (_smartPower) {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0x315191B1L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0x5171B1D1L, LEN_TX_POWER);
#endif
                } else {
#if DWM1000_OPTIMIZED
                    UWBUtil::writeValueToBytes(txpower, 0xB1B1B1B1L, LEN_TX_POWER);
#else
                    UWBUtil::writeValueToBytes(txpower, 0xD1D1D1D1L, LEN_TX_POWER);
#endif
                }
            } else {
                // TODO proper error/warning handling
            }
        } else {
            // TODO proper error/warning handling
        }
        _writeBytesToRegister(TX_POWER, NO_SUB, txpower, LEN_TX_POWER);
    }

    void _useSmartPower(boolean smartPower) {
        _smartPower = smartPower;
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, DIS_STXP_BIT, !smartPower);
        _writeSystemConfigurationRegister();
        if (_autoTXPower)
            _txpowertune();
    }

    void _setSFDMode(SFDMode mode) {
        switch (mode) {
            case SFDMode::STANDARD_SFD:
                UWBUtil::setBit(_chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, false);
                UWBUtil::setBit(_chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, false);
                UWBUtil::setBit(_chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, false);
                _standardSFD = true;
                break;
            case SFDMode::DECAWAVE_SFD:
                UWBUtil::setBit(_chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, true);
                UWBUtil::setBit(_chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, true);
                UWBUtil::setBit(_chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, true);
                _standardSFD = false;
                break;
            default:
                return; //TODO Proper error handling
        }
    }

    void _setChannel(Channel channel) {
        byte chan = static_cast<byte>(channel);
        chan &= 0xF;
        _chanctrl[0] = ((chan | (chan << 4)) & 0xFF);

        _channel = channel;
    }

    void _setDataRate(DataRate data_rate) {
        byte rate = static_cast<byte>(data_rate);
        rate &= 0x03;
        _txfctrl[1] &= 0x83;
        _txfctrl[1] |= (byte) ((rate << 5) & 0xFF);
        // special 110kbps flag
        if (data_rate == DataRate::RATE_110KBPS) {
            UWBUtil::setBit(_syscfg, LEN_SYS_CFG, RXM110K_BIT, true);
        } else {
            UWBUtil::setBit(_syscfg, LEN_SYS_CFG, RXM110K_BIT, false);
        }
        _dataRate = data_rate;
    }

    void _setPulseFrequency(PulseFrequency frequency) {
        byte freq = static_cast<byte>(frequency);
        freq &= 0x03;
        _txfctrl[2] &= 0xFC;
        _txfctrl[2] |= (byte) (freq & 0xFF);
        _chanctrl[2] &= 0xF3;
        _chanctrl[2] |= (byte) ((freq << 2) & 0xFF);

        _pulseFrequency = frequency;
    }

    void _setPreambleLength(PreambleLength preamble_length) {
        byte prealen = static_cast<byte>(preamble_length);
        prealen &= 0x0F;
        _txfctrl[2] &= 0xC3;
        _txfctrl[2] |= (byte) ((prealen << 2) & 0xFF);

        switch (preamble_length) {
            case PreambleLength::LEN_64:
                _pacSize = PacSize::SIZE_8;
                break;
            case PreambleLength::LEN_128:
                _pacSize = PacSize::SIZE_8;
                break;
            case PreambleLength::LEN_256:
                _pacSize = PacSize::SIZE_16;
                break;
            case PreambleLength::LEN_512:
                _pacSize = PacSize::SIZE_16;
                break;
            case PreambleLength::LEN_1024:
                _pacSize = PacSize::SIZE_32;
                break;
            default:
                _pacSize = PacSize::SIZE_64; // In case of 1536, 2048 or 4096 preamble length.
        }

        _preambleLength = preamble_length;
    }

    void _setPreambleCode(PreambleCode preamble_code) {
        byte preacode = static_cast<byte>(preamble_code);
        preacode &= 0x1F;
        _chanctrl[2] &= 0x3F;
        _chanctrl[2] |= ((preacode << 6) & 0xFF);
        _chanctrl[3] = 0x00;
        _chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);

        _preambleCode = preamble_code;
    }

    boolean _checkPreambleCodeValidity() {
        byte preacode = static_cast<byte>(_preambleCode);
        if (_pulseFrequency == PulseFrequency::FREQ_16MHZ) {
            for (auto i = 0; i < 2; i++) {
                if (preacode == preamble_validity_matrix_PRF16[(int) _channel][i])
                    return true;
            }
            return false;
        } else if (_pulseFrequency == PulseFrequency::FREQ_64MHZ) {
            for (auto i = 0; i < 4; i++) {
                if (preacode == preamble_validity_matrix_PRF64[(int) _channel][i])
                    return true;
            }
            return false;
        } else {
            return false; //TODO Proper error handling
        }
    }

    void _setValidPreambleCode() {
        PreambleCode preamble_code;

        switch (_channel) {
            case Channel::CHANNEL_1:
                preamble_code =
                        _pulseFrequency == PulseFrequency::FREQ_16MHZ ? PreambleCode::CODE_2 : PreambleCode::CODE_10;
                break;
            case Channel::CHANNEL_3:
                preamble_code =
                        _pulseFrequency == PulseFrequency::FREQ_16MHZ ? PreambleCode::CODE_6 : PreambleCode::CODE_10;
                break;
            case Channel::CHANNEL_4:
            case Channel::CHANNEL_7:
                preamble_code =
                        _pulseFrequency == PulseFrequency::FREQ_16MHZ ? PreambleCode::CODE_8 : PreambleCode::CODE_18;
                break;
            case Channel::CHANNEL_2:
            case Channel::CHANNEL_5:
                preamble_code =
                        _pulseFrequency == PulseFrequency::FREQ_16MHZ ? PreambleCode::CODE_3 : PreambleCode::CODE_10;
                break;
            default:
                return; //TODO Proper Error Handling
        }
        byte preacode = static_cast<byte>(preamble_code);
        preacode &= 0x1F;
        _chanctrl[2] &= 0x3F;
        _chanctrl[2] |= ((preacode << 6) & 0xFF);
        _chanctrl[3] = 0x00;
        _chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);

        _preambleCode = preamble_code;
    }

    void _setNonStandardSFDLength() {
        switch (_dataRate) {
            case DataRate::RATE_6800KBPS:
                _writeSingleByteToRegister(USR_SFD, SFD_LENGTH_SUB, 0x08);
                break;
            case DataRate::RATE_850KBPS:
                _writeSingleByteToRegister(USR_SFD, SFD_LENGTH_SUB, 0x10);
                break;
            case DataRate::RATE_110KBPS:
                _writeSingleByteToRegister(USR_SFD, SFD_LENGTH_SUB, 0x40);
                break;
            default:
                return; //TODO Proper error handling
        }
    }
    void _writeConfiguration() {
        // write all configurations back to device
        _writeSystemConfigurationRegister();
        //_writeChannelControlRegister();
        //_writeTransmitFrameControlRegister();
    }
    void _tune() {
        // these registers are going to be tuned/configured
        _agctune1();
        _agctune2();
        _agctune3();
        _drxtune0b();
        _drxtune1a();
        _drxtune1b();
        _drxtune2();
        _drxtune4H();
        _ldecfg1();
        _ldecfg2();
        _lderepc();
        if(_autoTXPower) _txpowertune();
        _rfrxctrlh();
        _rftxctrl();
        if(_autoTCPGDelay) _tcpgdelaytune();
        _fspll();
    }


    void applyConfiguration(device_configuration_t config) {
        forceTRxOff();

        _useExtendedFrameLength(config.extendedFrameLength);
        _setReceiverAutoReenable(config.receiverAutoReenable);
        _useSmartPower(config.smartPower);
        _useFrameCheck(config.frameCheck);
        _setNlosOptimization(config.nlos);
        _setSFDMode(config.sfd);
        _setChannel(config.channel);
        _setDataRate(config.dataRate);
        _setPulseFrequency(config.pulseFreq);
        _setPreambleLength(config.preambleLen);
        _setPreambleCode(config.preaCode);

        if (!_checkPreambleCodeValidity())
            _setValidPreambleCode();

        if (!_standardSFD)
            _setNonStandardSFDLength();

        // writes configuration to registers
        _writeConfiguration();
        // tune according to configuration
        _tune();
    }
    void enableFrameFiltering(frame_filtering_configuration_t config) {
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, FFEN_BIT, true);
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, FFBC_BIT, config.behaveAsCoordinator);
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, FFAB_BIT, config.allowBeacon);
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, FFAD_BIT, config.allowData);
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, FFAA_BIT, config.allowAcknowledgement);
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, FFAM_BIT, config.allowMacCommand);
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, FFAR_BIT, config.allowAllReserved);
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, FFA4_BIT, config.allowReservedFour);
        UWBUtil::setBit(_syscfg, LEN_SYS_CFG, FFA5_BIT, config.allowReservedFive);

        _writeSystemConfigurationRegister();
    }

}