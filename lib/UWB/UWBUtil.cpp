//
// Created by Ali Erdem Sunar on 30.11.2022.
//

#include <Arduino.h>
#include "UWBUtil.h"
#include "UWBConstant.h"
#include "UWBRegister.h"

namespace UWBUtil {

    /*
    * Set the value of a bit in an array of bytes that are considered
    * consecutive and stored from MSB to LSB.
    * @param data
    * 		The number as byte array.
    * @param n
    * 		The number of bytes in the array.
    * @param bit
    * 		The position of the bit to be set.
    * @param val
    *		The boolean value to be set to the given bit position.
    */
    void setBit(byte data[], uint16_t n, uint16_t bit, boolean val) {
        uint16_t idx;
        uint8_t shift;

        idx = bit/8;
        if(idx >= n) {
            return; // TODO proper error handling: out of bounds
        }
        byte* targetByte = &data[idx];
        shift = bit%8;
        if(val) {
            bitSet(*targetByte, shift);
        } else {
            bitClear(*targetByte, shift);
        }
    }

    /*
    * Check the value of a bit in an array of bytes that are considered
    * consecutive and stored from MSB to LSB.
    * @param data
    * 		The number as byte array.
    * @param n
    * 		The number of bytes in the array.
    * @param bit
    * 		The position of the bit to be checked.
    */
    boolean getBit(byte data[], uint16_t n, uint16_t bit) {
        uint16_t idx;
        uint8_t  shift;

        idx = bit/8;
        if(idx >= n) {
            return false; // TODO proper error handling: out of bounds
        }
        byte targetByte = data[idx];
        shift = bit%8;

        return bitRead(targetByte, shift); // TODO wrong type returned byte instead of boolean
    }

    void writeValueToBytes(byte data[], uint64_t val, uint8_t n) {
        for(auto i = 0; i < n; i++) {
            data[i] = ((val >> (i*8)) & 0xFF);
        }
    }

    uint64_t bytesAsValue(byte data[], uint8_t n) {
        uint64_t value = 0;
        for(auto i = 0; i < n; i++) {
            value |= ((uint64_t)data[i] << (i*8));
        }
        return value;
    }

    uint8_t nibbleFromChar(char c) {
        if(c >= '0' && c <= '9') {
            return c-'0';
        }
        if(c >= 'a' && c <= 'f') {
            return c-'a'+10;
        }
        if(c >= 'A' && c <= 'F') {
            return c-'A'+10;
        }
        return 255;
    }

    void convertToByte(char string[], byte* bytes) {
        byte eui_byte[LEN_EUI];
        // we fill it with the char array under the form of "AA:FF:1C:...."
        for(uint16_t i = 0; i < LEN_EUI; i++) {
            eui_byte[i] = (nibbleFromChar(string[i*3]) << 4)+nibbleFromChar(string[i*3+1]);
        }
        memcpy(bytes, eui_byte, LEN_EUI);
    }

}
