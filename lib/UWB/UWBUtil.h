//
// Created by Ali Erdem Sunar on 30.11.2022.
//

#ifndef MINERPATH_ANCHOR_UWBUTIL_H
#define MINERPATH_ANCHOR_UWBUTIL_H

namespace UWBUtil {
    /**
    Returns target bit value inside a byte array
    @param [in] data the byte array
    @param [in] n the length of the byte array
    @param [in] bit the bit position
    returns bit value (true = 1, false = 0)
    */
    boolean getBit(byte data[], uint16_t n, uint16_t bit);

    /**
    Sets the target bit value inside an array of bytes
    @param [in] data the byte array
    @param [in] n the length of the byte array
    @param [in] bit the bit position
    @param [in] val the bit value
    */
    void setBit(byte data[], uint16_t n, uint16_t bit, boolean val);

    /**
    Writes the target value inside a given byte array.
    @param [in] data the byte array
    @param [in] val the value to insert
    @param [in] n the length of the byte array
    */
    void writeValueToBytes(byte data[], uint64_t val, uint8_t n);

    /**
    Gets the target byte array value
    @param [in] data the byte array
    @param [in] n the length of the byte array
    returns the byte array value
    */
    uint64_t bytesAsValue(byte data[], uint8_t n);

    /**
    Converts from char to 4 bits (hexadecimal)
    @param [in] c the character
    returns target value
    */

    uint8_t nibbleFromChar(char c);

    /**
    Converts the target string to eui bytes
    @param [in] string The eui string (in format XX:XX:XX:XX:XX:XX:XX:XX)
    @param [out] eui_byte The eui bytes
    */
    void convertToByte(char string[], byte *eui_byte);
}

#endif //MINERPATH_ANCHOR_UWBUTIL_H
