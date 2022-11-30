//
// Created by Ali Erdem Sunar on 30.11.2022.
//

#ifndef MINERPATH_ANCHOR_UWBPORTING_H
#define MINERPATH_ANCHOR_UWBPORTING_H


#include <Arduino.h>
#include "UWBConstant.h"
#include <SPI.h>

namespace UWBPorting{

    /**
	Initializes the SPI bus.
	*/
    void SPIinit(SPIClass &spi = SPI);

    /**
	Tells the driver library that no communication to a DW1000 will be required anymore.
	This basically just frees SPI and the previously used pins.
	*/
    void SPIend();

    /**
	(Re-)selects a specific DW1000 chip for communication. Used in case you switched SPI to another device.
	*/
    void SPIselect(uint8_t slaveSelectPIN, uint8_t irq = 0xff);

    /**
    Arduino function to write to the SPI.
    Takes two separate byte buffers for write header and write data
    @param [in] Header lenght
    @param [in] Header array built before
    @param [in] Data lenght
    @param [in] Data array
    */
    void writeToSPI(uint8_t slaveSelectPIN, uint8_t headerLen, byte header[], uint16_t dataLen, byte data[]);

    /**
    Arduino function to read from the SPI.
    Takes two separate byte buffers for write header and write data
    @param [in] Header lenght
    @param [in] Header array built before
    @param [in] Data lenght
    @param [out] Data array
    */
    void readFromSPI(uint8_t slaveSelectPIN, uint8_t headerLen, byte header[], uint16_t dataLen, byte data[]);

    /**
    Sets speed of SPI clock, fast or slow(20MHz or 2MHz)
    @param [in] SPIClock FAST or SLOW
    */
    void setSPIspeed(SPIClock speed);

}


#endif //MINERPATH_ANCHOR_UWBPORTING_H
