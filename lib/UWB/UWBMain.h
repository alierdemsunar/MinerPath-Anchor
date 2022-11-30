//
// Created by Ali Erdem Sunar on 30.11.2022.
//

#ifndef MINERPATH_ANCHOR_UWBMAIN_H
#define MINERPATH_ANCHOR_UWBMAIN_H

#include <stdlib.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "UWBConstant.h"
#include "UWBConfiguration.h"
//compile options #include "DW1000NgCompileOptions.hpp"

namespace UWBMain {
    void interruptServiceRoutine();
    void initialize(uint8_t ss, uint8_t irq, uint8_t rst = 0xff, SPIClass&spi = SPI);
    /**
	Stops the transceiver immediately, this actually sets the device in Idle mode.
	*/
    void forceTRxOff();
};


#endif //MINERPATH_ANCHOR_UWBMAIN_H
