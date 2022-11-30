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

    void reset();
    void softwareReset();

    void setDeviceAddress(uint16_t val);
    void setNetworkId(uint16_t val);
    /**
	Sets the device Extended Unique Identifier.
	This is a long identifier of the device.
	@param[in] eui A string containing the eui in its normal notation using columns.
	*/
    void setEUI(char eui[]);

    /**
    Sets the device Extended Unique Identifier.
    This is a long identifier of the device.
    @param[in] eui The raw bytes of the eui.
    */
    void setEUI(byte eui[]);


    /* ##### Print device id, address, etc. ###################################### */
    /**
    Generates a String representation of the device identifier of the chip. That usually
    are the letters "DECA" plus the	version and revision numbers of the chip.
    @param[out] msgBuffer The String buffer to be filled with printable device information.
        Provide 128 bytes, this should be sufficient.
    */
    void getPrintableDeviceIdentifier(char msgBuffer[]);

    /**
    Generates a String representation of the extended unique identifier (EUI) of the chip.
    @param[out] msgBuffer The String buffer to be filled with printable device information.
        Provide 128 bytes, this should be sufficient.
    */
    void getPrintableExtendedUniqueIdentifier(char msgBuffer[]);
/**
	Generates a String representation of the short address and network identifier currently
	defined for the respective chip.
	@param[out] msgBuffer The String buffer to be filled with printable device information.
		Provide 128 bytes, this should be sufficient.
	*/
    void getPrintableNetworkIdAndShortAddress(char msgBuffer[]);





    /**
	Applies the target configuration to the DW1000
	@param [in] config the configuration to apply to the DW1000
	*/
    void applyConfiguration(device_configuration_t config);

    /**
	Enables the frame filtering functionality using the provided configuration.
	Messages must be formatted using 802.15.4-2011 format.
	@param [in] config frame filtering configuration
	*/
    void enableFrameFiltering(frame_filtering_configuration_t config);

};


#endif //MINERPATH_ANCHOR_UWBMAIN_H
