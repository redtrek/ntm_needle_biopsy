
/* hw_config.c
Copyright 2021 Carl John Kugler III

Licensed under the Apache License, Version 2.0 (the License); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at

   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an AS IS BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.
*/

/*
This file should be tailored to match the hardware design.

See
https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/tree/main#customizing-for-the-hardware-configuration
*/

/**
 * @file hw_config.c
 * @author Carl John Kugler III, Thomas Chang
 * @brief This is a modified configuration file from the FatFS library. Configured to communicate with the microSD via SPI with pin definitions dependent on platform.
 * @version 0.1
 * @date 2025-05-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "hw_config.h"

#ifndef PCB
#define PCB 0
#endif

// Pin Defintions (see hw_config.c for SPI)
#if PCB == 0
    #define MY_SPI      spi0
    #define SPI_SCK    18  // SCK
    #define SPI_MOSI   19  // MOSI
    #define SPI_MISO   20  // MISO
    #define SPI_CS     1   // RX (BB Pin)
#elif PCB == 1
    #define MY_SPI      spi0
    #define SPI_SCK    18
    #define SPI_MOSI   19
    #define SPI_MISO   20
    #define SPI_CS     21
#else
    #error "Unsupported PLATFORM value"
#endif

/* Configuration of hardware SPI object */
static spi_t spi = {
    .hw_inst     = MY_SPI,  // SPI component
    .sck_gpio    = SPI_SCK,    // GPIO number (not Pico pin number)
    .mosi_gpio   = SPI_MOSI,
    .miso_gpio   = SPI_MISO,
    // THOMAS CHANG: We've kept this baud rate to match that of the 2040
    .baud_rate = 125 * 1000 * 1000 / 4  // 31250000 Hz

};

/* SPI Interface */
static sd_spi_if_t spi_if = {
    .spi = &spi,  // Pointer to the SPI driving this card
    .ss_gpio     = SPI_CS  // The SPI slave select GPIO for this SD card
};

/* Configuration of the SD Card socket object */
static sd_card_t sd_card = {
    .type = SD_IF_SPI,
    .spi_if_p = &spi_if  // Pointer to the SPI interface driving this card
};

/* ********************************************************************** */

size_t sd_get_num() { return 1; }

/**
 * @brief Get a pointer to an SD card object by its number.
 *
 * @param[in] num The number of the SD card to get.
 *
 * @return A pointer to the SD card object, or @c NULL if the number is invalid.
 */
sd_card_t *sd_get_by_num(size_t num) {
    if (0 == num) {
        // The number 0 is a valid SD card number.
        // Return a pointer to the sd_card object.
        return &sd_card;
    } else {
        // The number is invalid. Return @c NULL.
        return NULL;
    }
}

/* [] END OF FILE */