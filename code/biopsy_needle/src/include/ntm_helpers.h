/**
 * @file ntm_helpers.h
 * @author Thomas Chang
 * @brief This file holds the prototypes for smart needle's helper functions.
 * @version 0.1
 * @date 2025-05-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "pins.h"
#include "config.h"

#include <iostream>
#include <string>
#include <stdio.h>

// Peripherals
#include "../../libs/SSD1306/ssd1306.h"

// Includes for the FatFS Library
#include "pico/stdlib.h"
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"

// MSC-specific
#include <bsp/board.h>
#include <stdlib.h>
#include <tusb.h>
#include "tusb_config.h"

/**
 * @brief Initiates all GPIO pins as well as communication protocols.
 * 
 */
void board_gpio_init();

/**
 * @brief Turns on motor with configureable speed and direction values.
 * 
 * @param direction Integer value where 1 means forward and 0 means backward.
 * @param power Integer value representing power of the motor ranging from 0-255 where 255 is the maximum.
 */
void setMotor(bool direction, uint16_t power);

/**
 * @brief Helper function used to abstract how sensor data is shown on the OLED screen
 * 
 * @param screen OLED object representing the actual screen in firmware.
 * @param y_pos The y position in pixels (bottom left) of where the text is written.
 * @param data The actual sensor information.
 * @param info Any string messages detailing what the sensor information is measuring.
 */
void displayData(ssd1306_t* screen, int y_pos, float data, std::string info);

/**
 * @brief Takes detected motor signals and converts them to the real-world number of revolutions of the motor shaft. 
 * 
 * @details The conversion is as follows: number_of_counts * (1 small revolution / 12 counts) * (1 big revolution / 34.014 small revolutions). 34.014 is the gear ratio whilst the (small revolutions / counts) is from the motor documentation.
 * @param count Number of signal edges recieved from motor (see quadrature encoder information in motor documentation).
 * @return float 
 */
float getRevolutions(int count);

