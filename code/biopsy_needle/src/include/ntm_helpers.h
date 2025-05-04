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

void board_gpio_init();
void setMotor(bool direction, uint16_t power);
void displayData(ssd1306_t* screen, int y_pos, float data, std::string info);
float getRevolutions(int count);

