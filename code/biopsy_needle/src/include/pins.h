#pragma once

// Pin Defintions (see hw_config.c for SPI)
// Please see hw_config.c to change SPI configuration.
#ifndef PCB
#define PCB 0
#endif

#if PCB == 0
    #define MY_I2C      i2c0
    #define I2C_SDA     12
    #define I2C_SCL     13
    #define state_input 11
    #define msc_input   10
    #define speed_input 29 // A3 on Feather
    #define bat_lvl     26 // A0 on Feather
    #define PWM         0  // TX on Feather
    #define DIR         6  // D4 on Feather
    #define motorA_out  9
    #define motorB_out  8  // SCL on Feather
#elif PCB == 1
    #define MY_I2C      i2c0
    #define I2C_SDA     12
    #define I2C_SCL     13
    #define state_input 23
    #define msc_input   24
    #define speed_input 29
    #define bat_lvl     26
    #define PWM         0
    #define DIR         4
    #define motorA_out  9
    #define motorB_out  8  
#else
    #error "Unsupported PLATFORM value"
#endif