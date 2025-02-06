/* NTM Manufacturing Lab - Compact Compression Load Cell Library
// Author: Darren Leong
// Desc: I2C Driver for the FX29 load cell for Adafruit Pico/RP2040 boards.
*/

#include "fx29.h"

// Read data from the FX29
uint16_t FX29_read(i2c_inst_t *i2c, uint8_t sladdr){
    uint8_t no_value = 0;

    uint8_t buffer[2];

    uint8_t byte1, byte2;

    // Initiate Measurement Request
    i2c_read_blocking(i2c, sladdr, &no_value, 0, false);

    // Read data
    i2c_read_blocking(i2c, sladdr, buffer, 2, false);

    // 14-bit data
    byte1 = buffer[0] & 0x3F;
    byte2 = buffer[1];

    uint16_t data = (byte1 << 8) | byte2;

    return data; 
}

uint16_t compute_force(uint16_t force_data) {
    uint32_t temp_force;
    uint16_t force;
    const uint16_t scale = 100;
    const uint16_t divisor = 125.893;

    if (force_data < 1000) {
        force = 0;
    } else {
        temp_force = ((force_data - 1000) * scale) / divisor;
        force = (uint16_t) temp_force;
    }

    return force;
}