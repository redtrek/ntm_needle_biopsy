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

float compute_force(uint16_t force_data) {
    float force;
    float N_perbit = 0.00794f;

    force = ((force_data - 1000.0f) * N_perbit);
    return force;
}