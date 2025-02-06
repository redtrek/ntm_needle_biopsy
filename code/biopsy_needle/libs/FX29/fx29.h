#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

uint16_t FX29_read(i2c_inst_t *i2c, uint8_t sladdr);

uint16_t FX29_read(i2c_inst_t *i2c, uint8_t sladdr);

uint16_t compute_force(uint16_t force_data);