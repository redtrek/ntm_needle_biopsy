#pragma once
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

uint16_t FX29_read(i2c_inst_t *i2c, uint8_t sladdr);

float compute_force(uint16_t force_data);

#ifdef __cplusplus
}
#endif