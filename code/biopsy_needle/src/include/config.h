#pragma once

// I2C addresses for peripheral modules
#define OLED_ADDR   0x3C
#define INA219_ADDR 0x40
#define FX29_ADDR   0x28

// General timing
#define debounce_us   100000
#define hold_us       3000000 // 3 second hold for zeroing functionality
#define potIterations 1000

// ZERO state timing
#define SPIKE_TIME  500 // Time in ms used to avoid spikes during ZERO state.
#define CUTOFF_STALL 800.0f // Stall current in mA used to ZERO the device

// Motor commands
#define MOTOR_FW          1    
#define MOTOR_BW          0    
#define MOTOR_ON          255  
#define MOTOR_OFF         0    

// Battery Monitoring and ADC
#define NTM_MAX(A, B) ((A) >= (B) ? (A) : (B))
#define NTM_MIN(A, B) ((A) <= (B) ? (A) : (B))
#define BAT_MAX_ADC ((int)(0.5 + (4095/3.3) * (4.2/2)))
#define BAT_MIN_ADC ((int)(0.5 + (4095/3.3) * (3.2/2)))
#define BAT_ADC_PER (100.0 / (BAT_MAX_ADC - BAT_MIN_ADC))
#define BAT_ADC_OFFSET  -320
#define SEC_US      1000000