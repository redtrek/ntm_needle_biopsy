/**
 * @file config.h
 * @author Thomas Chang
 * @brief This holds the macros for I2C device addresses, general debug and timing values, control abstractions, and ADC power calculations.
 * @version 0.1
 * @date 2025-05-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

// I2C addresses for peripheral modules
#define OLED_ADDR   0x3C
#define INA219_ADDR 0x40
#define FX29_ADDR   0x28

// General timing
#define debounce_us   100000
#define hold_us       3000000   ///< Hold [STATE] for 3 sec to enter ZERO state.
#define potIterations 1000
#define debug_us      5000000


//==== ZERO STATE ====//
#define SPIKE_TIME      500     ///< Time in ms used to avoid measuring current spikes during the ZERO state. Without this the ZERO state will exit immediately as it will detect motor startup spikes as collision with the housing.
#define CUTOFF_STALL    800.0f  ///< Stall current that determines when the ZERO state is complete. If the device detects this current, it will be because it has reached the housing.
//==== ZERO STATE ====//



//==== MOTOR COMMANDS ====//
/**
 * @defgroup MotorMacros Motor Macros
 * Macros that abstract controlling the motor.
 * @{
 */
#define MOTOR_FW          1     
#define MOTOR_BW          0     
#define MOTOR_ON          255   ///< Maximum PWM value for motor
#define MOTOR_OFF         0     ///< Minimum PWM value for motor
/** @} */
//==== MOTOR COMMANDS ====//



//==== BATTERY CALCULATIONS ====//
/**
 * @defgroup BatteryMacros Battery Macros
 * Macros used for calculating the battery capacity.
 * @{
 */

/// Calculates the bigger value between A and B using operator. Essentially: if (A) >= (B) return A. Else return B.
#define NTM_MAX(A, B) ((A) >= (B) ? (A) : (B))

/// Calculates the smaller value between A and B using operator. Essentially: if (A) <= (B) return A. Else return B.
#define NTM_MIN(A, B) ((A) <= (B) ? (A) : (B))

/**
 * @brief Represents the battery's max capacity from the perspective of the microcontroller.
 * This code maps the battery's voltage to the ADC module value.
 * The battery's "maximum charge" is 4.2 V which is too much for the pins to handle so we halve it using a voltage divider. Therefore, when the battery is at maximum the microcontroller pin recieves 2.1 V.
 * We then map this to the 12 bit ADC value. There are 12 bits to represent the voltage so 4096 corresponding values (inclusive of 0) with a maximum of 3.3 V recievable by the pin.
 * When we multiply these to values together we are figuring out what the ADC value of the battery is at maximum charge (4.2 V / 2)
 * 0.5 is just a value to round this result to the nearest whole integer.
 */
#define BAT_MAX_ADC ((int)(0.5 + (4095/3.3) * (4.2/2)))

/**
 * @brief Represents the battery's min capacity from the perspective of the microcontroller.
 * This code maps the battery's voltage to the ADC module value.
 * The battery's "minimum charge" is 3.2 V. To be consistent with what we did for the maximum voltage we halve it using a voltage divider. Therefore, when the battery is at maximum the microcontroller pin recieves 1.6 V.
 * We then map this to the 12 bit ADC value. There are 12 bits to represent the voltage so 4096 corresponding values (inclusive of 0) with a maximum of 3.3 V recievable by the pin.
 * When we multiply these to values together we are figuring out what the ADC value of the battery is at maximum charge (3.2 V / 2)
 * 0.5 is just a value to round this result to the nearest whole integer.
 */
#define BAT_MIN_ADC ((int)(0.5 + (4095/3.3) * (3.2/2)))

/// Looks strange out of context. Used to calculate the batter capacity i.e. (Current Battery Value - BAT_MIN_ADC) * BAT_ADC_PER = % Capacity
#define BAT_ADC_PER (100.0 / (BAT_MAX_ADC - BAT_MIN_ADC))

/// 1 Second in microseconds.
#define SEC_US      1000000
/** @} */
//==== BATTERY CALCULATIONS ====//