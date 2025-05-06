/**
 * @file ntm_helpers.cpp
 * @author Thomas Chang
 * @brief This file holds the definitions for the smart needle's helper functions.
 * @version 0.1
 * @date 2025-05-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "include/ntm_helpers.h"

/**
 * @brief Global slice value for the PWM module. Automatically determined by chosen pin at runtime. Corresponds to the organization of PWM configurations by clock. Please see RP2040 documentation.
 * 
 */
uint slice =  pwm_gpio_to_slice_num(MOTOR_PWM);

/**
 * @brief Global channel value for the PWM module. Automatically determined by chosen pin at runtime. Corresponds to PWM output pin itself. Please see RP2040 documentation.
 * 
 */
uint channel = pwm_gpio_to_channel(MOTOR_PWM);

void board_gpio_init() {
    // Potentiometer Input
    adc_init();
    adc_gpio_init(speed_input);
    adc_gpio_init(bat_lvl);

    // Pushbutton State Input
    gpio_init(state_input);
    gpio_set_dir(state_input, GPIO_IN);
    gpio_pull_down(state_input);

    // Pushbutton MSC Input
    gpio_init(msc_input);
    gpio_set_dir(msc_input, GPIO_IN);
    gpio_pull_down(msc_input);
    
    // Motor Control
    gpio_set_function(MOTOR_PWM, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, 48.83f);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, channel, 0);
    pwm_set_enabled(slice, true);
    gpio_init(MOTOR_DIR);
    gpio_set_dir(MOTOR_DIR, GPIO_OUT);

    // Motor Feedback - Output Signals
    gpio_init(motorA_out);
    gpio_set_dir(motorA_out, GPIO_IN);
    gpio_pull_up(motorA_out);
    gpio_init(motorB_out);
    gpio_set_dir(motorB_out, GPIO_IN);
    gpio_pull_up(motorB_out);
    
    // I2C Initialization
    i2c_init(MY_I2C, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void setMotor(bool direction, uint16_t power) {
    gpio_put(MOTOR_DIR, direction);
    pwm_set_chan_level(slice, channel, power);
}

void displayData(ssd1306_t* screen, int y_pos, float data, std::string info) {
    std::string temp = info + std::to_string(data);
    const char* data_formatted = temp.c_str();
    ssd1306_draw_string(screen, 0, y_pos, 1, data_formatted);
}

float getRevolutions(int count) {
    float re = count/12.0/34.014f;
    return re;
}

