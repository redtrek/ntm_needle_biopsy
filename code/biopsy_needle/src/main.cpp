/**
 * @file main.cpp
 * @author Thomas Chang
 * @brief This is the main file for the electronic biopsy needle. It handles the operation of the state machine, sensors, motors, and display.
 * @version 0.1
 * @date 2024-12-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <stdio.h>
#include <string>
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

// Includes for the FatFS Library
#include "pico/stdlib.h"
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"

#include "../libs/INA219_driver/INA219.h"
#include "../libs/SSD1306/ssd1306.h"


#define I2C0_SDA    12  // 12
#define I2C0_SCL    13  // 13

#define SPI0_SCK    18  // SCK
#define SPI0_MOSI   19  // MOSI
#define SPI0_MISO   20  // MISO
#define SPI0_CS     1   // RX

#define OLED_ADDR   0x3C
#define INA219_ADDR 0x40

#define state_input 11   // 11
#define speed_input 29   // A3

#define debounce_us   10000
#define potIterations 1000

#define PWM         0    // TX
#define DIR         6    // D4

#define motorA_out  9    // 9
#define motorB_out  8    // SCL

using namespace std;

// ==== Experimental Values ==== //
int fwRev = 50;
int bwRev = 0;

// ==== Function Prototypes ==== //
void init_i2c0();
void oled_init();
void sd_test();
void displayState();
void displayInputSpeed();
void handleButton();
long getInputSpeed();
float getRevolutions();
void getRPM();
void displayRPM();
void displayCurrent();


// ==== Globals ==== //
ssd1306_t oled;
uint16_t speed_lvl = 0;
long temp_speed = 0;
int count = 0;
int numPulses = 0;
absolute_time_t prevTime = get_absolute_time();
absolute_time_t currTime = get_absolute_time();
float rpm = 0;
float current = 0;
uint slice = pwm_gpio_to_slice_num(PWM);
uint channel = pwm_gpio_to_channel(PWM);

// ==== Debugging ==== //
#define debug_us   5000000
absolute_time_t prevBug = get_absolute_time();
absolute_time_t currBug = get_absolute_time();

// ==== Interrupt Flags ==== //
volatile bool motor_flag = false;
volatile bool button_flag = false;

// ==== Motor Operation State Machine ==== //
enum states {
    WAIT,
    STANDBY,
    CUTTING,
    REMOVAL,
    EXITING
};
enum states state, nextState;

// ==== Interrupt Service Routines ==== //
void gpio_ISR(uint gpio, uint32_t events) {
    if (gpio == motorA_out) {
        numPulses++;
        if (gpio_get(DIR) == 0) {
                count--;
            } else {
                count++;
            }
    } else if (gpio == state_input) {
        button_flag = true;
    }
}

//FATFS filesys;
//FRESULT fr = f_mount(&filesys, "", 1);
//FIL fil;

int main() {
    // Initialize serial port and wait for serial monitor
    stdio_init_all();
    gpio_init(PWM);
    gpio_put(PWM, 0);
    sleep_ms(3000);

    // FatFS

    // ==== Pin Initialization ==== //
    {
        adc_init();
        adc_gpio_init(speed_input);
        adc_select_input(3);

        gpio_init(state_input);
        gpio_set_dir(state_input, GPIO_IN);
        gpio_pull_down(state_input);
        
        
        gpio_set_function(PWM, GPIO_FUNC_PWM);
        pwm_set_clkdiv(slice, 48.83f);
        pwm_set_wrap(slice, 255);
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_enabled(slice, true);
        gpio_init(DIR);
        gpio_set_dir(DIR, GPIO_OUT);

        gpio_init(motorA_out);
        gpio_set_dir(motorA_out, GPIO_IN);
        gpio_pull_up(motorA_out);
        
        gpio_init(motorB_out);
        gpio_set_dir(motorB_out, GPIO_IN);
        gpio_pull_up(motorB_out);
    }

    // ==== General Initialization ==== //
    init_i2c0();
    oled_init();
    //sd_test();
    
    state = WAIT;
    nextState = STANDBY;

    INA219 ina219(i2c0, INA219_ADDR);
    ina219.calibrate(0.1, 3.2);

    // ==== Interrupts ==== //
    gpio_set_irq_enabled_with_callback(motorA_out, GPIO_IRQ_EDGE_FALL, true, &gpio_ISR);
    gpio_set_irq_enabled(state_input, GPIO_IRQ_EDGE_FALL, true);

    const char* const filename = "demo.txt";

    while(1) {
        handleButton();
        getRPM();
        
        current = ina219.read_current();
        float voltage = ina219.read_voltage();
        
        switch(state) {
            case WAIT:
                
                gpio_put(DIR, 1);
                pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                
                nextState = STANDBY;
                break;

            case STANDBY:
                gpio_put(DIR, 1);
                pwm_set_chan_level(slice, PWM_CHAN_A, 0);

                temp_speed = getInputSpeed();
                speed_lvl = uint16_t(temp_speed / 100.0f * 255.0f);         

                displayState();
                nextState = CUTTING;
                break;
            
            case CUTTING:
                displayState();
                nextState = REMOVAL;
                speed_lvl = speed_lvl;

                // ==== SAFETY CHECK ==== //
                if (getRevolutions() > fwRev) {
                    gpio_put(DIR, 0);
                    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                    state = REMOVAL;
                } else {
                    gpio_put(DIR, 1);
                    pwm_set_chan_level(slice, PWM_CHAN_A, speed_lvl);
                }

                break;
            
            case REMOVAL:
                gpio_put(DIR, 0);
                pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                temp_speed = getInputSpeed();
                speed_lvl = int(temp_speed / 100.0f * 255.0f);
                

                displayState();
                nextState = EXITING;
                break;
            
            case EXITING:
                speed_lvl = speed_lvl;

                // ==== SAFETY CHECK ==== //
                if (getRevolutions() < bwRev) {
                    gpio_put(DIR, 1);
                    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                    state = STANDBY;
                } else {
                    gpio_put(DIR, 0);
                    pwm_set_chan_level(slice, PWM_CHAN_A, speed_lvl);
                }

                displayState();
                nextState = STANDBY;
                break;
            
        }

    }
    return 0;
}

void init_i2c0() {
    i2c_init(i2c0, 400000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);
}

void oled_init() {

    // external_vcc must be false for this to work
    oled.external_vcc = false;
    bool oled_status = ssd1306_init(
        &oled,
        128,
        64,
        OLED_ADDR,
        i2c0
    );

    if (oled_status) {
        ssd1306_clear(&oled);
        ssd1306_draw_string(
            &oled,
            0,
            2,
            1,
            "Starting... :)"
        );
        ssd1306_show(&oled);
    } else {
        printf("OLED initialization failed.\n");
    }
}
/*
void sd_test() {
    FATFS filesys;
    FRESULT fr = f_mount(&filesys, "", 1);
    
    if (fr != FR_OK) {
        while(true) {
            printf("ERROR: could not mount filesystem \n", fr);
        }
    }
    FIL fil;
    const char* const filename = "testing03.txt";
    
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        
        while (true) {
            printf("ERROR: Could not open file (%d)\r\n", fr);
        }
    }
    f_printf(&fil, "Testing new library again \n");
    
    fr = f_close(&fil);
    if (fr != FR_OK) {
        while (true) {
            printf("ERROR: Could not close file (%d)\r\n", fr);
        }
    }


    // Unmount drive
    f_unmount("");

}
*/
void displayInputSpeed() {
    std::string temp = "Input Speed: " + to_string(temp_speed) + "%";
    const char* in_speed = temp.c_str();
    ssd1306_draw_string(&oled, 0, 20, 1, in_speed);
}

void displayState() {
    ssd1306_clear(&oled);
    switch(state) {
            case STANDBY:
                ssd1306_draw_string(&oled, 0, 2, 2, "STANDBY");
                displayInputSpeed();
                break;
            
            case CUTTING:
                ssd1306_draw_string(&oled, 0, 2, 2, "CUTTING");
                displayRPM();
                displayCurrent();
                break;
            
            case REMOVAL:
                ssd1306_draw_string(&oled, 0, 2, 2, "REMOVAL");
                displayInputSpeed();
                break;
            
            case EXITING:
                ssd1306_draw_string(&oled, 0, 2, 2, "EXITING");
                displayRPM();
                displayCurrent();
                break;
            
        }
    ssd1306_show(&oled);
}

void displayRPM() {
    std::string temp = "RPM: " + to_string(rpm);
    const char* rpm = temp.c_str();
    ssd1306_draw_string(&oled, 0, 20, 1, rpm);
}

void displayCurrent() {
    std::string temp = "Current: " + to_string(current);
    const char* curr = temp.c_str();
    ssd1306_draw_string(&oled, 0, 40, 1 , curr);
}

std::string stateToString() {
    switch(state) {
        case WAIT:    return "WAIT";
        case STANDBY: return "STANDBY";
        case CUTTING: return "CUTTING";
        case REMOVAL: return "REMOVAL";
        case EXITING: return "EXITING";
        default:      return "STATE ERROR"; 
    }
}

long getInputSpeed() {
    long val = 0;
    for (int i = 0; i < potIterations; i++) {
        val += adc_read();
    }
    
    return long(0.0243 * (val * 1.0 / potIterations) + 1);
}

float getRevolutions() {
    float re = count/12.0/34.014f;
    return re;
}

void getRPM() {
    absolute_time_t currTime = get_absolute_time();
    //printf("Current time: %lld us\n", currTime);
    //printf("Prev time: %lld us\n", prevTime);

    if (absolute_time_diff_us(prevTime, currTime) >= 1000000) {
        //printf("this point has been reached! %d", currTime);
        
        rpm = (numPulses/12.0f/34.014f) * 60.0f;
        numPulses = 0;
        prevTime = currTime;
    }
}

void handleButton() {
    if (button_flag) {
        currTime = get_absolute_time();
        if (absolute_time_diff_us(prevTime, currTime) >= debounce_us) {
            if (gpio_get(state_input) == 0) {
                state = nextState;
                //std::string temp = "State: " + stateToString() + ":\n";
                //const TCHAR* name = temp.c_str();
                //f_printf(&fil, name);
            }
            button_flag = false; 
        }
    }
}