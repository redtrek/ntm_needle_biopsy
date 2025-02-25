/**
 * @file main.cpp
 * @author Thomas Chang
 * @brief This is the main file for the electronic biopsy needle. It handles the operation of the state machine, sensors, motors, and display.
 * @version 1.0
 * @date 2025-02-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <iostream>
#include <stdio.h>
#include <string>
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"

// Includes for the FatFS Library
#include "pico/stdlib.h"
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"

// MSC-specific
#include <bsp/board.h>
#include <stdlib.h>
#include <tusb.h>
#include "include/tusb_config.h"

// Peripheral Devices
#include "../libs/INA219_driver/INA219.h"
#include "../libs/SSD1306/ssd1306.h"
#include "../libs/FX29/fx29.h"


#define I2C0_SDA    12  // 12
#define I2C0_SCL    13  // 13

#define SPI0_SCK    18  // SCK
#define SPI0_MOSI   19  // MOSI
#define SPI0_MISO   20  // MISO
#define SPI0_CS     1   // RX

#define UART_TX     24  // 24
#define UART_RX     25  // 25
#define UART_ID     uart1
#define BAUD_RATE   115200

#define OLED_ADDR   0x3C
#define INA219_ADDR 0x40
#define FX29_ADDR   0x28

#define state_input 11   // 11
#define speed_input 29   // A3
#define msc_input   10   // 10
//#define msc_signal  10   // 10

#define debounce_us   100000
#define hold_us       3000000 // 3 second hold for zeroing functionality
#define potIterations 1000

#define PWM         0    // TX
#define DIR         6    // D4

#define motorA_out  9    // 9
#define motorB_out  8    // SCL

#define MAF_sz      5    // Window size for Moving Average Filter.

using namespace std;

// ==== Experimental Values ==== //
int fwRev = 50;
int bwRev = 0;

// ==== Function Prototypes ==== //
void gpio_init();
void oled_init();
string stateToString();
void displayState();
void displayInputSpeed();
void displayRPM();
void displayCurrent();
void displayPos();
void handleButton();
void handleRelease();
void handleMSCButton();
long getInputSpeed();
float getRevolutions();
void getRPM();
void createDataFile();
void resetFiltering();

// ==== Globals ==== //
ssd1306_t oled;
uint16_t speed_lvl = 0;
long temp_speed = 0;
int count = 0;
int numPulses = 0;
absolute_time_t now = get_absolute_time();
absolute_time_t prevTime = get_absolute_time();
absolute_time_t pressTime = get_absolute_time();
absolute_time_t pressedTime = get_absolute_time();
absolute_time_t mscPressTime = get_absolute_time();
float rpm = 0;
float current = 0;
float displacement = 0;
float lp_current = 0;
float lp_alpha = 0.1; // Smoothing factor for LP Filter
float MAF[MAF_sz] = {0};
int MAF_counter = 0;
float MAF_sum = 0;
uint slice = pwm_gpio_to_slice_num(PWM);
uint channel = pwm_gpio_to_channel(PWM);

// ==== Debugging ==== //
#define debug_us   5000000
absolute_time_t prevBug = get_absolute_time();
absolute_time_t currBug = get_absolute_time();

// ==== Flags ==== //
volatile bool motor_flag = false;
volatile bool button_press_flag = false;
volatile bool button_release_flag = false;
volatile bool button_msc_flag = false;

// ==== Data Logging ==== //
FATFS filesys;
FIL fil;
TCHAR filename[20] = "data0.csv\0";

// ==== Motor Operation State Machine ==== //
enum states {
    WAIT,
    STANDBY,
    CUTTING,
    REMOVAL,
    EXITING,
    FINISH,
    ZERO
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
        if (gpio_get(state_input) == 1) {
            pressTime = get_absolute_time();
            button_press_flag = true;
        } else if (gpio_get(state_input) == 0) {
            button_release_flag = true;
        }
        
    } else if (gpio == msc_input) {
        mscPressTime = get_absolute_time();
        button_msc_flag = true;
    }
}

int main() {
    // MSC: Initialize board
    board_init();
    printf("\033[2J\033[H");
    tud_init(BOARD_TUD_RHPORT);

    // Initialize serial port and wait for serial monitor
    stdio_init_all();
    gpio_init(PWM);
    gpio_put(PWM, 0);
    sleep_ms(3000);
    
    // ==== General Initialization ==== //
    gpio_init();
    oled_init();
    
    state = WAIT;
    nextState = STANDBY;

    INA219 ina219(i2c0, INA219_ADDR);
    ina219.calibrate(0.1, 3.2);

    // ==== Interrupts ==== //
    gpio_set_irq_enabled_with_callback(motorA_out, GPIO_IRQ_EDGE_FALL, true, &gpio_ISR);
    gpio_set_irq_enabled(state_input, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(msc_input, GPIO_IRQ_EDGE_FALL, true);

    while(1) {
        //uart_putc(UART_ID, 'A');
        //uart_puts(uart1, "Bruh\n");
        //printf("Bruh\n");
        now = get_absolute_time();
        int64_t time_ms = to_ms_since_boot(now);

        handleRelease();
        handleButton();
        handleMSCButton();
        getRPM();
        
        current = ina219.read_current() * 1000;
        displacement = getRevolutions() * 0.5f;

        if (MAF_counter == MAF_sz) {
            MAF_counter = 0;
        }
        // Remove oldest value and add newest value to sum
        MAF_sum -= MAF[MAF_counter];
        MAF_sum += current;
        MAF[MAF_counter] = current;

        MAF_counter++;
        float MAF_current = MAF_sum * 1.0f/MAF_sz;

        switch(state) {
            case WAIT: {
                // Enable MSC
                f_unmount("");
                tud_task();

                gpio_put(DIR, 1);
                pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                
                nextState = STANDBY;
                break;
            }
            case STANDBY: {


                // Disable MSC
                tud_disconnect();
                f_mount(&filesys, "", 1);
                tud_deinit(BOARD_TUD_RHPORT);

                //gpio_put(msc_signal, 0);

                gpio_put(DIR, 1);
                pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                
                temp_speed = getInputSpeed();
                speed_lvl = uint16_t(temp_speed / 100.0f * 255.0f);         
                resetFiltering();
                displayState();

                nextState = CUTTING;
                break;
            }
            case CUTTING: {
                speed_lvl = speed_lvl;
                
                lp_current = lp_alpha * current + (1 - lp_alpha) * lp_current;
                f_printf(&fil, "%s,%lld,%f,%f,%f,%f,%f\n", stateToString().c_str(), time_ms, current, lp_current, MAF_current, rpm, displacement);

                // ==== SAFETY CHECK ==== //
                if (getRevolutions() > fwRev) {
                    gpio_put(DIR, 0);
                    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                    state = REMOVAL;
                } else {
                    gpio_put(DIR, 1);
                    pwm_set_chan_level(slice, PWM_CHAN_A, speed_lvl);
                }
                displayState();
                
                nextState = REMOVAL;
                break;
            }
            case REMOVAL: {

                gpio_put(DIR, 0);
                pwm_set_chan_level(slice, PWM_CHAN_A, 0);

                temp_speed = getInputSpeed();
                speed_lvl = int(temp_speed / 100.0f * 255.0f);
                resetFiltering();
                displayState();

                nextState = EXITING;
                break;
            }
            case EXITING: {
                speed_lvl = speed_lvl;

                lp_current = lp_alpha * current + (1 - lp_alpha) * lp_current;
                f_printf(&fil, "%s,%lld,%f,%f,%f,%f,%f\n", stateToString().c_str(), time_ms, current, lp_current, MAF_current, rpm, displacement);

                // ==== SAFETY CHECK ==== //
                if (getRevolutions() < bwRev) {
                    gpio_put(DIR, 1);
                    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                    state = FINISH;
                } else {
                    gpio_put(DIR, 0);
                    pwm_set_chan_level(slice, PWM_CHAN_A, speed_lvl);
                }
                displayState();

                nextState = FINISH;
                break;
            }
            case FINISH: {
                f_close(&fil);
                f_unmount("");

                gpio_put(DIR, 1);
                pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                
                displayState();

                nextState = STANDBY;
                break;
            }
            case ZERO: {
                gpio_put(DIR, 1);
                pwm_set_chan_level(slice, PWM_CHAN_A, 255);

                if (current > 1.5f) {
                    count = 0;
                    state = FINISH;
                }
                
                displayState();

                nextState = FINISH;
                break;
            }
        }

    }
    return 0;
}

void gpio_init() {
    // Potentiometer Input
    adc_init();
    adc_gpio_init(speed_input);
    adc_select_input(3);

    // Pushbutton State Input
    gpio_init(state_input);
    gpio_set_dir(state_input, GPIO_IN);
    gpio_pull_down(state_input);

    // Pushbutton MSC Input
    gpio_init(msc_input);
    gpio_set_dir(msc_input, GPIO_IN);
    gpio_pull_down(msc_input);
    
    // Motor Control
    gpio_set_function(PWM, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, 48.83f);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_enabled(slice, true);
    gpio_init(DIR);
    gpio_set_dir(DIR, GPIO_OUT);

    // Motor Feedback - Output Signals
    gpio_init(motorA_out);
    gpio_set_dir(motorA_out, GPIO_IN);
    gpio_pull_up(motorA_out);
    gpio_init(motorB_out);
    gpio_set_dir(motorB_out, GPIO_IN);
    gpio_pull_up(motorB_out);
    
    // I2C Initialization
    i2c_init(i2c0, 400000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);

    // UART Initialization - Second Serial Monitor
    uart_init(uart1, BAUD_RATE);
    gpio_set_function(UART_TX, UART_FUNCSEL_NUM(UART_ID, UART_TX));
    gpio_set_function(UART_RX, UART_FUNCSEL_NUM(UART_ID, UART_RX));

    // MSC Reset Pin Control
    /*
    gpio_init(msc_signal);
    gpio_put(msc_signal, 1);
    gpio_set_dir(msc_signal, GPIO_OUT);
    */
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
        ssd1306_draw_string(&oled, 0, 2, 2, "RECORDS");
        ssd1306_draw_string(&oled, 0, 20, 1, "PRESS state button to");
        ssd1306_draw_string(&oled, 0, 30, 1, "begin operation.");
        ssd1306_draw_string(&oled, 0, 44, 1, "HOLD state button to");
        ssd1306_draw_string(&oled, 0, 55, 1, "zero device.");
        ssd1306_show(&oled);
    } else {
        printf("OLED initialization failed.\n");
    }
}

void createDataFile() {
    
    int fileNum = 0;
    FRESULT file_created = f_open(&fil, filename, FA_CREATE_NEW | FA_WRITE);
    while (file_created == FR_EXIST) {
        fileNum++;
        sprintf(filename, "data%d.csv", fileNum);
        file_created = f_open(&fil, filename, FA_CREATE_NEW | FA_WRITE);
    }

    // Print header of file
    f_printf(&fil, "State, Time(ms), Current(mA), CurrentLP(mA), CurrentMAF(mA), RPM, Displacement(mm)\n");
}

void displayInputSpeed() {
    std::string temp = "INPUT SPEED: " + to_string(temp_speed) + "%";
    const char* in_speed = temp.c_str();
    ssd1306_draw_string(&oled, 0, 50, 1, in_speed);
}

void displayState() {
    ssd1306_clear(&oled);
    switch(state) {
            case STANDBY:
                ssd1306_draw_string(&oled, 0, 2, 2, "STANDBY");
                ssd1306_draw_string(&oled, 0, 20, 1, "PRESS MSC button to");
                ssd1306_draw_string(&oled, 0, 30, 1, "access past logs.");
                displayInputSpeed();
                break;
            
            case CUTTING:
                ssd1306_draw_string(&oled, 0, 2, 2, "CUTTING");
                displayRPM();
                displayCurrent();
                displayPos();
                break;
            
            case REMOVAL:
                ssd1306_draw_string(&oled, 0, 2, 2, "REMOVAL");
                displayPos();
                displayInputSpeed();
                break;
            
            case EXITING:
                ssd1306_draw_string(&oled, 0, 2, 2, "EXITING");
                displayRPM();
                displayCurrent();
                displayPos();
                break;
            
            case FINISH: // For debugging purposes.
                ssd1306_draw_string(&oled, 0, 2, 2, "TASK DONE");
                ssd1306_draw_string(&oled, 0, 20, 1, "PRESS state button");
                ssd1306_draw_string(&oled, 0, 30, 1, "for new operation.");
                ssd1306_draw_string(&oled, 0, 44, 1, "PRESS MSC button to");
                ssd1306_draw_string(&oled, 0, 54, 1, "access log.");
                break;
            case ZERO:
                ssd1306_draw_string(&oled, 0, 2, 2, "ZERO");
                ssd1306_draw_string(&oled, 0, 20, 1, "Resetting to origin");
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
    ssd1306_draw_string(&oled, 0, 30, 1 , curr);
}

void displayPos() {
    std::string temp = "Position: " + to_string(displacement);
    const char* pos = temp.c_str();
    ssd1306_draw_string(&oled, 0, 40, 1 , pos);
}

string stateToString() {
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

bool validPress = false;

void handleButton() {
    if (button_press_flag) {
        if (absolute_time_diff_us(pressTime, now) >= debounce_us) {
            if (gpio_get(state_input) == 1) {
                validPress = true;
            } else {
                button_press_flag = false;
            }
        }

        if (validPress) {
            if (absolute_time_diff_us(pressTime, now) >= hold_us) {
                state = ZERO;
                nextState = FINISH;
                button_press_flag = false;
                validPress = false;
            } 
        }
    }
}

void handleRelease() {
    if (button_release_flag) {
        if (validPress) {
            state = nextState;
            //gpio_put(msc_signal, 1);
            if (state == CUTTING) {
                createDataFile();
            }
            validPress = false;
        }
        button_release_flag = false;
    }
}

void handleMSCButton() {
    if (button_msc_flag) {
        if (absolute_time_diff_us(mscPressTime, now) >= debounce_us) {
            if (state == STANDBY || state == FINISH) {
                if (gpio_get(state_input) == 0) {
                    watchdog_enable(1, 1);
                }
            }
            button_msc_flag = false;
        }
    }
}

void resetFiltering() {
    // Initialize all moving average filter values to zero.
    MAF_counter = 0;
    MAF_sum = 0;
    memset(MAF, 0, sizeof(MAF));

    // Reset previous low pass output.
    lp_current = 0;
}