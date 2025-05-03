/**
 * @file main.cpp
 * @author Thomas Chang
 * @brief This is the main file for the smart biopsy needle. It handles the high level operation of the state machine, sensors, motors, and display.
 * @version 1.0
 * @date 2025-04-30
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
#include "hardware/clocks.h"
#include "include/pins.h"
#include "include/config.h"
//#include "include/ntm_helpers.h"

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

using namespace std;

// ==== Experimental Values ==== //
int fwRev = 50;
int bwRev = 0;
#define MAF_SZ      5    // Window size for Moving Average Filter.
#define LP_ALPHA    0.1f // Smoothing factor for LP Filter

// ==== Function Prototypes ==== //
void gpio_init();
void oled_init();
string stateToString();
void displayState();
void displayInputSpeed(int y_pos);
void displayRPM(int y_pos);
void displayCurrent(int y_pos);
void displayPos(int y_pos);
void displayBat(int y_pos);
void displayForce(int y_pos);
void handleButton();
void handleRelease();
void handleMSCButton();
float getBatLevel();
long getInputSpeed();
float getRevolutions();
void getRPM();
void createDataFile();
void resetFiltering();
void setMotor(bool direction, uint16_t power);
void testingSuite();
void testMSC();
void testSD();
void testPins();
void enableMSC();
void disableMSC();

// ==== Globals ==== //
ssd1306_t oled;
uint16_t speed_lvl = 0;
long temp_speed = 0;
long bat_per = 0;
int count = 0;
int numPulses = 0;
absolute_time_t now = get_absolute_time();
absolute_time_t prevTime = get_absolute_time();
absolute_time_t pressTime = get_absolute_time();
absolute_time_t pressedTime = get_absolute_time();
absolute_time_t mscPressTime = get_absolute_time();
float rpm = 0;
float current_mA = 0;
float force = 0;
float displacement = 0;
float lp_current = 0;
float MAF[MAF_SZ] = {0};
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


void testingSuite() {
    stdio_init_all();
    sleep_ms(1000);

    gpio_init(26);
    gpio_set_dir(26, GPIO_OUT);

    // I2C
    i2c_init(i2c0, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // OLED
    oled_init();

    // BUTTMOTOR_ONS
    gpio_init(state_input);
    gpio_set_dir(state_input, GPIO_IN);
    gpio_pull_down(state_input);
    gpio_init(msc_input);
    gpio_set_dir(msc_input, GPIO_IN);
    gpio_pull_down(msc_input);

    // POTENTIOMETER
    adc_init();
    adc_gpio_init(speed_input);

    // Motor Control
    gpio_set_function(PWM, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, 48.83f);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_enabled(slice, true);
    gpio_init(DIR);
    gpio_set_dir(DIR, GPIO_OUT);

    while(1) {
        // Testing functionality of pushbuttons.
        while (gpio_get(msc_input) == 1 || gpio_get(state_input) == 1) {
            gpio_put(26, 1);
        }
        
        // Testing potentiometer.
        long test_pot = getInputSpeed() * 10;
        gpio_put(26, 1);
        sleep_ms(test_pot);
        gpio_put(26, 0);
        sleep_ms(test_pot);

        // Testing functionality of motor.
        if (gpio_get(state_input) == 1) {
            setMotor(MOTOR_FW, 255);
            sleep_ms(3000);
            setMotor(MOTOR_BW, MOTOR_OFF);
        }
        
    }
}
void testMSC() {
    board_init();
    printf("\033[2J\033[H");
    tud_init(BOARD_TUD_RHPORT);
    stdio_init_all();

    // I2C
    i2c_init(i2c0, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    uint32_t sys_freq = clock_get_hz(clk_sys);
    std::string temp = to_string(sys_freq);
    const char* sys_freq_str = temp.c_str();
    oled.external_vcc = false;
    bool oled_status = ssd1306_init(&oled, 128, 64, OLED_ADDR, MY_I2C);
    
    if (oled_status) {
        ssd1306_clear(&oled);
        ssd1306_draw_string(&oled, 0, 2, 2, "Sys Clock:");
        ssd1306_draw_string(&oled, 0, 20, 1 , sys_freq_str);
        ssd1306_show(&oled);
    } else {
        printf("OLED initialization failed.\n");
    }
    

    while(1) {
        tud_task();
    }
}
void testSD() {
    stdio_init_all();
    tusb_init();
    sleep_ms(5000);
    printf("Testing if seral works.\n");

    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (FR_OK != fr) {
        panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    }

    // Open a file and write to it
    FIL fil;
    const char* const filename = "filename.txt";
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr) {
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    }
    if (f_printf(&fil, "Hello, world!\n") < 0) {
        printf("f_printf failed\n");
    }

    // Close the file
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }

    // Unmount the SD card
    f_unmount("");

    while(1) {
        printf("Testing serial.\n");
        sleep_ms(1000);
    }
}
void testPins() {
    #define SPI_SCK    18
    #define SPI_MOSI   19
    #define SPI_MISO   20
    #define SPI_CS     21
    gpio_init(SPI_SCK);
    gpio_init(SPI_MOSI);
    gpio_init(SPI_MISO);
    gpio_init(SPI_CS);
    gpio_set_dir(SPI_SCK, 1);
    gpio_set_dir(SPI_MOSI, 1);
    gpio_set_dir(SPI_CS, 1);
    gpio_set_dir(SPI_MISO, 1);

    while(1) {
        gpio_put(SPI_SCK, 1);
        gpio_put(SPI_MOSI, 1);
        gpio_put(SPI_CS, 1);
        gpio_put(SPI_MISO, 1);
        sleep_ms(500);
        gpio_put(SPI_SCK, 0);
        gpio_put(SPI_MOSI, 0);
        gpio_put(SPI_CS, 0);
        gpio_put(SPI_MISO, 0);
        sleep_ms(500);
    }
}

int main() {
    // Uncomment to enter testing mode.
    //testingSuite();
    //testMSC();
    //testSD();
    //testSPI();

    // MSC: Initialize board
    board_init();
    tud_init(BOARD_TUD_RHPORT);

    // Initialize serial port and wait for serial monitor
    stdio_init_all();
    gpio_init(PWM);
    gpio_put(PWM, 0);
    sleep_ms(1000);
    
    // ==== General Initialization ==== //
    gpio_init();
    bat_per = getBatLevel();
    oled_init();
    state = WAIT;
    nextState = STANDBY;
    INA219 ina219(MY_I2C, INA219_ADDR);
    ina219.calibrate(0.1, 3.2);

    // ==== Interrupts ==== //
    gpio_set_irq_enabled_with_callback(motorA_out, GPIO_IRQ_EDGE_FALL, true, &gpio_ISR);
    gpio_set_irq_enabled(state_input, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(msc_input, GPIO_IRQ_EDGE_FALL, true);

    while(1) {
        printf("Testing!\n");
        now = get_absolute_time();
        int64_t time_ms = to_ms_since_boot(now);

        handleRelease();
        handleButton();
        handleMSCButton();
        getRPM();
        
        bat_per = getBatLevel();
        current_mA = ina219.read_current() * 1000;
        displacement = getRevolutions() * 0.5f;
        force = compute_force(FX29_read(MY_I2C, FX29_ADDR));

        if (MAF_counter == MAF_SZ) {
            MAF_counter = 0;
        }
        // Remove oldest value and add newest value to sum
        MAF_sum -= MAF[MAF_counter];
        MAF_sum += current_mA;
        MAF[MAF_counter] = current_mA;

        MAF_counter++;
        float MAF_current = MAF_sum * 1.0f/MAF_SZ;

        switch(state) {
            case WAIT: {
                enableMSC();
                setMotor(MOTOR_FW, MOTOR_OFF);
                nextState = STANDBY;
                break;
            }
            case STANDBY: {
                disableMSC();
                setMotor(MOTOR_FW, MOTOR_OFF);
                
                temp_speed = getInputSpeed();
                speed_lvl = uint16_t(temp_speed / 100.0f * 255.0f); 
                
                resetFiltering();
                displayState();
                nextState = CUTTING;
                break;
            }
            case CUTTING: {
                speed_lvl = speed_lvl;
                lp_current = LP_ALPHA * current_mA + (1 - LP_ALPHA) * lp_current;
                f_printf(&fil, "%s,%lld,%f,%f,%f,%f,%f,%f\n", stateToString().c_str(), time_ms, current_mA, lp_current, MAF_current, rpm, displacement, force);

                // ==== SAFETY CHECK ==== //
                if (getRevolutions() > fwRev) {
                    setMotor(MOTOR_BW, MOTOR_OFF);
                    state = REMOVAL;
                } else {
                    setMotor(MOTOR_FW, speed_lvl);
                }
                
                displayState();
                nextState = REMOVAL;
                break;
            }
            case REMOVAL: {
                setMotor(MOTOR_BW, 0);

                temp_speed = getInputSpeed();
                speed_lvl = int(temp_speed / 100.0f * 255.0f);
                resetFiltering();
                
                displayState();
                nextState = EXITING;
                break;
            }
            case EXITING: {
                speed_lvl = speed_lvl;

                lp_current = LP_ALPHA * current_mA + (1 - LP_ALPHA) * lp_current;
                f_printf(&fil, "%s,%lld,%f,%f,%f,%f,%f,%f\n", stateToString().c_str(), time_ms, current_mA, lp_current, MAF_current, rpm, displacement, force);

                // ==== SAFETY CHECK ==== //
                if (getRevolutions() < bwRev) {
                    setMotor(MOTOR_FW, MOTOR_OFF);
                    state = FINISH;
                } else {
                    setMotor(MOTOR_BW, speed_lvl);
                }
                
                displayState();
                nextState = FINISH;
                break;
            }
            case FINISH: {
                f_close(&fil);
                f_unmount("");

                setMotor(MOTOR_FW, MOTOR_OFF);

                displayState();

                nextState = STANDBY;
                break;
            }
            case ZERO: {
                setMotor(MOTOR_BW, MOTOR_ON);
                
                if (current_mA > CUTOFF_STALL) {
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
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void oled_init() {
    // External_vcc must be false for this to work
    oled.external_vcc = false;
    bool oled_status = ssd1306_init(
        &oled,
        128,
        64,
        OLED_ADDR,
        MY_I2C
    );

    if (oled_status) {
        ssd1306_clear(&oled);
        ssd1306_draw_string(&oled, 0, 2, 2, "RECORDS");
        ssd1306_draw_string(&oled, 0, 20, 1, "[ PRESS STA  :  NEW ]");
        ssd1306_draw_string(&oled, 0, 30, 1, "[ HOLD  STA  : ZERO ]");
        ssd1306_draw_string(&oled, 0, 45, 1, "V2.0 - 4/10/25");
        ssd1306_draw_string(&oled, 0, 55, 1, "Author: Thomas Chang");
        displayBat(0);
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
    f_printf(&fil, "State, Time(ms), Current(mA), CurrentLP(mA), CurrentMAF(mA), RPM, Displacement(mm), Force(N)\n");
}

void displayInputSpeed(int y_pos) {
    std::string temp = "INPUT SPEED: [ " + to_string(temp_speed) + "% ]";
    const char* in_speed = temp.c_str();
    ssd1306_draw_string(&oled, 0, y_pos, 1, in_speed);
}

void displayState() {
    ssd1306_clear(&oled);
    switch(state) {
            case STANDBY:
                ssd1306_draw_string(&oled, 0, 2, 2, "STANDBY");
                ssd1306_draw_string(&oled, 0, 20, 1, "[ PRESS STA  :  NEW ]");
                ssd1306_draw_string(&oled, 0, 30, 1, "[ PRESS MSC  : LOGS ]");
                displayBat(0);
                displayInputSpeed(50);
                break;
            
            case CUTTING:
                ssd1306_draw_string(&oled, 0, 2, 2, "CUTTING");
                displayBat(0);
                displayCurrent(20);
                displayRPM(30);
                displayPos(40);
                displayForce(50);
                break;
            
            case REMOVAL:
                ssd1306_draw_string(&oled, 0, 2, 2, "REMOVAL");
                displayBat(0);
                displayPos(20);
                displayInputSpeed(50);
                break;
            
            case EXITING:
                ssd1306_draw_string(&oled, 0, 2, 2, "EXITING");
                displayBat(0);
                displayCurrent(20);
                displayRPM(30);
                displayPos(40);
                displayForce(50);
                break;
            
            case FINISH: // For debugging purposes.
                ssd1306_draw_string(&oled, 0, 2, 2, "COMPLETE");
                displayBat(0);
                ssd1306_draw_string(&oled, 0, 20, 1, "[ PRESS STA  :  NEW ]");
                ssd1306_draw_string(&oled, 0, 30, 1, "[ PRESS MSC  : LOGS ]");
                break;
            case ZERO:
                ssd1306_draw_string(&oled, 0, 2, 2, "ZERO");
                displayBat(0);
                ssd1306_draw_string(&oled, 0, 20, 1, "Resetting to origin");
                displayCurrent(30);
                break;
        }
    ssd1306_show(&oled);
}

void displayForce(int y_pos) {
    std::string temp = "FRC (N)   : " + to_string(force);
    const char* frc = temp.c_str();
    ssd1306_draw_string(&oled, 0, y_pos, 1, frc);
}

void displayRPM(int y_pos) {
    std::string temp = "SPD (RPM) : " + to_string(rpm);
    const char* rpm = temp.c_str();
    ssd1306_draw_string(&oled, 0, y_pos, 1, rpm);
}

void displayCurrent(int y_pos) {
    std::string temp = "CUR (mA)  : " + to_string(current_mA);
    const char* curr = temp.c_str();
    ssd1306_draw_string(&oled, 0, y_pos, 1 , curr);
}

void displayPos(int y_pos) {
    std::string temp = "POS (mm)  : " + to_string(displacement);
    const char* pos = temp.c_str();
    ssd1306_draw_string(&oled, 0, y_pos, 1 , pos);
}

void displayBat(int y_pos) {
    std::string temp = to_string(bat_per) + "%";
    int x_pos = 110;
    if (bat_per == 0) {
        ssd1306_draw_string(&oled, 110, y_pos, 1, "BAT");
        ssd1306_draw_string(&oled, 116, y_pos + 8, 1, "LO");
        return;
    } else if (bat_per < 10) {
        x_pos = 116;
    }
    const char* bat = temp.c_str();
    ssd1306_draw_string(&oled, 110, y_pos, 1, "BAT");
    ssd1306_draw_string(&oled, x_pos, y_pos + 8, 1, bat);
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

float getBatLevel() {
    // 3120 corresponds to 3.2 Volts
    adc_select_input(0);
    long bat = adc_read();
    for (int i = 0; i < potIterations; i++) {
        bat += adc_read();
    }
    bat /= 1.0 * potIterations;
    bat = max(bat, BAT_MIN_ADC);
    bat = min(bat, BAT_MAX_ADC);
    bat = BAT_ADC_PER * bat + BAT_ADC_OFFSET;
    return bat;
}

long getInputSpeed() {
    adc_select_input(3);
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
    static absolute_time_t prevRPMTime = get_absolute_time();
    absolute_time_t currRPMTime = get_absolute_time();

    if (absolute_time_diff_us(prevRPMTime, currRPMTime) >= SEC_US) {
        rpm = (numPulses/12.0f/34.014f) * 60.0f;
        numPulses = 0;
        prevRPMTime = currRPMTime;
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
                gpio_put(DIR, 0);
                pwm_set_chan_level(slice, PWM_CHAN_A, 255);
                displayState();
                sleep_ms(SPIKE_TIME);

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

void setMotor(bool direction, uint16_t power) {
    gpio_put(DIR, direction);
    pwm_set_chan_level(slice, PWM_CHAN_A, power);
}

void enableMSC() {
    f_unmount("");
    tud_task();
}

void disableMSC() {
    tud_disconnect();
    f_mount(&filesys, "", 1);
    tud_deinit(BOARD_TUD_RHPORT);
}