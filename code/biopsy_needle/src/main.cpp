/**
 * @file main.cpp
 * @author Thomas Chang
 * @brief This is the main file for the smart biopsy needle. It handles the high level operation of the state machine, sensors, motors, and display.
 * If onboarding, LOOK AT THIS FILE FIRST. Close all pragma regions and just try to understand the main function.
 * @version 1.0
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "include/ntm_helpers.h"

// Peripheral Devices
#include "../libs/INA219/INA219.h"
#include "../libs/SSD1306/ssd1306.h"
#include "../libs/FX29/fx29.h"

using namespace std;

// ==== Experimental Values ==== //
int fwRev = 50;             ///< Sets the max forward revolutions of the device from start position. Current ratio of (revolutions : distance) = (2 : 1)
int bwRev = 0;              ///< Sets the max backwards revolutions of the device from start position. Current ratio of (revolutions : distance) = (2 : 1)
#define MAF_SZ      5       ///< Window size for Moving Average Filter.
#define LP_ALPHA    0.1f    ///< Smoothing factor for LP Filter

// ==== Function Prototypes ==== //
#pragma region LOCAL PROTOTYPES

void oled_init();
void displayInputSpeed(int y_pos);
void displayBat(int y_pos);
void displayState();
void handleButton();
void handleRelease();
void handleMSCButton();
void createDataFile();
void resetFiltering();
float getBatLevel();
long getInputSpeed();
void getRPM();
void enableMSC();
void disableMSC();
void testingSuite();
void testMSC();
void testSD();
void testPins();

#pragma endregion

// ==== Globals ==== //
#pragma region GLOBALS

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

extern uint slice;
extern uint channel;

#pragma endregion

// ==== Debugging ==== //
absolute_time_t prevBug = get_absolute_time();
absolute_time_t currBug = get_absolute_time();

// ==== Flags ==== //
#pragma region FLAGS

volatile bool motor_flag = false;
volatile bool button_press_flag = false;
volatile bool button_release_flag = false;
volatile bool button_msc_flag = false;

#pragma endregion

// ==== Data Logging ==== //
FATFS filesys;
FIL fil;
TCHAR filename[20] = "data0.csv\0";

// ==== Motor State Machine ==== //
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
        if (gpio_get(MOTOR_DIR) == 0) {
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
    #pragma region TESTING

    //testingSuite();
    //testMSC();
    //testSD();
    //testSPI();

    #pragma endregion

    // MSC: Initialize board
    board_init();
    tud_init(BOARD_TUD_RHPORT);

    // Initialize serial port and wait for serial monitor
    stdio_init_all();
    gpio_init(MOTOR_PWM);
    gpio_put(MOTOR_PWM, 0);
    sleep_ms(1000);
    
    // ==== General Initialization ==== //
    board_gpio_init();
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
        now = get_absolute_time();
        int64_t time_ms = to_ms_since_boot(now);

        handleRelease();
        handleButton();
        handleMSCButton();
        getRPM();
        
        bat_per = getBatLevel();
        current_mA = ina219.read_current() * 1000;
        displacement = getRevolutions(count) * 0.5f;
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
                f_printf(&fil, "%s,%lld,%f,%f,%f,%f,%f,%f\n", "CUTTING", time_ms, current_mA, lp_current, MAF_current, rpm, displacement, force);

                // ==== SAFETY CHECK ==== //
                if (getRevolutions(count) > fwRev) {
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
                f_printf(&fil, "%s,%lld,%f,%f,%f,%f,%f,%f\n", "EXITING", time_ms, current_mA, lp_current, MAF_current, rpm, displacement, force);

                // ==== SAFETY CHECK ==== //
                if (getRevolutions(count) < bwRev) {
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

#pragma region LOCAL DEFINITIONS

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
        ssd1306_draw_string(&oled, 0, 45, 1, "V1.0 - 5/6/25");
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
                displayData(&oled, 20, current_mA, "CUR (mA)  : ");
                displayData(&oled, 30, rpm, "SPD (RPM) : ");
                displayData(&oled, 40, displacement, "POS (mm)  : ");
                displayData(&oled, 50, force, "FRC (N)   : ");
                break;
            
            case REMOVAL:
                ssd1306_draw_string(&oled, 0, 2, 2, "REMOVAL");
                displayBat(0);
                displayData(&oled, 20, displacement, "POS (mm)  : ");
                displayInputSpeed(50);
                break;
            
            case EXITING:
                ssd1306_draw_string(&oled, 0, 2, 2, "EXITING");
                displayBat(0);
                displayData(&oled, 20, current_mA, "CUR (mA)  : ");
                displayData(&oled, 30, rpm, "SPD (RPM) : ");
                displayData(&oled, 40, displacement, "POS (mm)  : ");
                displayData(&oled, 50, force, "FRC (N)   : ");
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
                displayData(&oled, 30, current_mA, "CUR (mA)  : ");
                break;
        }
    ssd1306_show(&oled);
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

float getBatLevel() {
    adc_select_input(0);
    long bat = adc_read();
    for (int i = 0; i < potIterations; i++) {
        bat += adc_read();
    }
    bat /= 1.0 * potIterations;
    bat = NTM_MAX(bat, BAT_MIN_ADC);
    bat = NTM_MIN(bat, BAT_MAX_ADC);
    bat = (bat - BAT_MIN_ADC) * BAT_ADC_PER;
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
                gpio_put(MOTOR_DIR, 0);
                pwm_set_chan_level(slice, channel, 255);
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

void enableMSC() {
    f_unmount("");
    tud_task();
}

void disableMSC() {
    tud_disconnect();
    f_mount(&filesys, "", 1);
    tud_deinit(BOARD_TUD_RHPORT);
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
    uint slice = pwm_gpio_to_slice_num(MOTOR_PWM);
    uint channel = pwm_gpio_to_channel(MOTOR_PWM);
    gpio_set_function(MOTOR_PWM, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, 48.83f);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_enabled(slice, true);
    gpio_init(MOTOR_DIR);
    gpio_set_dir(MOTOR_DIR, GPIO_OUT);

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

#pragma endregion