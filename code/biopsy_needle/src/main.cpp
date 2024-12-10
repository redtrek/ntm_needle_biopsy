#include <iostream>
#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "stdlib.h"

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
#define speed_input 26   // A0

#define debounce_us   50000
#define potIterations 1000

#define PWM         0     // TX
#define DIR         6     // D4

#define motorA_out  25    // 25
#define motorB_out  24    // 24

using namespace std;

// ==== Experimental Values ==== //
int fwRev = 60;
int bwRev = -60;

// ==== Function Prototypes ==== //
void init_i2c0();
void oled_init();
void sd_test();
void displayState();
void displayInputSpeed();
void buttonHandler();
long getInputSpeed();
float getRevolutions();
float getRPM();
void displayRPM();
void displayCurrent();
const char* string();

// ==== Globals ==== //
ssd1306_t oled;
uint16_t speed_lvl = 0;
long temp_speed = 0;
bool countA_flag = false;
int count = 0;
int numPulses = 0;
absolute_time_t prevTime = get_absolute_time();
float rpm = 0;
float current = 0;

// ==== Motor Operation State Machine ==== //
enum states {
    WAIT,
    STANDBY,
    CUTTING,
    REMOVAL,
    EXITING
};
enum states state, nextState;

void motor_ISR(uint gpio, uint32_t events) {
    countA_flag = true;
    if (gpio_get(motorB_out)) {
        count--;
    } else if(gpio_get(motorB_out) == 0) {
        count++;
    }
    numPulses++;
}

FATFS filesys;
FRESULT fr = f_mount(&filesys, "", 1);
FIL fil;

int main() {

    // Initialize serial port and wait for serial monitor
    stdio_init_all();
    sleep_ms(500);


    // ==== Pin Initialization ==== //
    adc_init();
    adc_gpio_init(speed_input);
    gpio_set_dir(speed_input, GPIO_IN);

    gpio_init(state_input);
    gpio_set_dir(state_input, GPIO_IN);
    gpio_pull_down(state_input);

    gpio_set_function(PWM, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM);
    uint channel = pwm_gpio_to_channel(PWM);
    pwm_set_clkdiv(slice, 48.83f);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_enabled(slice, true);
    gpio_init(DIR);
    gpio_set_dir(DIR, GPIO_OUT);

    gpio_init(motorB_out);
    gpio_set_dir(motorB_out, GPIO_IN);

    // ==== General Initialization ==== //
    oled_init();
    //sd_test();
    
    state = WAIT;
    nextState = STANDBY;

    INA219 ina219(i2c0, INA219_ADDR);
    ina219.I2C_START(I2C0_SDA, I2C0_SCL, 400);
    ina219.calibrate(0.1, 2);

    // Interrupt for encoder_outputA
    gpio_set_irq_enabled_with_callback(motorA_out, GPIO_IRQ_EDGE_FALL, true, &motor_ISR);

    const char* const filename = "demo.txt";

    while(1) {

        float revolutions = getRevolutions();
        //printf("Count %d\n", count);
        //printf("Number of revolutions %f\n", revolutions);

        rpm = getRPM();
        printf("RPM: %f \n", rpm);
        current = ina219.read_current();
        float voltage = ina219.read_voltage();
        //printf("Voltage %.4f V\n", voltage);
        //printf("Current %.4f A\n", current);
        //printf("Analog value %d\n", uint16_t(getInputSpeed() / 100.0f * 255));
        

        buttonHandler();
        switch(state) {
            case WAIT:
                gpio_put(DIR, 1);
                pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                
                nextState = STANDBY;
                break;

            case STANDBY:
                fr = f_close(&fil);
                //f_unmount("");
                fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);

                gpio_put(DIR, 1);
                pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                temp_speed = getInputSpeed();
                speed_lvl = uint16_t(temp_speed / 100.0f * 255);

                displayState();
                nextState = CUTTING;
                break;
            
            case CUTTING:
                f_printf(&fil, "Current: %f\n", current);
                f_printf(&fil, "RPM: %f\n", rpm);

                displayState();
                nextState = REMOVAL;

                // ==== SAFETY CHECK ==== //
                if (revolutions > fwRev) {
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
                speed_lvl = uint16_t(temp_speed / 100.0f * 255);
                
                displayState();
                nextState = EXITING;
                break;
            
            case EXITING:
                f_printf(&fil, "Current: %f\n", current);
                f_printf(&fil, "RPM: %f\n", rpm);

                gpio_put(DIR, 0);
                pwm_set_chan_level(slice, PWM_CHAN_A, speed_lvl);
                
                displayState();
                nextState = STANDBY;

                // ==== SAFETY CHECK ==== //
                if (revolutions < bwRev) {
                    gpio_put(DIR, 1);
                    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
                    state = STANDBY;
                } else {
                    gpio_put(DIR, 0);
                    pwm_set_chan_level(slice, PWM_CHAN_A, speed_lvl);
                }

                break;
            
        }
        // Clear terminal 
        //printf("\033[1;1H\033[2J");
        sleep_ms(100);
    }
    return 0;
}

void init_i2c0() {
    i2c_init(i2c0, 400000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);
}

void oled_init() {
    // Initialize I2C communication
    init_i2c0();

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

void buttonHandler() {
    int button = gpio_get(state_input);
    if (button) {
        absolute_time_t start = get_absolute_time();
        absolute_time_t current;

        while (1) {
           
            current = get_absolute_time();
            if (absolute_time_diff_us(start, current) >= debounce_us) {
                button = gpio_get(state_input);
                if (button) {
                    while(button) {
                        button = gpio_get(state_input);
                        sleep_ms(10);
                    }
                    state = nextState;
                    std::string temp = "State: " + stateToString() + ":\n";
                    const TCHAR* name = temp.c_str();
                    f_printf(&fil, name);
                    
                }
                break;
            } 
            
            sleep_ms(10);
        }
    }
}

long getInputSpeed() {
    long val = 0;
    for (int i = 0; i < potIterations; i++) {
        val += adc_read();
    }
    
    return 0.0243 * (val / potIterations) + 1;
}

float getRevolutions() {
    float re = count/12.0/34.014f;
    return re;
}

float getRPM() {
    //printf("Current numPulses, %d\n", numPulses);
    absolute_time_t currTime = get_absolute_time();
    printf("Current time: %lld us\n", currTime);
    printf("Prev time: %lld us\n", prevTime);

    if (absolute_time_diff_us(prevTime, currTime) >= 1000000) {
        printf("this point has been reached! %d", currTime);
        
        rpm = (numPulses/12.0f/34.014f) * 60.0f;
        numPulses = 0;
        prevTime = currTime;
        printf("Current rpm, %d\n", rpm);
    }
    return rpm;
}

