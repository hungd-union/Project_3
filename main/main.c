#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include <sdkconfig.h>
#include <stdbool.h>
#include "esp_adc/adc_oneshot.h"
#include "math.h"
#include "driver/ledc.h"

//Pin number declaration
#define redLED_PIN          4  
#define greenLED_PIN        5   
#define driveSeatBelt       16
#define passengerSeatBelt   17 
#define Alarm               2
#define ignitionButton      1
#define driveSeatSense      37
#define passengerSeatSense  36
#define LCD_bright          18

//ADC constants
#define CHANNEL_Mode    ADC_CHANNEL_6
#define CHANNEL_Timer   ADC_CHANNEL_5
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define BITWIDTH        ADC_BITWIDTH_12
#define DELAY_MS        25                  // Loop delay (ms)
adc_oneshot_unit_handle_t adc1_handle;      // ADC for Mode
int int_timer; 
int int_mode; 

//Motor Variables 
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO         (15)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT   // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (50)                // Frequency in Hertz.
#define degree_0                200
#define degree_90               585
float LEDC_DUTY = 200;

hd44780_t lcd = {
    .write_cb = NULL,
    .font = HD44780_FONT_5X8,
    .lines = 2,
    .pins = {
        .rs = GPIO_NUM_8,
        .e  = GPIO_NUM_3,
        .d4 = GPIO_NUM_9,
        .d5 = GPIO_NUM_10,
        .d6 = GPIO_NUM_11,
        .d7 = GPIO_NUM_12,
        .bl = HD44780_NOT_USED
    }
};

//Global boolean values
bool initial_message = true;
bool dSense = false;
bool dsbelt = false;
bool pSense = false;
bool psbelt = false;
bool engine = false;
bool hold = false;
bool resume = true;

//Function prototypes
static void pinConfig(void);
static void ADC_Config(void);
static bool enable(void);
static bool ignitionPressed(void);
static void servo_init(void);
void servo_task();

void app_main()
{
    TaskHandle_t xHandle;
    pinConfig();
    ADC_Config();
    servo_init();

    ESP_ERROR_CHECK(hd44780_init(&lcd));

    xTaskCreate(servo_task, "Servo Task", 2048, NULL, 5, &xHandle);

    while(1){
        bool ignitEn = ignitionPressed();
        if (!engine) {
            if (resume) {
                vTaskDelete (xHandle);
                resume = !resume;
                hd44780_clear (&lcd);   
            }
            //Check if the engine is not started.
            bool ready = enable();

            if (dSense && initial_message){
                //Prints out the welcome message when the driver is seated.
                printf("Welcome to enhanced Alarm system model 218 -W25\n");
                initial_message = false;

            }

            if(ready){
                //Turn on greenlight if the engine is ready
                gpio_set_level(greenLED_PIN, 1);
            }
            else {
                gpio_set_level(greenLED_PIN, 0);
            }

            if(ignitEn){
                //Turn on the engine if ready and ignite pressed.
                if (ready) {
                    printf("Starting the engine.\n");
                    gpio_set_level(greenLED_PIN, 0);
                    gpio_set_level(redLED_PIN, 1);
                    engine = true;
                }

                else {
                    //Prints error and raise alarm otherwise.
                    gpio_set_level (Alarm, 1);

                    if (!dSense){
                    printf("Driver seat not occupied\n");
                    }
                
                    if (!dsbelt){
                    printf("Driver seatbelt not fastened\n");
                    }

                    if (!pSense){
                    printf("Passenger seat not occupied\n");
                    }

                    if (!psbelt){
                    printf("Passenger seatbelt not fastened\n");
                    }
                    vTaskDelay (3000/ portTICK_PERIOD_MS);
                }
            }

            else {
                gpio_set_level (Alarm, 0);
            }
        }

        else {  
            //If the engine is started
            if (ignitEn) {
                //Turn off engine is ignite is pressed again.
                gpio_set_level (redLED_PIN, 0);
                printf("Stopping the engine.\n");
                engine = false;
            }
            else {
                if (!resume) {
                    xTaskCreate(servo_task, "Servo Task", 2048, NULL, 5, &xHandle);
                    resume = !resume;
                }
                
                adc_oneshot_read
                (adc1_handle, CHANNEL_Mode, &int_mode); 

                adc_oneshot_read
                (adc1_handle, CHANNEL_Timer, &int_timer);
            }
        }
        vTaskDelay (DELAY_MS/ portTICK_PERIOD_MS);
    }
}

//Configure the GPIO
void pinConfig(void){
    gpio_reset_pin(greenLED_PIN);
    gpio_reset_pin(redLED_PIN);
    gpio_reset_pin(ignitionButton);
    gpio_reset_pin(driveSeatBelt);
    gpio_reset_pin(passengerSeatBelt);
    gpio_reset_pin(driveSeatSense);
    gpio_reset_pin(passengerSeatSense);
    gpio_reset_pin(Alarm);
    gpio_reset_pin(LCD_bright);

    gpio_set_direction(greenLED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(redLED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(Alarm, GPIO_MODE_OUTPUT);
    gpio_set_direction(ignitionButton, GPIO_MODE_INPUT);
    gpio_set_direction(driveSeatBelt, GPIO_MODE_INPUT);
    gpio_set_direction(passengerSeatBelt, GPIO_MODE_INPUT);
    gpio_set_direction(driveSeatSense, GPIO_MODE_INPUT);
    gpio_set_direction(passengerSeatSense, GPIO_MODE_INPUT);

    gpio_pullup_en(ignitionButton);
    gpio_pullup_en(driveSeatBelt);
    gpio_pullup_en(driveSeatSense);
    gpio_pullup_en(passengerSeatBelt);
    gpio_pullup_en(passengerSeatSense);

    gpio_set_level(greenLED_PIN, 0);
    gpio_set_level(redLED_PIN, 0);
    gpio_set_level(Alarm, 0);

}

//Configure the ADC
void ADC_Config (void) {
    // Unit configuration
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };                                                
    adc_oneshot_new_unit(&init_config1, &adc1_handle);  

    // Channel configuration
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  
    adc_oneshot_config_channel (adc1_handle, CHANNEL_Mode, &config);
    adc_oneshot_config_channel (adc1_handle, CHANNEL_Timer, &config);
}

//Configure LEDC
static void servo_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

/*Check the seat and seatbelt sensor
to see if the engine is ready.*/
bool enable(void){
    bool dslvl = gpio_get_level(driveSeatSense);
    bool dsbeltlvl = gpio_get_level(driveSeatBelt);
    bool pslvl = gpio_get_level(passengerSeatSense);
    bool psbltlvl = gpio_get_level(passengerSeatBelt);

    if (!dslvl){
        dSense = true; //Driver sensor
    }
    else{dSense = false;}

    if (!dsbeltlvl){
        dsbelt = true; // driver seatbelt sensor
    }
    else{dsbelt = false;}

    if (!pslvl){
        pSense = true; // passenger seat level
    }
    else{pSense = false;}

    if (!psbltlvl){
        psbelt = true; // passenger seatbelt level
    }
    else{psbelt = false;}

    bool IgnitReady = dSense && dsbelt && pSense && psbelt;
    return IgnitReady;
}


/*Check if the Ignition button is pressed 
(only return true after a hold and release)*/
bool ignitionPressed (void) {
    bool igniteHold = gpio_get_level(ignitionButton) == 0;
    if (igniteHold) {
        hold = true;
    }
    if (hold && !igniteHold)
    {
        hold = false;
        return true;
    }
    return false;
}

int mode=0;
int pastmode=0;
int timer=0;
int pasttimer=0;
void servo_task (void *pvParameter) {
    while (LEDC_DUTY != 200) {
        LEDC_DUTY -= 2.567;
        if (LEDC_DUTY < 200) {
            LEDC_DUTY = 200;
        }
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(10 /portTICK_PERIOD_MS);
    }
    
    int timer_delay = 1000;
    while (1){
        if (engine) {
        
            if (int_timer < 1365) {
                timer_delay = 5000;
                timer = 5;
            }

            else if (int_timer < 2730) {
                timer_delay = 3000;
                timer = 3;
            }

            else {
                timer_delay = 1000;
                timer = 1;
            }


            if (int_mode < 1024) {
                mode = 0;
                if (mode!= pastmode)    hd44780_clear (&lcd);  
                hd44780_gotoxy(&lcd, 0, 0); 
                hd44780_puts(&lcd, "\x08 MODE: OFF");
                LEDC_DUTY = 200;
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            }

            else if (int_mode < 2047) {
                mode = 1;
                if (mode!= pastmode)    hd44780_clear (&lcd);    
                hd44780_gotoxy(&lcd, 0, 0); 
                hd44780_puts(&lcd, "\x08 MODE: LO");
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                while (LEDC_DUTY != 585) {
                    LEDC_DUTY += 2.567;
                    if (LEDC_DUTY > 585) {
                        LEDC_DUTY = 585;
                    }
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(10 /portTICK_PERIOD_MS);
                }
                while (LEDC_DUTY != 200) {
                    LEDC_DUTY -= 2.567;
                    if (LEDC_DUTY < 200) {
                            LEDC_DUTY = 200;
                    }
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(10 /portTICK_PERIOD_MS);
                }
            }

            else if (int_mode < 3071) {
                mode = 2;
                if (mode!= pastmode)    hd44780_clear (&lcd);   
                hd44780_gotoxy(&lcd, 0, 0); 
                hd44780_puts(&lcd, "\x08 MODE: HI");
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                while (LEDC_DUTY != 585) {
                    LEDC_DUTY += 6.4167;
                    if (LEDC_DUTY > 585) {
                        LEDC_DUTY = 585;
                    }
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(10 /portTICK_PERIOD_MS);
                }
                while (LEDC_DUTY != 200) {
                    LEDC_DUTY -= 6.4167;
                    if (LEDC_DUTY < 200) {
                        LEDC_DUTY = 200;
                    }
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(10 /portTICK_PERIOD_MS);
                }
            }
            
            else {
                mode = 3;
                if (mode!= pastmode || timer != pasttimer)    hd44780_clear (&lcd);   
                hd44780_gotoxy(&lcd, 0, 0); 
                hd44780_puts(&lcd, "\x08 MODE: INT");
                if (timer_delay == 1000) {
                    hd44780_gotoxy(&lcd, 0, 1); 
                    hd44780_puts(&lcd, "\x08 SHORT");
                }
                if (timer_delay == 3000) {
                    hd44780_gotoxy(&lcd, 0, 1); 
                    hd44780_puts(&lcd, "\x08 MEDIUM");
                }
                if (timer_delay == 5000) {
                    hd44780_gotoxy(&lcd, 0, 1); 
                    hd44780_puts(&lcd, "\x08 LONG");
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                while (LEDC_DUTY != 585) {
                    LEDC_DUTY += 2.567;
                    if (LEDC_DUTY > 585) {
                        LEDC_DUTY = 585;
                    }
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(10 /portTICK_PERIOD_MS);
                }
                while (LEDC_DUTY != 200) {
                    LEDC_DUTY -= 2.567;
                    if (LEDC_DUTY < 200) {
                        LEDC_DUTY = 200;
                    }
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(10 /portTICK_PERIOD_MS);
                }
                vTaskDelay(timer_delay / portTICK_PERIOD_MS);
            }
        }

        else {
            while (LEDC_DUTY != 200) {
                LEDC_DUTY -= 6.4167;
                if (LEDC_DUTY < 200) {
                    LEDC_DUTY = 200;
                }
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(10 /portTICK_PERIOD_MS);
            }
        }

        if (mode != pastmode || timer != pasttimer) {
                pastmode = mode;
                timer = pasttimer;
        }

        vTaskDelay (20/ portTICK_PERIOD_MS);
    }
}