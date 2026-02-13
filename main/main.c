//Compiler directives:
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

//ADC constants and global variables
#define CHANNEL_Mode    ADC_CHANNEL_6       // GPIO for Mode
#define CHANNEL_Timer   ADC_CHANNEL_5       // GPIO for Timer
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define BITWIDTH        ADC_BITWIDTH_12
#define DELAY_MS        25                  // Loop delay (ms)
adc_oneshot_unit_handle_t adc1_handle;      // ADC handle for Mode and Timer
int int_timer;                              // Keeping track of motor timer
int int_mode;                               // Keeping track of motor mode

//Motor constants and global variables 
#define LEDC_TIMER              LEDC_TIMER_0        //Set LEDC timer
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (15)                // GPIO connected to LEDC
#define LEDC_CHANNEL            LEDC_CHANNEL_0      // Set LEDC Channel
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT   // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (50)                // Set LEC Frequency         
#define degree_0                200
#define degree_90               585
float LEDC_DUTY = 200;

//LCD structure, including GPIOs
hd44780_t lcd = {
    .write_cb = NULL,
    .font = HD44780_FONT_5X8,
    .lines = 2,
    .pins = {
        .rs = GPIO_NUM_8,                           // GPIO for Register Select
        .e  = GPIO_NUM_3,                           // GPIO for enable
        .d4 = GPIO_NUM_9,                           // GPIO for data 4
        .d5 = GPIO_NUM_10,                          // GPIO for data 5
        .d6 = GPIO_NUM_11,                          // GPIO for data 6
        .d7 = GPIO_NUM_12,                          // GPIO for data 7
        .bl = HD44780_NOT_USED
    }
};

//Global boolean values
bool initial_message = true;
bool dSense = false;                //Driver Seat Sensor
bool dsbelt = false;                //Passenger Seat Sensor
bool pSense = false;                //Driver Seatbelt Sensor
bool psbelt = false;                //Passenger Seatbelt Sensor
bool engine = false;                //Engine state, false = off, true = on
bool hold = false;                  //used for ignitionPressed()
bool resume = true;                 //State of servo_task: resume/pause

//Function prototypes
static void pinConfig(void);        //Configure and set up the GPIOs
static void ADC_Config(void);       //Configure and set up ADC
static bool enable(void);           //Check 4 seat and seatbelt sensors
static bool ignitionPressed(void);  //Check if ignition button is pressed and released
static void servo_init(void);       //Initialize the motor analog output
void servo_task();                  //Task function for the servo

void app_main()
{
    TaskHandle_t xHandle;
    pinConfig();
    ADC_Config();
    servo_init();

    ESP_ERROR_CHECK(hd44780_init(&lcd)); //Initialize the LCD

    xTaskCreate(servo_task, "Servo Task", 2048, NULL, 5, &xHandle); //Start servo task

    while(1){
        bool ignitEn = ignitionPressed();
        if (!engine) {                  //Engine off State
            if (resume) {               //Clear LCD and delete servo task
                vTaskDelete (xHandle);
                resume = !resume;
                hd44780_clear (&lcd);   
            }

            bool ready = enable();      //Check if engine is ready to start

            if (dSense && initial_message){
                //Prints out the welcome message when the driver is seated.
                printf("Welcome to enhanced Alarm system model 218 -W25\n");
                initial_message = false;

            }

            if(ready){
                //Turn on greenlight if the engine is ready
                gpio_set_level(greenLED_PIN, 1);
            }
            else {//Turn off greenlight otherwise
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
                    vTaskDelay (3000/ portTICK_PERIOD_MS); //3 Seconds delay
                }
            }

            else {//Turn off alarm after 3 seconds
                gpio_set_level (Alarm, 0);
            }
        }

        else {  
            //If the engine is started
            if (ignitEn) {
                //Turn off engine if ignite is pressed again.
                gpio_set_level (redLED_PIN, 0);
                printf("Stopping the engine.\n");
                engine = false;
            }
            else {
                if (!resume) {  //Start servo task
                    xTaskCreate(servo_task, "Servo Task", 2048, NULL, 5, &xHandle);
                    resume = !resume;
                }
                
                //Read Mode and Timer inputs
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
    //Reset pins
    gpio_reset_pin(greenLED_PIN);
    gpio_reset_pin(redLED_PIN);
    gpio_reset_pin(ignitionButton);
    gpio_reset_pin(driveSeatBelt);
    gpio_reset_pin(passengerSeatBelt);
    gpio_reset_pin(driveSeatSense);
    gpio_reset_pin(passengerSeatSense);
    gpio_reset_pin(Alarm);
    gpio_reset_pin(LCD_bright);

    //Set directions
    gpio_set_direction(greenLED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(redLED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(Alarm, GPIO_MODE_OUTPUT);
    gpio_set_direction(ignitionButton, GPIO_MODE_INPUT);
    gpio_set_direction(driveSeatBelt, GPIO_MODE_INPUT);
    gpio_set_direction(passengerSeatBelt, GPIO_MODE_INPUT);
    gpio_set_direction(driveSeatSense, GPIO_MODE_INPUT);
    gpio_set_direction(passengerSeatSense, GPIO_MODE_INPUT);

    //Configure pullup/down
    gpio_pullup_en(ignitionButton);
    gpio_pullup_en(driveSeatBelt);
    gpio_pullup_en(driveSeatSense);
    gpio_pullup_en(passengerSeatBelt);
    gpio_pullup_en(passengerSeatSense);

    //Initial levels
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
        .duty           = 200, 
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
//These variables are used to compare current and past states
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
    //Slowly move the servo back to 0 degree when the engine is started
    
    int timer_delay = 1000;     //Initial time delay
    while (1){
        if (engine) {
        
            if (int_timer < 1365) {
                timer_delay = 5000;
                timer = 5;
            }//Set timer to long delay

            else if (int_timer < 2730) {
                timer_delay = 3000;
                timer = 3;
            }//Set timer to medium delay

            else {
                timer_delay = 1000;
                timer = 1;
            }//Set timer to short delay


            if (int_mode < 1024) {//Off mode
                mode = 0;
                if (mode!= pastmode)    hd44780_clear (&lcd);  
                hd44780_gotoxy(&lcd, 0, 0); 
                hd44780_puts(&lcd, "MODE: OFF");
                LEDC_DUTY = 200;
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            }

            else if (int_mode < 2047) {//Low speed mode
                mode = 1;
                if (mode!= pastmode)    hd44780_clear (&lcd);    
                hd44780_gotoxy(&lcd, 0, 0); 
                hd44780_puts(&lcd, "MODE: LO");
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

            else if (int_mode < 3071) {//High speed mode
                mode = 2;
                if (mode!= pastmode)    hd44780_clear (&lcd);   
                hd44780_gotoxy(&lcd, 0, 0); 
                hd44780_puts(&lcd, "MODE: HI");
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
            
            else {//Intermittent mode, low speed + delay
                mode = 3;
                if (mode!= pastmode || timer != pasttimer)    hd44780_clear (&lcd);   
                hd44780_gotoxy(&lcd, 0, 0); 
                hd44780_puts(&lcd, "MODE: INT");
                if (timer_delay == 1000) {
                    hd44780_gotoxy(&lcd, 0, 1); 
                    hd44780_puts(&lcd, "SHORT");
                }
                if (timer_delay == 3000) {
                    hd44780_gotoxy(&lcd, 0, 1); 
                    hd44780_puts(&lcd, "MEDIUM");
                }
                if (timer_delay == 5000) {
                    hd44780_gotoxy(&lcd, 0, 1); 
                    hd44780_puts(&lcd, "LONG");
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

        if (mode != pastmode || timer != pasttimer) {//Update past mode and timer
                pastmode = mode;
                timer = pasttimer;
        }

        vTaskDelay (20/ portTICK_PERIOD_MS);//Debounce delay
    }
}