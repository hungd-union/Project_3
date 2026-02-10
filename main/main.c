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

//Pin number declaration
#define redLED_PIN          4  
#define greenLED_PIN        5   
#define driveSeatBelt       16
#define passengerSeatBelt   17 
#define Alarm               1
#define ignitionButton      2
#define driveSeatSense      38
#define passengerSeatSense  37
#define LCD_bright          18

//ADC constants
#define CHANNEL_Mode    ADC_CHANNEL_6
#define CHANNEL_Timer   ADC_CHANNEL_5
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define BITWIDTH        ADC_BITWIDTH_12
#define DELAY_MS        20               // Loop delay (ms)
#define NUM_SAMPLES     1000                // Number of samples


//Global boolean values
bool initial_message = true;
bool dSense = false;
bool dsbelt = false;
bool pSense = false;
bool psbelt = false;
bool engine = false;
bool hold = false;


static uint32_t get_time_sec()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec;
}

static const uint8_t char_data[] =
{
    0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00,
    0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x1f, 0x00
};

void lcd_test(void *pvParameters)
{
    hd44780_t lcd =
    {
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

    ESP_ERROR_CHECK(hd44780_init(&lcd));

    hd44780_upload_character(&lcd, 0, char_data);
    hd44780_upload_character(&lcd, 1, char_data + 8);

    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, "\x08 Hello, World!");
    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, "\x09 ");

    char time[16];

    while (1)
    {
        hd44780_gotoxy(&lcd, 2, 1);

        snprintf(time, 7, "%" PRIu32 "  ", get_time_sec());
        time[sizeof(time) - 1] = 0;

        hd44780_puts(&lcd, time);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main()
{
    xTaskCreate(lcd_test, "lcd_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

/**
 * returns a boolean determining whether all of the car alarms systems have been satisifed ie:
 * driver seat belt, driver is seated etc.
 */
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

    /**
     * Will configure all of the pins within the design resetting all of the pins within the design to a known state
     * setting the direction to either input or output 
     * enabling pullup resistors within the ESP
     * And finally setting all of the output pins to zero
     */

//Check if the Ignition button is pressed (only return true after a hold and release)
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

//Pin configuration function, setting up mode, pullup/down
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