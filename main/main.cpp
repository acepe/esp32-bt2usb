/*
ESP32-BT2USB
Software for Espressif ESP32-S3 (and others with USB-OTG Support) for Bluetooth BLE HID keyboard to USB interfacing.
Based on esp32-BT2PS2 (https://github.com/Hamberthm/esp32-bt2ps2) by Humberto Möckel. Adapted by André Ackermann for USB interface instead of PS2.
Thanks to Humberto Möckel and all the pioneers who worked on the code libraries that made this project possible, check them on the README!
Copyright Humberto Möckel - Hamcode - 2023 (hamberthm@gmail.com), André Ackermann - 2024,
*/

#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "led_strip.h"
#include "nvs_flash.h"

#include "../include/globals.hpp"
#include "../include/bt_keyboard.hpp"

#define BLINK_GPIO 48

static constexpr char const *TAG = "BTKeyboard";

// BTKeyboard section
BTKeyboard bt_keyboard;

int NumDigits(int x) // Returns number of digits in keyboard code
{
    x = abs(x);
    return (x < 10 ? 1 : (x < 100 ? 2 : (x < 1000 ? 3 : (x < 10000 ? 4 : (x < 100000 ? 5 : (x < 1000000 ? 6 : (x < 10000000 ? 7 : (x < 100000000 ? 8 : (x < 1000000000 ? 9 : 10)))))))));
}

void pairing_handler(uint32_t pid)
{
    int x = (int)pid;
    std::cout << "Please enter the following pairing code, "
              << std::endl
              << "followed by ENTER on your keyboard: "
              << pid
              << std::endl;

    for (int i = 0; i < 10; i++) // Flash quickly many times to alert user of incoming code display
    {
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    int dig = NumDigits(x); // How many digits does our code have?

    for (int i = 1; i <= dig; i++)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGW(TAG, "PAIRING CODE (TYPE ON KEYBOARD AND PRESS ENTER): %d ", pid);
        int flash = ((int)((pid / pow(10, (dig - i)))) % 10); // This extracts one ditit at a time from our code
        ESP_LOGI(TAG, "Flashing %d times", flash);
        for (int n = 0; n < flash; n++) // Flash the LED as many times as the digit
        {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(GPIO_NUM_2, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        if (flash < 1) // If digit is 0, keep a steady light for 1.5sec
        {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            gpio_set_level(GPIO_NUM_2, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (int i = 0; i < 10; i++) // Quick flashes indicate end of code display
    {

        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

extern "C" {
    
    static led_strip_handle_t led_strip;

    static void blink_led(uint8_t state) {
        /* If the addressable LED is enabled */
        if (state) {
            ESP_LOGD(TAG, "LED on");
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            led_strip_set_pixel(led_strip, 0, 64, 0, 64);
            /* Refresh the strip to send data */
            led_strip_refresh(led_strip);
        } else {
            ESP_LOGD(TAG, "LED off");
            /* Set all LED off to clear all pixels */
            led_strip_clear(led_strip);
        }
    }

    static void init_led(void) {
        ESP_LOGI(TAG, "On board adressable LED initialisation");
        /* LED strip initialization with the GPIO and pixels number*/
        led_strip_config_t strip_config = {
            .strip_gpio_num = BLINK_GPIO,
            .max_leds = 1,  // one LED on board
        };
        led_strip_rmt_config_t rmt_config = {
            .resolution_hz = 10 * 1000 * 1000,  // 10MHz
        };
        ESP_ERROR_CHECK(
            led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
        ESP_LOGI(TAG, "On board adressable LED initialisation DONE");
    }

    /************* TinyUSB descriptors ****************/

    #define TUSB_DESC_TOTAL_LEN \
    (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

    /**
     * @brief HID report descriptor
     *
     * In this example we implement Keyboard + Mouse HID device,
     * so we must define both report descriptors
     */
    const uint8_t hid_report_descriptor[] = {
        TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD))};

    /**
     * @brief String descriptor
     */
    const char *hid_string_descriptor[5] = {
        // array of pointer to string descriptors
        (char[]){0x09, 0x04},     // 0: is supported language is English (0x0409),
                                // 0x0407 is for german
        "TinyUSB",                // 1: Manufacturer
        "TinyUSB Device",         // 2: Product
        "123456",                 // 3: Serials, should use chip ID
        "Example HID interface",  // 4: HID
    };

    /**
     * @brief Configuration descriptor
     *
     * This is a simple configuration descriptor that defines 1 configuration and 1
     * HID interface
     */
    static const uint8_t hid_configuration_descriptor[] = {
        // Configuration number, interface count, string index, total length,
        // attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN,
                            TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

        // Interface number, string index, boot protocol, report descriptor len, EP
        // In address, size & polling interval
        TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16,
                        10),
    };

    /********* TinyUSB HID callbacks ***************/

    // Invoked when received GET HID REPORT DESCRIPTOR request
    // Application return pointer to descriptor, whose contents must exist long
    // enough for transfer to complete
    uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
        // We use only one interface and one HID report descriptor, so we can ignore
        // parameter 'instance'
        return hid_report_descriptor;
    }

    // Invoked when received GET_REPORT control request
    // Application must fill buffer report's content and return its length.
    // Return zero will cause the stack to STALL request
    uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type, uint8_t *buffer,
                                uint16_t reqlen) {
        (void)instance;
        (void)report_id;
        (void)report_type;
        (void)buffer;
        (void)reqlen;

        return 0;
    }

    // Invoked when received SET_REPORT control request or
    // received data on OUT endpoint ( Report ID = 0, Type = 0 )
    void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type, uint8_t const *buffer,
                            uint16_t bufsize) {}

    
    void reset_report(hid_keyboard_report_t *report) {
        report->modifier = 0;  // Reset modifiers
        for (int i = 0; i < sizeof(report->keycode); i++) {
            report->keycode[i] = 0;
        }
    }

    void init_usb() {
        ESP_LOGI(TAG, "USB initialization");
        const tinyusb_config_t tusb_cfg = {
            .device_descriptor = NULL,
            .string_descriptor = hid_string_descriptor,
            .string_descriptor_count =
                sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
            .external_phy = false,
            .configuration_descriptor = hid_configuration_descriptor,
        };
        ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
        ESP_LOGI(TAG, "USB initialization DONE");
    }
    static void app_send_hid_report(hid_keyboard_report_t *report) {
      ESP_LOGI(TAG, "Sending Keyboard report");

      tud_hid_report(HID_ITF_PROTOCOL_KEYBOARD, report,
                     sizeof(hid_keyboard_report_t));
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    void app_main(void)
    {
        init_usb();
        init_led();

        gpio_reset_pin(GPIO_NUM_2);                       // using built-in LED for notifications
        gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT); // Set the GPIO as a push/pull output
        gpio_set_level(GPIO_NUM_2, 1);
        
        gpio_set_level(GPIO_NUM_2, 0);

        // init BTKeyboard
        esp_err_t ret;

        // To test the Pairing code entry, uncomment the following line as pairing info is
        // kept in the nvs. Pairing will then be required on every boot.
        // ESP_ERROR_CHECK(nvs_flash_erase());

        ret = nvs_flash_init();
        if ((ret == ESP_ERR_NVS_NO_FREE_PAGES) || (ret == ESP_ERR_NVS_NEW_VERSION_FOUND))
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        if (bt_keyboard.setup(pairing_handler))
        { // Must be called once
            while (!bt_keyboard.devices_scan())
            {
                if (BTKeyboard::btFound)
                {
                    BTKeyboard::btFound = false;
                    for (int i = 0; i < 60; i++)
                    {
                        if (BTKeyboard::isConnected)
                            break;
                        ESP_LOGI(TAG, "Waiting for BT manual code entry...");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
                else
                {
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "Rescan...");
                }
            }                              // Required to discover new keyboards and for pairing
                                           // Default duration is 5 seconds
            gpio_set_level(GPIO_NUM_2, 1); // success, device found
        }

        // time variables, don't adjust unless you know what you're doing
        uint8_t typematicRate = 20;    // characters per second in Typematic mode

        // fixed stuff
        uint8_t cycle = 1000 / typematicRate;            // keywait timeout in ms. Important so we can check connection and do Typematic
        TickType_t repeat_period = pdMS_TO_TICKS(cycle); // keywait timeout in ticks. Important so we can check connection and do Typematic
        BTKeyboard::KeyInfo info;                        // freshly received
        BTKeyboard::KeyInfo infoBuf;                     // currently pressed
        bool found = false;                              // just an innocent flasg I mean flag

        info.modifier = infoBuf.modifier = (BTKeyboard::KeyModifier)0;

        for (int j = 0; j < BTKeyboard::MAX_KEY_COUNT; j++)
        {
            infoBuf.keys[j] = 0;
            info.keys[j] = 0;
        }

        while (true)
        {
            if (bt_keyboard.wait_for_low_event(info, repeat_period))
            {
                ESP_LOGI(TAG, "Received BT-HID report");
                ESP_LOGI(TAG, "Modifier: %02X", info.modifier);
                ESP_LOGI(TAG, "Keys:");
                ESP_LOG_BUFFER_HEX(TAG, info.keys, sizeof(info.keys));
                
                hid_keyboard_report_t report;
                reset_report(&report);

                report.modifier = (uint8_t)info.modifier;
                               
                // release the keys that have been just released
                for (int i = 0; i < BTKeyboard::MAX_KEY_COUNT; i++)
                {
                    if (!infoBuf.keys[i]) // Detect END FLAG
                        break;
                    for (int j = 0; j < BTKeyboard::MAX_KEY_COUNT; j++)
                    {
                        if (infoBuf.keys[i] == info.keys[j])
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        ESP_LOGI(TAG, "Up key: %x", infoBuf.keys[i]);
                        gpio_set_level(GPIO_NUM_2, 1);

                        report.keycode[i] = info.keys[i];
                    }
                    else
                        found = false;
                }

                // press the keys that have been just pressed
                for (int i = 0; i < BTKeyboard::MAX_KEY_COUNT; i++)
                {
                    if (!info.keys[i]) // Detect END FLAG
                        break;
                    for (int j = 0; (j < BTKeyboard::MAX_KEY_COUNT); j++)
                    {
                        if (info.keys[i] == infoBuf.keys[j])
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        ESP_LOGI(TAG, "Down key: %x", info.keys[i]);
                        gpio_set_level(GPIO_NUM_2, 0);

                        report.keycode[i] = info.keys[i];
                    }
                    else 
                        found = false;
                }

                // Process the received HID report            
                if (tud_mounted()) {
                    blink_led(1);
                    app_send_hid_report(&report);
                    blink_led(0);
                }
                infoBuf = info;                 // Now all the keys are handled, we save the state
            }

            else
            {
                while (!BTKeyboard::isConnected)
                {                                  // check connection
                    gpio_set_level(GPIO_NUM_2, 0); // disconnected
                    bt_keyboard.quick_reconnect(); // try to reconnect
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                }
                gpio_set_level(GPIO_NUM_2, 1);
            }
        }
    }
    }
