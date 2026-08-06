/*
 * ESP32-S3 Gateway Board Abstraction Implementation
 */

#include <stdio.h>
#include "esp_log.h"
#include "board.h"

static const char *TAG = "BOARD";

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    gpio_set_level((gpio_num_t)pin, onoff ? LED_ON : LED_OFF);
}

void board_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_0) | (1ULL << LED_1),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    board_led_operation(LED_0, LED_OFF);
    board_led_operation(LED_1, LED_OFF);
    ESP_LOGI(TAG, "ESP32-S3 Gateway Board initialized");
}
