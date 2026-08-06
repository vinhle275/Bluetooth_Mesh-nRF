/*
 * ESP32-S3 Gateway Board Abstraction Header
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "driver/gpio.h"

#define LED_0 GPIO_NUM_4
#define LED_1 GPIO_NUM_5

#define LED_ON  1
#define LED_OFF 0

void board_init(void);
void board_led_operation(uint8_t pin, uint8_t onoff);

#endif /* _BOARD_H_ */
