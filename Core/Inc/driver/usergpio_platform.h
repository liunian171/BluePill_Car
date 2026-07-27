#ifndef USERGPIO_PLATFORM_H__
#define USERGPIO_PLATFORM_H__

#include "usergpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void usergpio_stm32_write(void *hgpio_port, uint16_t gpio_pin, uint8_t state);
void usergpio_stm32_toggle(void *hgpio_port, uint16_t gpio_pin);
uint8_t usergpio_stm32_read(void *hgpio_port, uint16_t gpio_pin);

extern const UserGPIO_PlatformOps_t usergpio_platform_ops_stm32;

#ifdef __cplusplus
}
#endif

#endif
