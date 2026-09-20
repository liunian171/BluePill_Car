#include "usergpio.h"

void usergpio_write(UserGPIO_Handle *hGPIO, uint8_t state)
{
    hGPIO->ops->write(hGPIO->hgpio_port, hGPIO->gpio_pin, state);
}

uint8_t usergpio_read(UserGPIO_Handle *hGPIO)
{
    return hGPIO->ops->read(hGPIO->hgpio_port, hGPIO->gpio_pin);
}

/* 2026-09-20 P1-6: usergpio_toggle 判死删除（§7 零调用）。
 * 平台原语 ops->toggle 保留（原子集合完整）。 */
