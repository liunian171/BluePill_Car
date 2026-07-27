#include "usergpio.h"

void usergpio_write(UserGPIO_Handle *hGPIO, uint8_t state)
{
    hGPIO->ops->write(hGPIO->hgpio_port, hGPIO->gpio_pin, state);
}

void usergpio_toggle(UserGPIO_Handle *hGPIO)
{
    hGPIO->ops->toggle(hGPIO->hgpio_port, hGPIO->gpio_pin);
}

uint8_t usergpio_read(UserGPIO_Handle *hGPIO)
{
    return hGPIO->ops->read(hGPIO->hgpio_port, hGPIO->gpio_pin);
}
