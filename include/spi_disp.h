#ifndef _SPI_SIDP_H_
#define _SPI_SIDP_H_

/* Commands/registers. */
#include <stdint.h>
#define ILI9XXX_SWRESET 0x01
#define ILI9XXX_SLPOUT 0x11
#define ILI9XXX_DINVON 0x21
#define ILI9XXX_GAMSET 0x26
#define ILI9XXX_DISPOFF 0x28
#define ILI9XXX_DISPON 0x29
#define ILI9XXX_CASET 0x2a
#define ILI9XXX_PASET 0x2b
#define ILI9XXX_RAMWR 0x2c
#define ILI9XXX_RGBSET 0x2d
#define ILI9XXX_RAMRD 0x2e
#define ILI9XXX_MADCTL 0x36
#define ILI9XXX_PIXSET 0x3A
#define ILI9XXX_RAMRD_CONT 0x3e

/* MADCTL register fields. */
#define ILI9XXX_MADCTL_MY BIT(7U)
#define ILI9XXX_MADCTL_MX BIT(6U)
#define ILI9XXX_MADCTL_MV BIT(5U)
#define ILI9XXX_MADCTL_ML BIT(4U)
#define ILI9XXX_MADCTL_BGR BIT(3U)
#define ILI9XXX_MADCTL_MH BIT(2U)

void ili9341_init();
void ili9341_flush(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *color_data);

void bl_init_lvgl();

void bl_lvgl_start();

#endif