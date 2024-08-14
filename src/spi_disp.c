// benchmark demon non rtos /4 : 77 35 30
// rtos /5 : 70 23 33

#include "spi_disp.h"
#include "bflb_dma.h"
#include "bflb_gpio.h"
#include "bflb_mtimer.h"
#include "bflb_spi.h"
#include "board.h"
#include "log.h"
#include "lvgl.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define USE_DMA_TRANSFER 1
#define USE_FREERTOS 1

// device
static struct bflb_device_s *gpio;
static struct bflb_device_s *spi0;
static struct bflb_device_s *dma0_ch0;

// spi and dma
static struct bflb_spi_config_s spi_cfg;
static struct bflb_dma_channel_config_s tx_config = {0};
static struct bflb_dma_channel_lli_transfer_s tx_transfers[1];
static struct bflb_dma_channel_lli_pool_s tx_llipool[5];

static int disp_spi_reset_pin = GPIO_PIN_4;
static int disp_spi_dc_pin = GPIO_PIN_11;
static int disp_spi_scl_pin = GPIO_PIN_3;
static int disp_spi_sda_pin = GPIO_PIN_12;

#if USE_FREERTOS
#include <FreeRTOS.h>
#define disp_sleep_ms(x) vTaskDelay(x)
#include "semphr.h"
#include "task.h"
static TaskHandle_t current_task_handler = NULL;
static bool send_first = true;
#else
static volatile bool spi_dma_busy = false;
#define disp_sleep_ms(x) bflb_mtimer_delay_ms(x)
#endif

// lvgl9 variables
static lv_display_t *display;
#define DISP_WIDTH 320
#define DISP_HEIGHT 240

#if USE_DMA_TRANSFER
static ATTR_NOCACHE_NOINIT_RAM_SECTION uint16_t buff_1[DISP_WIDTH * DISP_HEIGHT / 5];
static ATTR_NOCACHE_NOINIT_RAM_SECTION uint16_t buff_2[DISP_WIDTH * DISP_HEIGHT / 5];
#else
static uint16_t buff_1[DISP_WIDTH * DISP_HEIGHT / 5];
#endif

static void _isr_dma_spi_send_done(void *arg)
{
#if USE_FREERTOS
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(current_task_handler, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
    spi_dma_busy = false;
#endif
}

static void spi_send_one_8bit(uint8_t data)
{
#if USE_DMA_TRANSFER
    tx_config.src_width = DMA_DATA_WIDTH_8BIT;
    tx_config.dst_width = DMA_DATA_WIDTH_8BIT;
    bflb_dma_channel_init(dma0_ch0, &tx_config);
    bflb_dma_channel_irq_attach(dma0_ch0, _isr_dma_spi_send_done, NULL);
#if USE_FREERTOS
    current_task_handler = xTaskGetCurrentTaskHandle();
#else
    spi_dma_busy = true;
#endif
    tx_transfers[0].src_addr = (uint32_t)&data;
    tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_SPI0_TDR;
    tx_transfers[0].nbytes = 1;
    bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 1, tx_transfers, 1);
    bflb_dma_channel_start(dma0_ch0);
#if USE_FREERTOS
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#else
    while (spi_dma_busy)
        ;
#endif
#else
    bflb_spi_poll_send(spi0, data);
#endif
}

static void spi_send_block_8bit(uint8_t *data, size_t len)
{
#if USE_DMA_TRANSFER
    tx_config.src_width = DMA_DATA_WIDTH_8BIT;
    tx_config.dst_width = DMA_DATA_WIDTH_8BIT;
    bflb_dma_channel_init(dma0_ch0, &tx_config);
    bflb_dma_channel_irq_attach(dma0_ch0, _isr_dma_spi_send_done, NULL);
#if USE_FREERTOS
    current_task_handler = xTaskGetCurrentTaskHandle();
#else
    spi_dma_busy = true;
#endif
    tx_transfers[0].src_addr = (uint32_t)data;
    tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_SPI0_TDR;
    tx_transfers[0].nbytes = len;
    bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 1, tx_transfers, 1);
    bflb_dma_channel_start(dma0_ch0);
#if USE_FREERTOS
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#else
    while (spi_dma_busy)
        ;
#endif
#else
    bflb_spi_poll_exchange(spi0, data, NULL, len);
#endif
}

static void spi_send_block_16bit(uint16_t *data, size_t len)
{
#if USE_DMA_TRANSFER
    tx_config.src_width = DMA_DATA_WIDTH_16BIT;
    tx_config.dst_width = DMA_DATA_WIDTH_16BIT;
    bflb_dma_channel_init(dma0_ch0, &tx_config);
    bflb_dma_channel_irq_attach(dma0_ch0, _isr_dma_spi_send_done, NULL);
#if USE_FREERTOS
    current_task_handler = xTaskGetCurrentTaskHandle();
#else
    spi_dma_busy = true;
#endif
    tx_transfers[0].src_addr = (uint32_t)data;
    tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_SPI0_TDR;
    tx_transfers[0].nbytes = len * 2;
    bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 5, tx_transfers, 1);
    bflb_dma_channel_start(dma0_ch0);
#else
    bflb_spi_poll_exchange(spi0, data, NULL, len * 2);
#endif
}

static void disp_send_cmd(uint8_t cmd)
{
    bflb_gpio_reset(gpio, disp_spi_dc_pin);
    bflb_spi_feature_control(spi0, SPI_CMD_SET_DATA_WIDTH, SPI_DATA_WIDTH_8BIT);
    spi_send_one_8bit(cmd);
}

static void disp_send_data(uint8_t *data, size_t len)
{
    bflb_gpio_set(gpio, disp_spi_dc_pin);
    bflb_spi_feature_control(spi0, SPI_CMD_SET_DATA_WIDTH, SPI_DATA_WIDTH_8BIT);
    spi_send_block_8bit(data, len);
}

static void disp_send_data16(uint16_t *data, size_t len)
{
    lv_draw_sw_rgb565_swap(data, len);
    bflb_gpio_set(gpio, disp_spi_dc_pin);
    bflb_spi_feature_control(spi0, SPI_CMD_SET_DATA_WIDTH, SPI_DATA_WIDTH_16BIT);
    spi_send_block_16bit(data, len);
}

static void disp_reset()
{
    bflb_gpio_reset(gpio, disp_spi_reset_pin);
    disp_sleep_ms(100);
    bflb_gpio_set(gpio, disp_spi_reset_pin);
    disp_sleep_ms(100);
}

void ili9341_init()
{
    spi_cfg.freq = 40 * 1000 * 1000;
    spi_cfg.role = SPI_ROLE_MASTER;
    spi_cfg.mode = SPI_MODE0;
    spi_cfg.data_width = SPI_DATA_WIDTH_8BIT;
    spi_cfg.bit_order = SPI_BIT_MSB;
    spi_cfg.byte_order = SPI_BYTE_LSB;
    spi_cfg.tx_fifo_threshold = 0;
    spi_cfg.rx_fifo_threshold = 0;

    gpio = bflb_device_get_by_name("gpio");
    // spi gpio init
    bflb_gpio_init(gpio, disp_spi_scl_pin, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, disp_spi_sda_pin, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    // da and reset gpio init
    bflb_gpio_init(gpio, disp_spi_reset_pin, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_init(gpio, disp_spi_dc_pin, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);

    spi0 = bflb_device_get_by_name("spi0");
    bflb_spi_init(spi0, &spi_cfg);

    // enable spi err interrupt
    // bflb_spi_errint_mask(spi0, false);
    // bflb_irq_attach(spi0->irq_num, _isr_spi_send_done, NULL);
    // bflb_irq_enable(spi0->irq_num);

#if USE_DMA_TRANSFER
    bflb_spi_link_txdma(spi0, true);
    dma0_ch0 = bflb_device_get_by_name("dma0_ch0");

    tx_config.direction = DMA_MEMORY_TO_PERIPH;
    tx_config.src_req = DMA_REQUEST_NONE;
    tx_config.dst_req = DMA_REQUEST_SPI0_TX;
    tx_config.src_addr_inc = DMA_ADDR_INCREMENT_ENABLE;
    tx_config.dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE;
    tx_config.src_burst_count = DMA_BURST_INCR1;
    tx_config.dst_burst_count = DMA_BURST_INCR1;
#else

    // bflb_spi_txint_mask(spi0, false);
    // bflb_spi_tcint_mask(spi0, false);
#endif

    // display init
    disp_reset();
    disp_send_cmd(ILI9XXX_SWRESET);
    disp_sleep_ms(100);
    disp_send_cmd(ILI9XXX_SLPOUT);

    disp_send_cmd(0xcf);
    uint8_t data[] = {0x00, 0x83, 0x30};
    disp_send_data(data, 3);

    disp_send_cmd(0xe8);
    uint8_t data1[] = {0x85, 0x01, 0x79};
    disp_send_data(data1, 3);

    disp_send_cmd(0xf7);
    uint8_t data2[] = {0x20};
    disp_send_data(data2, 1);

    disp_send_cmd(0xea);
    uint8_t data3[] = {0x00, 0x00};
    disp_send_data(data3, 2);

    disp_send_cmd(0x3a);
    uint8_t data33[] = {0x55};
    disp_send_data(data33, 1);

    disp_send_cmd(0x36);
    uint8_t data4[] = {(1 << 3) | (1 << 5)};
    disp_send_data(data4, 1);

    disp_send_cmd(0x2a);
    uint8_t data5[] = {0x00, 0x00, (uint8_t)(DISP_WIDTH >> 8), (uint8_t)(DISP_WIDTH & 0xFF)};
    disp_send_data(data5, 4);

    disp_send_cmd(0x2b);
    uint8_t data6[] = {0x00, 0x00, (uint8_t)(DISP_HEIGHT >> 8), (uint8_t)(DISP_HEIGHT & 0xFF)};
    disp_send_data(data6, 4);

    disp_send_cmd(0xb1);
    uint8_t data7[] = {0x00, 0x10};
    disp_send_data(data7, 2);

    disp_send_cmd(0xf2);
    uint8_t data8[] = {0x08};
    disp_send_data(data8, 1);

    disp_send_cmd(0x26);
    uint8_t data9[] = {0x01};
    disp_send_data(data9, 1);

    disp_send_cmd(0xe0);
    uint8_t data10[] = {0x1f, 0x36, 0x36, 0x3a, 0x0c, 0x05, 0x4f, 0x87, 0x3c, 0x08, 0x11, 0x35, 0x19, 0x13, 0x00};
    disp_send_data(data10, 15);

    disp_send_cmd(0xe1);
    uint8_t data11[] = {0x00, 0x09, 0x09, 0x05, 0x13, 0x0a, 0x30, 0x78, 0x43, 0x07, 0x0e, 0x0a, 0x26, 0x2c, 0x1f};
    disp_send_data(data11, 15);

    disp_send_cmd(0x13);
    disp_send_cmd(0x29);
    disp_sleep_ms(10);
}

void ili9341_flush(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *color_data)
{
    disp_send_cmd(ILI9XXX_CASET);
    uint8_t _data1[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
    disp_send_data(_data1, 4);

    disp_send_cmd(ILI9XXX_PASET);
    uint8_t _data2[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
    disp_send_data(_data2, 4);

    disp_send_cmd(ILI9XXX_RAMWR);

    disp_send_data16(color_data, (x_end - x_start + 1) * (y_end - y_start + 1));

    //    disp_send_data16(color_data, (x_end - x_start + 1) * (y_end - y_start + 1));
    // disp_send_data16(color_data, (x_end - x_start + 1) * (y_end - y_start + 1));
    // spi_busy = true;
}
// lvgl
static void ili9341_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
#if USE_FREERTOS
    if (send_first)
    {
        send_first = false;
    }
    else
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
#else
    while (spi_dma_busy)
        ;
#endif
    lv_display_flush_ready(display);
    ili9341_flush(area->x1, area->y1, area->x2, area->y2, (uint16_t *)px_map);
}

#if USE_FREERTOS
static void lvgl_fresh_task(void *p)
{

    while (true)
    {
        vTaskDelay(lv_timer_handler());
    }
}
#else
void lvgl_tick_task()
{
    lv_tick_inc(LV_TICK_PERIOD_MS);
}
#endif

void bl_init_lvgl()
{
    ili9341_init();

    lv_init();
    display = lv_display_create(DISP_WIDTH, DISP_HEIGHT);
    lv_display_set_flush_cb(display, ili9341_flush_cb);

    lv_display_set_buffers(display, buff_1,
#if USE_DMA_TRANSFER
                           buff_2
#else
                           NULL
#endif
                           ,
                           sizeof(buff_1), LV_DISPLAY_RENDER_MODE_PARTIAL);
#if USE_FREERTOS
    lv_tick_set_cb(xTaskGetTickCount);
#else
    bflb_mtimer_config(1000, lvgl_tick_task); // lv_tick_set_cb(xTaskGetTickCount);
#endif
}

void bl_lvgl_start()
{
#if USE_FREERTOS
    xTaskCreate(lvgl_fresh_task, "lvgl", 4096, NULL, configMAX_PRIORITIES - 2, NULL);
#else
    while (true)
    {
        disp_sleep_ms(lv_timer_handler());
    }
#endif
}