#include "benchmark/lv_demo_benchmark.h"
#include "board.h"
#include "spi_disp.h"
#include "ui_disp.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "task.h"

void lvgl_start_task(void *pvParameters)
{
    bl_init_lvgl();
    lv_demo_benchmark();
    bl_lvgl_start();
    vTaskDelete(NULL);
}

int main(void)
{
    board_init();
    xTaskCreate(lvgl_start_task, "test", 2048, NULL, 1, NULL);
    vTaskStartScheduler();
}

