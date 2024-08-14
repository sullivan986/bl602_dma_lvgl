#include "ui_disp.h"
#include "lvgl.h"
#include "src/core/lv_obj_style.h"
#include "src/core/lv_obj_style_gen.h"
#include "stdlib.h"

// chart
static lv_obj_t *chart_motor;
static lv_chart_series_t *motor_c12b_series;
static lv_chart_series_t *motor_mg6012_1_series;
static lv_chart_series_t *motor_mg6012_2_series;
static lv_chart_series_t *motor_ax12_series;
static lv_timer_t *chart_update_timer;

// label of motor data
lv_obj_t *label_motor_state_circle;
lv_obj_t *label_motor_state_text;

/* Timer handler: fetches sensor data and appends it to the chart */
static void sensor_timer_cb(lv_timer_t *timer)
{
    lv_chart_set_next_value(chart_motor, motor_c12b_series, rand() % 200);
    lv_chart_set_next_value(chart_motor, motor_mg6012_1_series, rand() % 200);
    lv_chart_set_next_value(chart_motor, motor_mg6012_2_series, rand() % 200);
    lv_chart_set_next_value(chart_motor, motor_ax12_series, rand() % 200);
}

void ui_init()
{
    // chart
    chart_motor = lv_chart_create(lv_screen_active());

    lv_obj_set_size(chart_motor, LV_HOR_RES, LV_VER_RES);
    lv_obj_center(chart_motor);
    lv_obj_set_style_size(chart_motor, 0, 0, LV_PART_INDICATOR); // 取消点显示

    lv_chart_set_type(chart_motor, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(chart_motor, 5, 8);                       // 设置背景分割线
    lv_chart_set_range(chart_motor, LV_CHART_AXIS_PRIMARY_Y, 0, 200);     // y轴范围
    lv_chart_set_point_count(chart_motor, 300);                           // x轴分辨率
    lv_chart_set_update_mode(chart_motor, LV_CHART_UPDATE_MODE_CIRCULAR); // 顺序更新

    motor_c12b_series = lv_chart_add_series(chart_motor, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    motor_mg6012_1_series = lv_chart_add_series(chart_motor, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    motor_mg6012_2_series =
        lv_chart_add_series(chart_motor, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    motor_ax12_series = lv_chart_add_series(chart_motor, lv_palette_main(LV_PALETTE_PURPLE), LV_CHART_AXIS_PRIMARY_Y);

    // label
    label_motor_state_circle = lv_obj_create(lv_screen_active());
    lv_obj_set_style_width(label_motor_state_circle, LV_SIZE_CONTENT, 0);
    lv_obj_set_style_height(label_motor_state_circle, LV_SIZE_CONTENT, 0);
    lv_obj_set_style_x(label_motor_state_circle, 5, 0);
    lv_obj_set_style_y(label_motor_state_circle, 180, 0);
    lv_obj_set_style_pad_all(label_motor_state_circle, 0, 0);                                 // 内边距
    lv_obj_set_style_bg_opa(label_motor_state_circle, 200, 0);                                // 背景透明度
    lv_obj_set_style_bg_color(label_motor_state_circle, lv_palette_main(LV_PALETTE_GREY), 0); // 背景颜色
    lv_obj_set_style_border_width(label_motor_state_circle, 1, 0);                            // 边框宽度

    label_motor_state_text = lv_label_create(label_motor_state_circle);
    // lv_label_set_long_mode(label_motor_state_text, LV_LABEL_LONG_WRAP); // 自动换行
    lv_label_set_text(label_motor_state_text, "Recolor is not \nsupported for v9 \nnow.");

    // lv_obj_set_width(label_motor_state_text, 150); /*Set smaller width to make the lines wrap*/
    // lv_obj_set_style_text_align(label_motor_state_text, LV_TEXT_ALIGN_CENTER, 0);
    // lv_obj_align(label_motor_state_text, LV_ALIGN_BOTTOM_LEFT, 0, -40);

    // lv_obj_set_style_text_align(label_motor_state_text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(label_motor_state_text, lv_palette_main(LV_PALETTE_DEEP_ORANGE), 0);

    // timer
    chart_update_timer = lv_timer_create(sensor_timer_cb, 1000 / 50, NULL);
}