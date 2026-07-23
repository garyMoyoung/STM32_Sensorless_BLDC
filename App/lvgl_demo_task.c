/*
 * LVGL 测试动画: 独立于现有LCD遥测显示(App/lcd_task.c)之外的新增模块,
 * 通过开关 g_lvgl_demo_enable 决定是否接管LCD显示LVGL动画,
 * 关闭时不影响现有基于 LCD_xxx_DMA 的FOC遥测绘制逻辑。
 *
 * 动画仍然通过 lv_port_disp.c 里已经接好的 disp_flush -> LCD_LVGL_Color_Fill_DMA
 * 路径, 直接用SPI+DMA把像素写到物理LCD上, 不引入任何软件模拟显示。
 *
 * 帧率统计复用LVGL disp_drv->monitor_cb 在每次刷新后回调的 (耗时ms, 像素数),
 * 与 lv_conf.h 里已经打开的 LV_USE_PERF_MONITOR 内建帧率/CPU占用悬浮窗数据同源,
 * 用于判断当前"逐字节DMA+忙等"方式的传输速度是否能撑起流畅动画。
 */
#include "lvgl_demo_task.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lcd_task.h"
#include "lcd_init.h"
#include <stdio.h>

volatile uint8_t g_lvgl_demo_enable = 0U;

typedef enum
{
    LVGL_DEMO_OFF = 0,
    LVGL_DEMO_STARTING,
    LVGL_DEMO_ON,
    LVGL_DEMO_STOPPING
} lvgl_demo_state_t;

/* 与LCD遥测任务(150ms周期)切换LCD/SPI总线使用权时的等待时间,
   需大于对方任务的刷新周期,确保对方已经看到禁用标志并退出SPI访问 */
#define LVGL_HANDOFF_SETTLE_MS   200U
#define LVGL_TICK_PERIOD_MS      5U
#define LVGL_FPS_REPORT_PERIOD_MS 1000U
/* 低于此帧率认为当前逐字节DMA传输方式不足以支撑流畅动画 */
#define LVGL_FPS_MIN_OK          20.0f

static lvgl_demo_state_t s_state = LVGL_DEMO_OFF;
static uint32_t s_state_change_tick = 0U;
static uint8_t s_lvgl_ready = 0U;

static lv_obj_t *s_scr = NULL;
static lv_obj_t *s_ball = NULL;
static lv_obj_t *s_fps_label = NULL;

static volatile uint32_t s_frame_count = 0U;
static volatile uint32_t s_frame_time_sum_ms = 0U;
static volatile uint32_t s_frame_px_sum = 0U;
static uint32_t s_stat_window_start = 0U;

static void demo_monitor_cb(lv_disp_drv_t *disp_drv, uint32_t time, uint32_t px)
{
    (void)disp_drv;
    s_frame_count++;
    s_frame_time_sum_ms += time;
    s_frame_px_sum += px;
}

static void ball_x_anim_cb(void *obj, int32_t v)
{
    lv_obj_set_x((lv_obj_t *)obj, v);
}

static void build_demo_scene(void)
{
    s_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_scr, LV_OPA_COVER, 0);

    lv_obj_t *title = lv_label_create(s_scr);
    lv_label_set_text(title, "LVGL DMA Anim Test");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

    /* 旋转的圆弧: 只重绘一小块区域,用来观察小面积高频局部刷新的帧率上限 */
    lv_obj_t *spinner = lv_spinner_create(s_scr, 1000, 90);
    lv_obj_set_size(spinner, 60, 60);
    lv_obj_align(spinner, LV_ALIGN_CENTER, 0, -10);

    /* 左右弹跳的小球: 每帧都要清旧画新、覆盖较大范围横向移动,是DMA传输速度的压力测试 */
    s_ball = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_ball);
    lv_obj_set_size(s_ball, 20, 20);
    lv_obj_set_style_radius(s_ball, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_ball, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_bg_opa(s_ball, LV_OPA_COVER, 0);
    lv_obj_set_pos(s_ball, 0, LCD_H - 40);

    static lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_ball);
    lv_anim_set_exec_cb(&a, ball_x_anim_cb);
    lv_anim_set_values(&a, 0, LCD_W - 20);
    lv_anim_set_time(&a, 900);
    lv_anim_set_playback_time(&a, 900);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);

    s_fps_label = lv_label_create(s_scr);
    lv_obj_set_style_text_color(s_fps_label, lv_color_hex(0x00FF66), 0);
    lv_label_set_text(s_fps_label, "FPS: --");
    lv_obj_align(s_fps_label, LV_ALIGN_TOP_LEFT, 4, 24);
}

static void report_fps_if_due(uint32_t now)
{
    uint32_t elapsed = now - s_stat_window_start;

    if (elapsed < LVGL_FPS_REPORT_PERIOD_MS)
    {
        return;
    }

    uint32_t frames = s_frame_count;
    uint32_t time_sum = s_frame_time_sum_ms;
    uint32_t px_sum = s_frame_px_sum;

    s_frame_count = 0U;
    s_frame_time_sum_ms = 0U;
    s_frame_px_sum = 0U;
    s_stat_window_start = now;

    float fps = (float)frames * 1000.0f / (float)elapsed;
    float avg_flush_ms = (frames > 0U) ? ((float)time_sum / (float)frames) : 0.0f;
    const char *verdict = (fps >= LVGL_FPS_MIN_OK) ? "OK" : "LOW";

    if (s_fps_label != NULL)
    {
        lv_label_set_text_fmt(s_fps_label, "FPS:%d.%d flush:%dms %s",
                               (int)fps, (int)(fps * 10) % 10,
                               (int)avg_flush_ms, verdict);
    }

    printf("$LVGL_FPS,%d.%d,%dms,px=%lu,%s#\r\n",
           (int)fps, (int)(fps * 10) % 10, (int)avg_flush_ms,
           (unsigned long)px_sum, verdict);
}

void LvglDemo_SetEnable(uint8_t enable)
{
    g_lvgl_demo_enable = (enable != 0U) ? 1U : 0U;
}

void LvglDemo_Process(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_state)
    {
        case LVGL_DEMO_OFF:
            if (g_lvgl_demo_enable != 0U)
            {
                /* 先让现有LCD遥测任务停止访问SPI/LCD,再切到LVGL动画,避免两个任务同时抢SPI总线 */
                LCD_SetEnable(0U);
                s_state_change_tick = now;
                s_state = LVGL_DEMO_STARTING;
            }
            break;

        case LVGL_DEMO_STARTING:
            if ((now - s_state_change_tick) >= LVGL_HANDOFF_SETTLE_MS)
            {
                if (s_lvgl_ready == 0U)
                {
                    lv_init();
                    lv_port_disp_init();

                    lv_disp_t *disp = lv_disp_get_default();
                    if ((disp != NULL) && (disp->driver != NULL))
                    {
                        disp->driver->monitor_cb = demo_monitor_cb;
                    }
                    s_lvgl_ready = 1U;
                }

                if (s_scr == NULL)
                {
                    build_demo_scene();
                }

                lv_scr_load(s_scr);
                s_frame_count = 0U;
                s_frame_time_sum_ms = 0U;
                s_frame_px_sum = 0U;
                s_stat_window_start = now;
                s_state = LVGL_DEMO_ON;
            }
            break;

        case LVGL_DEMO_ON:
            if (g_lvgl_demo_enable == 0U)
            {
                s_state_change_tick = now;
                s_state = LVGL_DEMO_STOPPING;
                break;
            }
            lv_tick_inc(LVGL_TICK_PERIOD_MS);
            lv_timer_handler();
            report_fps_if_due(now);
            break;

        case LVGL_DEMO_STOPPING:
            /* 停止喂给LVGL的tick/timer_handler后,等待稳定时间再把LCD交还给遥测任务 */
            if ((now - s_state_change_tick) >= LVGL_HANDOFF_SETTLE_MS)
            {
                LCD_SetEnable(1U);
                s_state = LVGL_DEMO_OFF;
            }
            break;

        default:
            s_state = LVGL_DEMO_OFF;
            break;
    }
}
