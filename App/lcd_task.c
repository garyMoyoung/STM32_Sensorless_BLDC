#include "lcd_task.h"
#include "cmsis_os.h"
#include "lcd_init.h"
#include "lcd.h"
#include "pic.h"
#include "math.h"
#include "main.h"
#include "lvgl.h"                // 它为整个LVGL提供了更完整的头文件引用
#include "lv_port_disp.h"        // LVGL的显示支持
#include "lv_port_indev.h"       // LVGL的触屏支持
#include "lcd_dma.h"

/* USER CODE BEGIN Header_LcdTask_Entry */
/**
* @brief Function show the informations of five-bar
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LcdTask_Entry */


void LcdTask_Entry(void const * argument)
{
    LCD_Init_DMA();
    // LCD_Fill_DMA(0,0,LCD_W, LCD_H,BLACK);
	lv_init();                             // LVGL 初始化
	lv_port_disp_init();                   // 注册LVGL的显示任务

   // 创建一个按钮
   lv_obj_t *btn = lv_btn_create(lv_scr_act());
   lv_obj_set_size(btn, 100, 40);
   lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 20);

   lv_obj_t *btn_label = lv_label_create(btn);
   lv_label_set_text(btn_label, "Click Me");
   lv_obj_center(btn_label);

   // 创建一个标签
   lv_obj_t *hello_label = lv_label_create(lv_scr_act());
   lv_label_set_text(hello_label, "Hello LVGL!");
   lv_obj_align(hello_label, LV_ALIGN_TOP_MID, 0, 70);

    // anim_btn = lv_btn_create(lv_scr_act());
    // lv_obj_set_size(anim_btn, 80, 40);
    // lv_obj_align(anim_btn, LV_ALIGN_TOP_LEFT, 0, 40);

    // lv_obj_t *btn_label = lv_label_create(anim_btn);
    // lv_label_set_text(btn_label, "动画按钮");
    // lv_obj_center(btn_label);

    for(;;)
    {

        osDelay(10);
    }
}

