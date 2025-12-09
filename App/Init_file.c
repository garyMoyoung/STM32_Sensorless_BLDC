#include "lcd_task.h"
#include "main.h"
#include "lvgl.h"                // 瀹冧负鏁翠釜LVGL鎻愪緵浜嗘洿瀹屾暣鐨勫ご鏂囦欢寮曠敤
#include "lv_port_disp.h"        // LVGL鐨勬樉绀烘敮鎸�
#include "lv_port_indev.h"       // LVGL鐨勮Е灞忔敮鎸�
#include "cmsis_os.h"
#include "IMU_task.h"
#include "MPU9250-DMP.h"
#include "MPU9250_RegisterMap.h"
#include "math.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "stdio.h"

void Init_All(void)
{
	__disable_irq();
	

	/*MPU9250 BEGIN*/
    if (MPU9250_begin() != INV_SUCCESS) 
    {
        printf("Unable to initialize MPU9250\n");
        // return -1;
    }
    // 鍚敤DMP鐗癸拷?锟藉拰璁剧疆FIFO閲囨牱锟�????????
    unsigned short dmpFeatures = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
    if (MPU9250_dmpBegin(dmpFeatures, 100) != INV_SUCCESS)  // 100Hz FIFO rate
    {
        printf("Failed to initialize DMP\n");
        // return -1;
    }
    // 鍚敤涓柇锛岃繖鏍锋瘡褰撴湁鏂版暟鎹彲璇绘椂锛孧PU9250浼氾拷?锟界煡MCU
    if (MPU9250_enableInterrupt(1) != INV_SUCCESS) 
    {
        printf("Failed to enable interrupts\n");
        // return -1;
    }
    /*MPU9250 END*/
	__enable_irq();
}