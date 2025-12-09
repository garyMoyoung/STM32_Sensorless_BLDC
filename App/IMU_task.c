#include "cmsis_os.h"
#include "math.h"
#include "main.h"
#include "MPU9250-DMP.h"
#include "MPU9250_RegisterMap.h"
#include "math.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "stdio.h"
#include "IMU_task.h"
/* GYROScope BEGIN*/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern int ax, ay, az;
extern int gx, gy, gz;
extern int mx, my, mz;
extern long qw, qx, qy, qz;
extern long temperature;
extern unsigned long time_inside;
extern float pitch_inside, roll_inside, yaw_inside;
extern float heading;
extern unsigned short _aSense;
extern float _gSense, _mSense;
/* GYROScope END*/

void IMU9250Task_Entry(void const * argument)
{
    /* USER CODE BEGIN IMU9250Task_Entry */

    /*MPU9250 BEGIN*/
    if (MPU9250_begin() != INV_SUCCESS) 
    {
        printf("Unable to initialize MPU9250\n");
        // return -1;
    }

    unsigned short dmpFeatures = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
    if (MPU9250_dmpBegin(dmpFeatures, 100) != INV_SUCCESS)  // 100Hz FIFO rate
    {
        printf("Failed to initialize DMP\n");
        // return -1;
    }

    if (MPU9250_enableInterrupt(1) != INV_SUCCESS) 
    {
        printf("Failed to enable interrupts\n");
        // return -1;
    }
    /*MPU9250 END*/

    /* Infinite loop */
    for(;;)
    {
        // printf("get in IMU9250Task_Entry\r\n");
        if (MPU9250_dataReady())
        {

            if (MPU9250_dmpUpdateFifo() == INV_SUCCESS)
            {
                MPU9250_updateCompass();
                MPU9250_computeEulerAnglesWithMag(true);
                printf("Pitch:Roll:Yaw:%.4f,%.4f,%.4f\n", 
                    pitch_inside, roll_inside, yaw_inside);
            }
        }
        osDelay(1);
    }
    /* USER CODE END IMU9250Task_Entry */
}
