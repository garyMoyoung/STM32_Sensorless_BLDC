#include "cmsis_os.h"
#include "math.h"
#include "main.h"
#include "AS5600.h"
extern AS5600_T AS5600;
extern float Mech_Angle;
void AngleTask_Entry(void *argument)
{

  for(;;)
  {
    AS5600_Update(&AS5600);  // 在任务中进行 I2C，不会卡中断
    Mech_Angle = AS5600_GetOnceAngle(&AS5600);
    osDelay(5);  // 100Hz 更新
  }
}

