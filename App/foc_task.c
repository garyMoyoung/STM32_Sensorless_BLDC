#include "cmsis_os.h"
#include "math.h"
#include "main.h"
#include "AS5600.h"
extern float Mech_Angle;
void AngleTask_Entry(void *argument)
{

  for(;;)
  {
    osDelay(5);  // 100Hz 更新
  }
}

