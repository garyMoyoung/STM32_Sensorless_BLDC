#include <string.h>
#include <stddef.h>
#include "pid_storage.h"
#include "pid.h"

extern PIDController PID_Current_D;
extern PIDController PID_Current_Q;
extern PIDController PID_Speed;
extern PIDController PID_Position;

/* Sector 7 = 0x08060000~0x0807FFFF, 512KB器件的最后一个扇区(128KB),
   固件代码目前只用到前面几十KB,离这里很远,不会随代码增长被覆盖。 */
#define PID_STORAGE_FLASH_ADDR   0x08060000U
#define PID_STORAGE_FLASH_SECTOR FLASH_SECTOR_7
#define PID_STORAGE_MAGIC        0x50494431U /* "PID1" */

typedef struct
{
    float kp;
    float ki;
    float kd;
    float target;
} PidStorage_Loop_t;

typedef struct
{
    uint32_t magic;
    PidStorage_Loop_t id;
    PidStorage_Loop_t iq;
    PidStorage_Loop_t speed;
    PidStorage_Loop_t position;
    uint32_t checksum;
} PidStorage_Data_t;

static uint32_t PidStorage_Checksum(const PidStorage_Data_t *data)
{
    const uint32_t *words = (const uint32_t *)data;
    uint32_t sum = 0U;
    size_t word_count = (offsetof(PidStorage_Data_t, checksum)) / sizeof(uint32_t);
    size_t i;

    for (i = 0U; i < word_count; i++)
    {
        sum += words[i];
    }

    return sum;
}

static void PidStorage_FillFromLive(PidStorage_Data_t *data)
{
    memset(data, 0, sizeof(*data));
    data->magic = PID_STORAGE_MAGIC;

    data->id.kp = PID_Current_D.kp;
    data->id.ki = PID_Current_D.ki;
    data->id.kd = PID_Current_D.kd;
    data->id.target = PID_Current_D.target;

    data->iq.kp = PID_Current_Q.kp;
    data->iq.ki = PID_Current_Q.ki;
    data->iq.kd = PID_Current_Q.kd;
    data->iq.target = PID_Current_Q.target;

    data->speed.kp = PID_Speed.kp;
    data->speed.ki = PID_Speed.ki;
    data->speed.kd = PID_Speed.kd;
    data->speed.target = PID_Speed.target;

    data->position.kp = PID_Position.kp;
    data->position.ki = PID_Position.ki;
    data->position.kd = PID_Position.kd;
    data->position.target = PID_Position.target;

    data->checksum = PidStorage_Checksum(data);
}

uint8_t PidStorage_Save(void)
{
    PidStorage_Data_t data;
    const uint32_t *words;
    size_t word_count;
    size_t i;
    uint32_t address;
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0U;

    PidStorage_FillFromLive(&data);

    words = (const uint32_t *)&data;
    word_count = sizeof(data) / sizeof(uint32_t);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                            FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    memset(&erase_init, 0, sizeof(erase_init));
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = PID_STORAGE_FLASH_SECTOR;
    erase_init.NbSectors = 1U;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    if ((status != HAL_OK) || (sector_error != 0xFFFFFFFFU))
    {
        HAL_FLASH_Lock();
        return 0U;
    }

    address = PID_STORAGE_FLASH_ADDR;
    for (i = 0U; i < word_count; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, words[i]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0U;
        }
        address += sizeof(uint32_t);
    }

    HAL_FLASH_Lock();

    return (memcmp((const void *)PID_STORAGE_FLASH_ADDR, &data, sizeof(data)) == 0) ? 1U : 0U;
}

uint8_t PidStorage_Load(void)
{
    const PidStorage_Data_t *stored = (const PidStorage_Data_t *)PID_STORAGE_FLASH_ADDR;
    PidStorage_Data_t data;

    memcpy(&data, stored, sizeof(data));

    if (data.magic != PID_STORAGE_MAGIC)
    {
        return 0U;
    }

    if (data.checksum != PidStorage_Checksum(&data))
    {
        return 0U;
    }

    PID_param_set(&PID_Current_D, data.id.kp, data.id.ki, data.id.kd);
    PID_Current_D.target = data.id.target;

    PID_param_set(&PID_Current_Q, data.iq.kp, data.iq.ki, data.iq.kd);
    PID_Current_Q.target = data.iq.target;

    PID_param_set(&PID_Speed, data.speed.kp, data.speed.ki, data.speed.kd);
    PID_Speed.target = data.speed.target;

    PID_param_set(&PID_Position, data.position.kp, data.position.ki, data.position.kd);
    PID_Position.target = data.position.target;

    return 1U;
}
