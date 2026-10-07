#ifndef RM26_IMU_TASK_H
#define RM26_IMU_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float accel[3];       /* XYZ acceleration, m/s^2 */
    float gyro[3];        /* XYZ angular velocity, rad/s */
    float temperature;    /* accelerometer temperature, degrees Celsius */
    /* Each validity flag describes its own sensor/register read. */
    uint8_t acc_valid;
    uint8_t gyro_valid;
    uint8_t temperature_valid;
} BMI088_Sample;

/* Initialize and calibrate the singleton BMI088 instance; nonzero means success. */
uint8_t BMI088_TaskInit(void);
/* Read one sample; returns nonzero only when all sample fields are valid. */
uint8_t BMI088_TaskRead(BMI088_Sample *sample);
/* Copy the latest sample published by the IMU ThreadX worker. */
void IMU_GetLatest(BMI088_Sample *sample);

#ifdef __cplusplus
}
#endif

#endif
