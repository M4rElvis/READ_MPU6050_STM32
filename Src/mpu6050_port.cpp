#include "STM32_ArduinoCompat.h"
#include "i2c.h"
#include "mpu6050_port.h"
#include "MPU6050_6Axis_MotionApps612.h"

static MPU6050 mpu(MPU6050_DEFAULT_ADDRESS, &hi2c1);
static uint8_t fifoPacket[28];
static Quaternion quaternion;
static VectorFloat gravity;
static float ypr[3];
static uint8_t initialized = 0;
static uint8_t dmpError = 0;

extern "C" uint8_t MPU6050_Port_Init(void)
{
    initialized = 0;
    dmpError = 0;

    HAL_Delay(100);

    /* The library's testConnection() expects the 6-bit WHO_AM_I value 0x34. */
    if (!mpu.testConnection()) {
        return 1;
    }

    dmpError = mpu.dmpInitialize();
    if (dmpError != 0U) {
        return 2;
    }

    mpu.setDMPEnabled(true);
    if (!mpu.getDMPEnabled()) {
        return 3;
    }

    mpu.resetFIFO();
    initialized = 1;
    return 0;
}

extern "C" uint8_t MPU6050_Port_DMPError(void)
{
    return dmpError;
}

extern "C" uint8_t MPU6050_Port_ReadYPR(float *yaw, float *pitch, float *roll)
{
    if ((!initialized) || (yaw == NULL) || (pitch == NULL) || (roll == NULL)) {
        return 0;
    }

    /* MotionApps 6.12 default packet is 28 bytes. This helper keeps the newest
     * complete packet and also recovers from FIFO backlog/overflow.
     */
    if (!mpu.dmpGetCurrentFIFOPacket(fifoPacket)) {
        return 0;
    }

    if (mpu.dmpGetQuaternion(&quaternion, fifoPacket) != 0U) return 0;
    if (mpu.dmpGetGravity(&gravity, &quaternion) != 0U) return 0;
    if (mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity) != 0U) return 0;

    *yaw = ypr[0];
    *pitch = ypr[1];
    *roll = ypr[2];
    return 1;
}
