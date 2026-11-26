#ifndef MPU6050_PORT_H
#define MPU6050_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Return values from MPU6050_Port_Init:
 * 0 = success
 * 1 = MPU6050 not detected (testConnection failed)
 * 2 = DMP firmware initialization failed (see MPU6050_Port_DMPError)
 * 3 = DMP did not become enabled
 */
uint8_t MPU6050_Port_Init(void);
uint8_t MPU6050_Port_DMPError(void);

/* Returns 1 when a fresh FIFO packet was processed, otherwise 0.
 * Yaw, pitch and roll are returned in radians.
 */
uint8_t MPU6050_Port_ReadYPR(float *yaw, float *pitch, float *roll);

#ifdef __cplusplus
}
#endif

#endif
