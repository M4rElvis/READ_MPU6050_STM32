#ifndef STM32_ARDUINO_COMPAT_H
#define STM32_ARDUINO_COMPAT_H

/* Small compatibility layer for the Arduino calls still present in
 * Jeff Rowberg's MPU6050 sources. Target: STM32F103 + STM32 HAL.
 * This does NOT provide Arduino Wire; I2C is implemented in I2Cdev_STM32.cpp.
 */
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* On STM32, const data is directly readable from Flash. */
#ifndef pgm_read_byte
#define pgm_read_byte(address) (*(const uint8_t *)(address))
#endif
#ifndef pgm_read_word
#define pgm_read_word(address) (*(const uint16_t *)(address))
#endif
#ifndef pgm_read_dword
#define pgm_read_dword(address) (*(const uint32_t *)(address))
#endif
#ifndef pgm_read_float
#define pgm_read_float(address) (*(const float *)(address))
#endif

#ifndef delay
#define delay(ms) HAL_Delay((uint32_t)(ms))
#endif

/* The original FIFO helper measures time in microseconds. HAL_GetTick has
 * millisecond resolution; sufficient for its 11 ms timeout in this first port.
 */
static inline uint32_t STM32_Compat_Micros(void)
{
    return HAL_GetTick() * 1000UL;
}
#ifndef micros
#define micros() STM32_Compat_Micros()
#endif

/* Arduino map() used only by the optional calibration helpers. */
static inline long STM32_Compat_Map(long x, long in_min, long in_max,
                                    long out_min, long out_max)
{
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#ifndef map
#define map(x, in_min, in_max, out_min, out_max) \
    STM32_Compat_Map((x), (in_min), (in_max), (out_min), (out_max))
#endif

/* No-op Serial substitute: calibration/debug print methods compile, but do
 * not emit text. Use HAL UART separately when logging is required.
 */
class STM32_Compat_SerialClass {
public:
    void write(uint8_t) {}
    template <typename T> void print(const T &) {}
    template <typename T> void print(const T &, int) {}
    template <typename T> void println(const T &) {}
    template <typename T> void println(const T &, int) {}
};
static STM32_Compat_SerialClass Serial;

#endif /* STM32_ARDUINO_COMPAT_H */
