#include "stm32f1xx_hal.h"
#include "I2Cdev.h"
#include <stddef.h>

/* The original I2Cdev.h API is retained. wireObj is now an
 * I2C_HandleTypeDef* (for example, &hi2c1), not an Arduino TwoWire object.
 */
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

I2Cdev::I2Cdev() {}

static I2C_HandleTypeDef *getI2C(void *wireObj)
{
    return static_cast<I2C_HandleTypeDef *>(wireObj);
}

static uint32_t halTimeout(uint16_t timeout)
{
    return (timeout == 0U) ? HAL_MAX_DELAY : (uint32_t)timeout;
}

int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                         uint8_t *data, uint16_t timeout, void *wireObj)
{
    I2C_HandleTypeDef *hi2c = getI2C(wireObj);
    if ((hi2c == NULL) || (data == NULL) || (length == 0U)) return -1;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        hi2c,
        (uint16_t)((uint16_t)devAddr << 1),
        (uint16_t)regAddr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        (uint16_t)length,
        halTimeout(timeout)
    );

    return (status == HAL_OK) ? (int8_t)length : -1;
}

int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data,
                        uint16_t timeout, void *wireObj)
{
    return readBytes(devAddr, regAddr, 1U, data, timeout, wireObj);
}

int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                         uint16_t *data, uint16_t timeout, void *wireObj)
{
    if ((data == NULL) || (length == 0U)) return -1;

    uint8_t bytes[2];
    for (uint8_t i = 0; i < length; ++i) {
        uint8_t reg = (uint8_t)(regAddr + (uint8_t)(2U * i));
        if (readBytes(devAddr, reg, 2U, bytes, timeout, wireObj) != 2) {
            return -1;
        }
        data[i] = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    }
    return (int8_t)length;
}

int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data,
                        uint16_t timeout, void *wireObj)
{
    return readWords(devAddr, regAddr, 1U, data, timeout, wireObj);
}

int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                       uint8_t *data, uint16_t timeout, void *wireObj)
{
    return readBits(devAddr, regAddr, bitNum, 1U, data, timeout, wireObj);
}

int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                        uint8_t length, uint8_t *data, uint16_t timeout,
                        void *wireObj)
{
    if ((data == NULL) || (length == 0U) || (length > 8U) || (bitStart > 7U) ||
        (length > (uint8_t)(bitStart + 1U))) return -1;

    uint8_t value = 0;
    if (readByte(devAddr, regAddr, &value, timeout, wireObj) != 1) return -1;

    uint8_t shift = (uint8_t)(bitStart - length + 1U);
    uint16_t mask = (uint16_t)(((1U << length) - 1U) << shift);
    *data = (uint8_t)((value & mask) >> shift);
    return 1;
}

int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                        uint16_t *data, uint16_t timeout, void *wireObj)
{
    return readBitsW(devAddr, regAddr, bitNum, 1U, data, timeout, wireObj);
}

int8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                         uint8_t length, uint16_t *data, uint16_t timeout,
                         void *wireObj)
{
    if ((data == NULL) || (length == 0U) || (length > 16U) || (bitStart > 15U) ||
        (length > (uint8_t)(bitStart + 1U))) return -1;

    uint16_t value = 0;
    if (readWord(devAddr, regAddr, &value, timeout, wireObj) != 1) return -1;

    uint8_t shift = (uint8_t)(bitStart - length + 1U);
    uint32_t baseMask = (length == 16U) ? 0xFFFFUL : ((1UL << length) - 1UL);
    uint16_t mask = (uint16_t)(baseMask << shift);
    *data = (uint16_t)((value & mask) >> shift);
    return 1;
}

bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                        uint8_t *data, void *wireObj)
{
    I2C_HandleTypeDef *hi2c = getI2C(wireObj);
    if ((hi2c == NULL) || (data == NULL) || (length == 0U)) return false;

    return HAL_I2C_Mem_Write(
        hi2c,
        (uint16_t)((uint16_t)devAddr << 1),
        (uint16_t)regAddr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        (uint16_t)length,
        halTimeout(I2Cdev::readTimeout)
    ) == HAL_OK;
}

bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data,
                       void *wireObj)
{
    return writeBytes(devAddr, regAddr, 1U, &data, wireObj);
}

bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                        uint16_t *data, void *wireObj)
{
    if ((data == NULL) || (length == 0U)) return false;

    for (uint8_t i = 0; i < length; ++i) {
        uint8_t bytes[2];
        bytes[0] = (uint8_t)(data[i] >> 8);
        bytes[1] = (uint8_t)(data[i] & 0xFFU);
        uint8_t reg = (uint8_t)(regAddr + (uint8_t)(2U * i));
        if (!writeBytes(devAddr, reg, 2U, bytes, wireObj)) return false;
    }
    return true;
}

bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data,
                       void *wireObj)
{
    return writeWords(devAddr, regAddr, 1U, &data, wireObj);
}

bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                      uint8_t data, void *wireObj)
{
    return writeBits(devAddr, regAddr, bitNum, 1U, data, wireObj);
}

bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                       uint8_t length, uint8_t data, void *wireObj)
{
    if ((length == 0U) || (length > 8U) || (bitStart > 7U) ||
        (length > (uint8_t)(bitStart + 1U))) return false;

    uint8_t value = 0;
    if (readByte(devAddr, regAddr, &value, I2Cdev::readTimeout, wireObj) != 1) return false;

    uint8_t shift = (uint8_t)(bitStart - length + 1U);
    uint16_t mask = (uint16_t)(((1U << length) - 1U) << shift);
    uint8_t newValue = (uint8_t)((value & (uint8_t)(~mask)) | ((data << shift) & mask));
    return writeByte(devAddr, regAddr, newValue, wireObj);
}

bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                       uint16_t data, void *wireObj)
{
    return writeBitsW(devAddr, regAddr, bitNum, 1U, data, wireObj);
}

bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                        uint8_t length, uint16_t data, void *wireObj)
{
    if ((length == 0U) || (length > 16U) || (bitStart > 15U) ||
        (length > (uint8_t)(bitStart + 1U))) return false;

    uint16_t value = 0;
    if (readWord(devAddr, regAddr, &value, I2Cdev::readTimeout, wireObj) != 1) return false;

    uint8_t shift = (uint8_t)(bitStart - length + 1U);
    uint32_t baseMask = (length == 16U) ? 0xFFFFUL : ((1UL << length) - 1UL);
    uint16_t mask = (uint16_t)(baseMask << shift);
    uint16_t newValue = (uint16_t)((value & (uint16_t)(~mask)) | ((data << shift) & mask));
    return writeWord(devAddr, regAddr, newValue, wireObj);
}
