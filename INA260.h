#ifndef INA260_H_
#define INA260_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
// Datasheet link
// https://www.ti.com/lit/ds/symlink/ina260.pdf?ts=1725866881912&ref_url=https%253A%252F%252Fwww.google.com%252F

// INA260 LSB constants (A, V, W, not mA, mV, mW)
#define INA260_LSB_CURRENT    0.00125f  // 1.25 mA or 0.00125 A per LSB
#define INA260_LSB_VOLTAGE    0.00125f  // 1.25 mV per LSB
#define INA260_LSB_POWER      0.01f     // 10 mW per LSB

// Time conversion factor from ms to hours
#define MS_TO_HOURS           3600000.0f

// Buffer length for I2C data
#define BUFFER_LEN            2

// Default register configuration
#define DEFAULT_CONFIG        0x6D27	//AVG=512, I=100, V=100, Mode=111
#define DEFAULT_MASK          0x400		//Conversion Ready
#define DEFAULT_MFG_ID        0x5449
#define DEFAULT_DEV_ID        0x2270

// INA260 I2C address (7-bit, shifted)
#define INA260_I2C_ADDRESS    (0x40 << 1)

// Timeout for I2C operations in milliseconds
#define TIMEOUT_MS            100

// INA260 register addresses
#define INA260_REG_CONFIG     0x00
#define INA260_REG_CURRENT    0x01
#define INA260_REG_VOLTAGE    0x02
#define INA260_REG_POWER      0x03
#define INA260_REG_MASK_EN    0x06
#define INA260_REG_ALRT_LIMIT 0x07
#define INA260_REG_MFG_ID     0xFE
#define INA260_REG_DEV_ID     0xFF

// Conversion time options for current and voltage measurements
typedef enum {
    INA260_TIME_140_us   = 0,
    INA260_TIME_204_us   = 1,
    INA260_TIME_332_us   = 2,
    INA260_TIME_588_us   = 3,
    INA260_TIME_1_1_ms   = 4,
    INA260_TIME_2_116_ms = 5,
    INA260_TIME_4_156_ms = 6,
    INA260_TIME_8_244_ms = 7,
} INA260_ConversionTime;

// Averaging count options for INA260
typedef enum {
    INA260_AVG_1     = 0,
    INA260_AVG_4     = 1,
    INA260_AVG_16    = 2,
    INA260_AVG_64    = 3,
    INA260_AVG_128   = 4,
    INA260_AVG_256   = 5,
    INA260_AVG_512   = 6,
    INA260_AVG_1024  = 7,
} INA260_AveragingCount;

// Operating modes for the INA260
typedef enum {
    INA260_MODE_POWERDOWN               = 0,
    INA260_MODE_CURRENT_TRIGGERED       = 1,
    INA260_MODE_VOLTAGE_TRIGGERED       = 2,
    INA260_MODE_CURRENT_VOLTAGE_TRIGGERED = 3,
    INA260_MODE_CURRENT_CONTINUOUS      = 5,
    INA260_MODE_VOLTAGE_CONTINUOUS      = 6,
    INA260_MODE_CURRENT_VOLTAGE_CONTINUOUS = 7,
} INA260_Mode;

// Configuration register (0x00) structure
typedef union {
    uint16_t Value;
    struct __attribute__((packed)){
        INA260_Mode            MODE : 3;
        INA260_ConversionTime  ISHCT : 3;
        INA260_ConversionTime  VBUSCT : 3;
        INA260_AveragingCount  AVG : 3;
        uint16_t RESERVED : 3;
        uint16_t RST : 1;
    } BitField;
} INA260_Config;

// Mask/Enable register (0x06) structure
typedef union {
    uint16_t Value;
    struct __attribute__((packed)){
        uint16_t LATCH_EN : 1;
        uint16_t APOL : 1;
        uint16_t MATH_OVERFLOW : 1;
        uint16_t CONVERSION_READY_EN : 1;
        uint16_t ALERT_FLAG : 1;
        uint16_t RESERVED : 5;
        uint16_t CONVERSION_READY : 1;
        uint16_t POWER_OVER_LIMIT : 1;
        uint16_t UNDER_VOLTAGE : 1;
        uint16_t OVER_VOLTAGE : 1;
        uint16_t UNDER_CURRENT : 1;
        uint16_t OVER_CURRENT : 1;
    } BitField;
} INA260_Mask;

// Function prototypes
HAL_StatusTypeDef INA260_Init(void);
HAL_StatusTypeDef INA260_Start_HighSpeed_Mode(void);
HAL_StatusTypeDef INA260_Reset(void);
HAL_StatusTypeDef INA260_GetConfig(INA260_Config *config);
HAL_StatusTypeDef INA260_SetConfig(INA260_Config *config);
HAL_StatusTypeDef INA260_GetCurrent(float *current_A);
HAL_StatusTypeDef INA260_GetVoltage(float *voltage_V);
HAL_StatusTypeDef INA260_GetPower(float *power_W);
HAL_StatusTypeDef INA260_GetMaskEnable(INA260_Mask *mask);
HAL_StatusTypeDef INA260_SetMaskEnable(INA260_Mask *mask);
HAL_StatusTypeDef INA260_GetAlertLimit(uint16_t *limit);
HAL_StatusTypeDef INA260_SetAlertLimit(uint16_t limit);
HAL_StatusTypeDef INA260_GetManufacturerID(uint16_t *id);
HAL_StatusTypeDef INA260_GetDieID(uint16_t *die_id);
HAL_StatusTypeDef INA260_MeasureEnergy(float power_W, float *energy_Wh);
void INA260_HandleInterrupt(float *measurements);

	
#endif /* INA260_H_ */