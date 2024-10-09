#include "ina260.h"
#include "i2c.h"
#include <string.h>

// Track energy consumption
static uint32_t start_time_ms;
static float total_energy_Wh = 0.0f;
static INA260_Config myConfig = {.Value = DEFAULT_CONFIG};
static INA260_Mask myMask = {.Value = DEFAULT_MASK};

// Buffers for DMA or regular I2C usage
extern uint8_t txBuffer[TX_LEN] __ATTR_DMA_BUFFER;
extern uint8_t rxBuffer[RX_LEN] __ATTR_DMA_BUFFER;

// Helper function to write a 16-bit register (MSB first)
static HAL_StatusTypeDef write_register(uint8_t reg, uint16_t value) {
    // Send MSB first, followed by LSB
    txBuffer[0] = (uint8_t)(value >> 8);  // MSB
    txBuffer[1] = (uint8_t)(value & 0xFF);  // LSB

#ifdef I2C_USE_DMA
    // Use DMA for writing
    return I2C_DMA_Write_MEM(INA260_I2C_ADDRESS, reg, txBuffer, BUFFER_LEN);
#else
    // Use regular I2C for writing
    return HAL_I2C_Mem_Write(&hi2c1, INA260_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, txBuffer, BUFFER_LEN, TIMEOUT_MS);
#endif
}

// Helper function to read a 16-bit register (MSB first)
static HAL_StatusTypeDef read_register(uint8_t reg, uint16_t *value) {
    HAL_StatusTypeDef status;

#ifdef INA260_USE_DMA
    // Use DMA for reading
    status = I2C_DMA_Read_MEM((uint16_t)INA260_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT);
#else
    // Use regular I2C for reading
    status = HAL_I2C_Mem_Read(&hi2c1, INA260_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, rxBuffer, BUFFER_LEN, TIMEOUT_MS);
#endif

    if (status == HAL_OK) {
        // Reconstruct the 16-bit value (MSB first)
        *value = (rxBuffer[0] << 8) | rxBuffer[1];  // MSB first, then LSB
    }

    return status;
}

// Initialize INA260
HAL_StatusTypeDef INA260_Init(void) {
    uint16_t mfg_id, dev_id;
    HAL_StatusTypeDef status;

    // Check manufacturer and device IDs
    status = INA260_GetManufacturerID(&mfg_id);
    if (status != HAL_OK || mfg_id != DEFAULT_MFG_ID) return HAL_ERROR;

    status = INA260_GetDieID(&dev_id);
    if (status != HAL_OK || dev_id != DEFAULT_DEV_ID) return HAL_ERROR;

    // Reset the device
    INA260_Reset();
    //INA260_Start_HighSpeed_Mode();
    // Set default configuration (continuous current and voltage measurement)
    status = INA260_SetConfig(&myConfig);
    if (status != HAL_OK) return HAL_ERROR;

    // Set mask/enable configuration
    status = INA260_SetMaskEnable(&myMask);
    if (status != HAL_OK) return HAL_ERROR;

    // Read back the configuration to ensure it is set correctly
    status = INA260_GetConfig(&myConfig);
    status = INA260_GetMaskEnable(&myMask);
    return status;
}

// Reset INA260
HAL_StatusTypeDef INA260_Reset(void) {
    // Set the reset bit (RST)
    myConfig.BitField.RST = 1;

    // Write the configuration back to trigger the reset
    HAL_StatusTypeDef status = INA260_SetConfig(&myConfig);
    if (status != HAL_OK) {
        return status;  // Return error if writing configuration fails
    }

    // Reset the reset bit (RST)
    myConfig.BitField.RST = 0;
    HAL_Delay(10);  // 10ms delay (may adjust based on your timing needs)

    // INA260 should now be reset to default values
    return HAL_OK;
}

HAL_StatusTypeDef INA260_Start_HighSpeed_Mode(void) {
    HAL_StatusTypeDef status;
    uint8_t hs_master_code = 0x08 << 1;  // High-speed master code (00001000 << 1) for write

#ifdef INA260_USE_DMA
    // Step 1: Send start condition and high-speed master code in fast mode (max 400 kHz)
    status = I2C_DMA_Write_MEM(INA260_I2C_ADDRESS, hs_master_code, I2C_MEMADD_SIZE_8BIT, 0);
#else
    // Step 1: Send start condition and high-speed master code in fast mode (max 400 kHz)
    status = HAL_I2C_Master_Transmit(&hi2c1, hs_master_code, NULL, 0, HAL_MAX_DELAY);
#endif

    if (status != HAL_OK) {
        return status;  // Return if sending the high-speed master code fails
    }

    // Step 2: Generate a repeated start condition to enter high-speed mode
    // We are not sending data but transitioning the protocol into high-speed mode.
#ifdef INA260_USE_DMA
    status = I2C_DMA_Write_MEM(INA260_I2C_ADDRESS, INA260_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, 0);  // Dummy write for repeated start
#else
    status = HAL_I2C_Master_Sequential_Transmit_IT(&hi2c1, INA260_I2C_ADDRESS, NULL, 0, I2C_FIRST_FRAME);
#endif

    if (status != HAL_OK) {
        return status;  // Return if repeated start fails
    }

    // Now the master can communicate at high-speed (up to 2.94 MHz)
    return HAL_OK;
}

// Get INA260 configuration
HAL_StatusTypeDef INA260_GetConfig(INA260_Config *config) {
    return read_register(INA260_REG_CONFIG, &config->Value);
}

// Set INA260 configuration
HAL_StatusTypeDef INA260_SetConfig(INA260_Config *config) {
    return write_register(INA260_REG_CONFIG, config->Value);
}

// Get current in amperes
HAL_StatusTypeDef INA260_GetCurrent(float *current_A) {
    uint16_t raw_current;
    HAL_StatusTypeDef status = read_register(INA260_REG_CURRENT, &raw_current);
    if (status == HAL_OK) {
        // Handle two's complement by casting to int16_t
        *current_A = (int16_t)raw_current * INA260_LSB_CURRENT;
    }
    return status;
}

// Get bus voltage in volts
HAL_StatusTypeDef INA260_GetVoltage(float *voltage_V) {
    uint16_t raw_voltage;
    HAL_StatusTypeDef status = read_register(INA260_REG_VOLTAGE, &raw_voltage);
    if (status == HAL_OK) {
        // No sign bit (bit 15 is always zero), so treat as unsigned
        *voltage_V = raw_voltage * INA260_LSB_VOLTAGE;
    }
    return status;
}

// Get power in watts
HAL_StatusTypeDef INA260_GetPower(float *power_W) {
    uint16_t raw_power;
    HAL_StatusTypeDef status = read_register(INA260_REG_POWER, &raw_power);
    if (status == HAL_OK) {
        *power_W = (float)raw_power * INA260_LSB_POWER;
    }
    return status;
}

// Get Mask/Enable register
HAL_StatusTypeDef INA260_GetMaskEnable(INA260_Mask *mask) {
    return read_register(INA260_REG_MASK_EN, &mask->Value);
}

// Set Mask/Enable register
HAL_StatusTypeDef INA260_SetMaskEnable(INA260_Mask *mask) {
    return write_register(INA260_REG_MASK_EN, mask->Value);
}

// Get alert limit
HAL_StatusTypeDef INA260_GetAlertLimit(uint16_t *limit) {
    return read_register(INA260_REG_ALRT_LIMIT, limit);
}

// Set alert limit
HAL_StatusTypeDef INA260_SetAlertLimit(uint16_t limit) {
    return write_register(INA260_REG_ALRT_LIMIT, limit);
}

// Get Manufacturer ID
HAL_StatusTypeDef INA260_GetManufacturerID(uint16_t *id) {
    return read_register(INA260_REG_MFG_ID, id);
}

// Get Die ID
HAL_StatusTypeDef INA260_GetDieID(uint16_t *die_id) {
    return read_register(INA260_REG_DEV_ID, die_id);
}

// Measure energy consumption (in watt-hours)
HAL_StatusTypeDef INA260_MeasureEnergy(float power_W, float *energy_Wh) {
    uint32_t current_time_ms = HAL_GetTick();
    float time_diff_h = (current_time_ms - start_time_ms) / MS_TO_HOURS;

    total_energy_Wh += power_W * time_diff_h;
    *energy_Wh = total_energy_Wh;

    start_time_ms = current_time_ms;
    return HAL_OK;
}

// Handle interrupt, capture current, voltage, power, and energy
void INA260_HandleInterrupt(float *measurements) {
    if (INA260_GetMaskEnable(&myMask) == HAL_OK && myMask.BitField.CONVERSION_READY) {
        INA260_GetCurrent(&measurements[0]);  // Current (A)
        INA260_GetVoltage(&measurements[1]);  // Voltage (V)
        INA260_GetPower(&measurements[2]);    // Power (W)
        INA260_MeasureEnergy(measurements[2], &measurements[3]);  // Energy (Wh)
    }
}
