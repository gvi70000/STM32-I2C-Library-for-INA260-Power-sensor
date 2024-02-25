#include "INA260.h"
#include "i2c.h"
#include <stdio.h>

// convert value at addr to little-endian (16-bit)
#define byteSwap(x) ((x << 8) | (x >> 8))
/* --- PRINTF_BYTE_TO_BINARY macro's --- */
#define PRINTF_BINARY_SEPARATOR
#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i)    \
    (((i) & 0x80ll) ? '1' : '0'), \
    (((i) & 0x40ll) ? '1' : '0'), \
    (((i) & 0x20ll) ? '1' : '0'), \
    (((i) & 0x10ll) ? '1' : '0'), \
    (((i) & 0x08ll) ? '1' : '0'), \
    (((i) & 0x04ll) ? '1' : '0'), \
    (((i) & 0x02ll) ? '1' : '0'), \
    (((i) & 0x01ll) ? '1' : '0')
#define PRINTF_BINARY_PATTERN_INT16 \
    PRINTF_BINARY_PATTERN_INT8               PRINTF_BINARY_SEPARATOR              PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BYTE_TO_BINARY_INT16(i) \
    PRINTF_BYTE_TO_BINARY_INT8((i) >> 8),   PRINTF_BYTE_TO_BINARY_INT8(i)
#define PRINTF_BINARY_PATTERN_INT32 \
    PRINTF_BINARY_PATTERN_INT16              PRINTF_BINARY_SEPARATOR              PRINTF_BINARY_PATTERN_INT16
#define PRINTF_BYTE_TO_BINARY_INT32(i) \
    PRINTF_BYTE_TO_BINARY_INT16((i) >> 16), PRINTF_BYTE_TO_BINARY_INT16(i)
#define PRINTF_BINARY_PATTERN_INT64    \
    PRINTF_BINARY_PATTERN_INT32              PRINTF_BINARY_SEPARATOR              PRINTF_BINARY_PATTERN_INT32
#define PRINTF_BYTE_TO_BINARY_INT64(i) \
    PRINTF_BYTE_TO_BINARY_INT32((i) >> 32), PRINTF_BYTE_TO_BINARY_INT32(i)
/* --- end macros --- */

static uint16_t const TIMEOUT_MS = 500;

// INA260 register addresses
static uint8_t const INA260_REG_CONFIG  = 0x00;
static uint8_t const INA260_REG_CURRENT = 0x01;
static uint8_t const INA260_REG_VOLTAGE = 0x02;
static uint8_t const INA260_REG_POWER   = 0x03;
static uint8_t const INA260_REG_MASK_EN = 0x06;
static uint8_t const INA260_REG_ALRTLIM = 0x07;
static uint8_t const INA260_REG_MFG_ID  = 0xFE;
static uint8_t const INA260_REG_DEV_ID  = 0xFF;

// INA260 register LSB values for A, V, W, not mA, MV, mW 
// Same for voltage and current
static float const INA260_LSB_CV	=  0.00125F; //1.25F;
static float const INA260_LSB_POWER	= 0.01F; //10.00F;

static ina260_configuration_t config, newConf;

// -- pre-defined device ID per datasheet */
static ina260_ID_t const INA260_DEVICE_ID = {
    .revision = 0x00,
    .device   = 0x227
};

static ina260_status_t ina260_i2c_read(uint8_t mem_addr, uint16_t *buff_dst);
static ina260_status_t ina260_write_config(void);
static ina260_status_t ina260_read_mask_enable(ina260_mask_enable_t *mask_en);

ina260_status_t ina260_ready(void) {
	ina260_ID_t id;
	ina260_status_t status = ina260_i2c_read(INA260_REG_DEV_ID, &(id.u16));
	if (status != HAL_OK){
		return status; }
	// INA260 returns MSB first
	id.u16 = byteSwap(id.u16);
	if (INA260_DEVICE_ID.u16 != id.u16) {
		return HAL_ERROR; }
	return HAL_OK;
}

ina260_status_t ina260_wait_until_ready(uint32_t timeout) {
	uint32_t start = HAL_GetTick();
	while (ina260_ready() != HAL_OK) {
			if (((HAL_GetTick() - start) > timeout)) {
				return HAL_TIMEOUT;
			}
	}
	return HAL_OK;
}

ina260_status_t ina260_set_config(ina260_operating_type_t operating_type, ina260_operating_mode_t operating_mode, ina260_conversion_time_t current_ctime, ina260_conversion_time_t voltage_ctime, ina260_sample_size_t sample_size) {
	config.type = operating_type;
	config.mode  = operating_mode;
	config.ctime  = current_ctime;
	config.vtime  = voltage_ctime;
	config.ssize  = sample_size;
	return ina260_write_config();
}

ina260_status_t ina260_start(void) {
	config.u16  = DEFAULT_CONF;
	ina260_ID_t id;
	//Initiate High speed - see 8.5.3.3.1 High-Speed I2C Mode in datasheet
	HAL_I2C_Master_Transmit(&hi2c1, INA_ADDR, (uint8_t *)INA_FastMode, 1, TIMEOUT_MS);
	HAL_Delay(1);
	if (ina260_i2c_read(INA260_REG_DEV_ID, &(id.u16)) != HAL_OK){
		return HAL_ERROR;
	} else {
		id.u16 = byteSwap(id.u16);
		if (INA260_DEVICE_ID.u16 != id.u16){
			return HAL_ERROR;
		}
	}
	ina260_write_config();
	return HAL_OK;
}

ina260_status_t ina260_set_operating_type(ina260_operating_type_t operating_type) {
	config.type = operating_type;
	return ina260_write_config();
}

ina260_status_t ina260_set_operating_mode(ina260_operating_mode_t operating_mode) {
	config.mode = operating_mode;
	return ina260_write_config();
}

ina260_status_t ina260_set_conversion_time(ina260_conversion_time_t time) {
	config.ctime = time;
	config.vtime = time;
	return ina260_write_config();
}

ina260_status_t ina260_set_current_conversion_time(ina260_conversion_time_t time) {
	config.ctime = time;
	return ina260_write_config();
}

ina260_status_t ina260_set_voltage_conversion_time(ina260_conversion_time_t time) {
	config.vtime = time;
	return ina260_write_config();
}

ina260_status_t ina260_set_sample_size(ina260_sample_size_t sample_size) {
	config.ssize = sample_size;
	return ina260_write_config();
}

ina260_status_t ina260_reset(uint8_t init) {
	config.reset = 1;
	ina260_status_t status = ina260_write_config();
	if (status != HAL_OK) {
		return status;
	}
	if (init == 0U) {
		config.u16  = DEFAULT_CONF;
		return HAL_OK;
	}
	else {
		return ina260_write_config();
	}
}

ina260_status_t ina260_conversion_ready(void) {
	ina260_mask_enable_t mask_en;
	ina260_status_t status = ina260_read_mask_enable(&mask_en);
	if (status != HAL_OK) {
		return status;
	}
	if (mask_en.conversion_ready == 0U) {
		return HAL_BUSY;
	} else {
		return HAL_OK;
	}
}

ina260_status_t ina260_conversion_start(void) {
//	if (config.mode == mode_Continuous) {
//		return HAL_BUSY;
//	}
	return ina260_write_config();
}

ina260_status_t ina260_get_current(float *current) {
	uint16_t u;
	int16_t c, tmp;
	ina260_status_t status = ina260_i2c_read(INA260_REG_CURRENT, &u);
	if (status != HAL_OK) {
		return status;
	}
	tmp = u;
	tmp = byteSwap(tmp);
	c = (int16_t)byteSwap(u);
	*current = (float)c * INA260_LSB_CV;
	return HAL_OK;
}

ina260_status_t ina260_get_voltage(float *voltage) {
	uint16_t v;
	ina260_status_t status = ina260_i2c_read(INA260_REG_VOLTAGE, &v);
	if (status != HAL_OK) {
		return status;
	}
	v = byteSwap(v);
	*voltage = (float)v * INA260_LSB_CV;
	return HAL_OK;
}

ina260_status_t ina260_get_power(float *power) {
	uint16_t p;
	ina260_status_t status = ina260_i2c_read(INA260_REG_POWER, &p);
	if (status != HAL_OK) {
		return status;
	}
	p = byteSwap(p);
	*power = (float)p * INA260_LSB_POWER;
	return HAL_OK;
}

ina260_status_t ina260_get_reg(uint8_t reg, uint16_t *conf) {
	uint16_t r = 0;
	ina260_status_t status = ina260_i2c_read(reg, &r);
	if (status != HAL_OK) {
		return status;
	}
	*conf = byteSwap(r);
	return HAL_OK;
}

ina260_status_t ina260_set_int(void) {
	ina260_status_t status = ina260_wait_until_ready(TIMEOUT_MS);
	uint16_t p = INA_FastMode;
	newConf.u16 = byteSwap(p);
	if (HAL_OK == status) {
		//status = ina260_i2c_write(INA260_REG_CONFIG, &(config));
		status = HAL_I2C_Mem_Write(&hi2c1, INA_ADDR, (uint16_t)INA260_REG_MASK_EN, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&newConf, BUFFER_LEN, TIMEOUT_MS);
	}
	return status;
}

static ina260_status_t ina260_i2c_read(uint8_t mem_addr, uint16_t *buff_dst) {
	ina260_status_t status;
	while (HAL_BUSY == (status = HAL_I2C_Mem_Read(&hi2c1, INA_ADDR, (uint16_t)mem_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buff_dst, BUFFER_LEN, TIMEOUT_MS))) {
		// should not happen, unless during IRQ routine
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
	}
	return status;
}

static ina260_status_t ina260_write_config(void) {
	ina260_status_t status = ina260_wait_until_ready(TIMEOUT_MS);
	//printf("Config " PRINTF_BINARY_PATTERN_INT16 "\r\n", PRINTF_BYTE_TO_BINARY_INT16(config.u16));
	newConf.u16 = byteSwap(config.u16);
	//printf("Con--- " PRINTF_BINARY_PATTERN_INT16 "\r\n", PRINTF_BYTE_TO_BINARY_INT16(newConf.u16));
	if (HAL_OK == status) {
		//status = ina260_i2c_write(INA260_REG_CONFIG, &(config));
		status = HAL_I2C_Mem_Write(&hi2c1, INA_ADDR, (uint16_t)INA260_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&newConf, BUFFER_LEN, TIMEOUT_MS);
	}
	return status;
}

static ina260_status_t ina260_read_mask_enable(ina260_mask_enable_t *mask_en) {
  ina260_mask_enable_t me;
  ina260_status_t status = ina260_i2c_read(INA260_REG_MASK_EN, &(me.u16));
  if (status != HAL_OK) {
		return status;
	}
  mask_en->u16 = byteSwap(me.u16);
  return HAL_OK;
}