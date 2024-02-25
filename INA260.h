#ifndef INA260_H_
#define INA260_H_

#ifdef __cplusplus
extern "C" {
#endif

	#include <stdint.h>
	#include "stm32h7xx_hal.h"
	// Number of samples to collect
	typedef enum {
		samples_1,		// = 0 (000b) -- default
		samples_4,		// = 1 (001b)
		samples_16,		// = 2 (010b)
		samples_64,		// = 3 (011b)
		samples_128,	// = 4 (100b)
		samples_256,	// = 5 (101b)
		samples_512,	// = 6 (110b)
		samples_1024,	// = 7 (111b)
	} ina260_sample_size_t;

	// Conversion time per sample for current and voltage
	typedef enum {
		time_140us,		// = 0 (000b)
		time_204us,		// = 1 (001b)
		time_332us,		// = 2 (010b)
		time_588us,		// = 3 (011b)
		time_1100us,	// = 4 (100b) -- default (voltage, current)
		time_2116us,	// = 5 (101b)
		time_4156us,	// = 6 (110b)
		time_8244us,	// = 7 (111b)
	} ina260_conversion_time_t;

	// Set measurement mode
	typedef enum {
		mode_Triggered,		// = 0 (000b)
		mode_Continuous,	// = 1 (001b) -- default
	} ina260_operating_mode_t;

	// specifies which measurements are performed for each conversion. you can
	// perform current-only, voltage-only, or both current and voltage. note the
	// bit patterns result in the following equalities:
	// iotShutdown	== 0
	// iotPower		== ( iotVoltage | iotCurrent )
	typedef enum {
		measure_Shutdown,	// = 0 (000b)
		measure_Current,	// = 1 (001b)
		measure_Voltage,	// = 2 (010b)
		measure_Power,		// = 3 (011b) -- default
	} ina260_operating_type_t;

	// format of the DEVICE_ID register (FFh)
	typedef union	{
		uint16_t u16;
		struct {
			uint8_t revision :	4;
			uint16_t device	: 12;
		}__attribute__ ((packed));
	} ina260_ID_t;
	
	typedef union {
		uint16_t u16;
		struct {
			uint8_t  alert_latch_enable : 1; //  0
      uint8_t      alert_polarity : 1; //  1
      uint8_t       math_overflow : 1; //  2
      uint8_t    conversion_ready : 1; //  3
      uint8_t alert_function_flag : 1; //  4
      uint8_t                resv : 5; //  5 -  9
      uint8_t    alert_conversion : 1; // 10
      uint8_t    alert_over_power : 1; // 11
      uint8_t alert_under_voltage : 1; // 12
      uint8_t  alert_over_voltage : 1; // 13
      uint8_t alert_under_current : 1; // 14
      uint8_t  alert_over_current : 1; // 15
  }__attribute__ ((packed));
} ina260_mask_enable_t;

// format of CONFIGURATION register (00h)
typedef union {
		uint16_t u16;
		struct {
			ina260_operating_type_t   type : 2; //  0 -  1
			ina260_operating_mode_t   mode : 1; //  2
			ina260_conversion_time_t ctime : 3; //  3 -  5
			ina260_conversion_time_t vtime : 3; //  6 -  8
			ina260_sample_size_t     ssize : 3; //  9 - 11
			uint8_t                   resv : 3; // 12 - 14
			uint8_t                  reset : 1; // 15
  }__attribute__ ((packed));
} ina260_configuration_t;

	typedef HAL_StatusTypeDef ina260_status_t;
	#define DEFAULT_CONF	0x6127
	#define BUFFER_LEN   	2

	// 7-bit I2C addresses see Table 2 in datasheet
	#define INA_ADDR		(0x40 << 1)
	// Check 8.5.3.3.1 High-Speed I2C Mode in datasheet
	#define INA_FastMode	0x08

	ina260_status_t ina260_ready(void);
	ina260_status_t ina260_wait_until_ready(uint32_t timeout);
	ina260_status_t ina260_set_config(ina260_operating_type_t operating_type, ina260_operating_mode_t operating_mode, ina260_conversion_time_t current_ctime, ina260_conversion_time_t voltage_ctime, ina260_sample_size_t sample_size);
	ina260_status_t ina260_start(void);
	ina260_status_t ina260_set_operating_type(ina260_operating_type_t operating_type);
	ina260_status_t ina260_set_operating_mode(ina260_operating_mode_t operating_mode);
	ina260_status_t ina260_set_conversion_time(ina260_conversion_time_t time);
	ina260_status_t ina260_set_current_conversion_time(ina260_conversion_time_t time);
	ina260_status_t ina260_set_voltage_conversion_time(ina260_conversion_time_t time);
	ina260_status_t ina260_set_sample_size(ina260_sample_size_t sample_size);
	ina260_status_t ina260_reset(uint8_t init);
	ina260_status_t ina260_conversion_ready(void);
	ina260_status_t ina260_conversion_start(void);
	ina260_status_t ina260_get_current(float *current);
	ina260_status_t ina260_get_voltage(float *voltage);
	ina260_status_t ina260_get_power(float *power);
	ina260_status_t ina260_get_reg(uint8_t reg, uint16_t *conf);
	ina260_status_t ina260_set_int(void);
#ifdef __cplusplus
}
#endif

#endif /* INA260_H_ */