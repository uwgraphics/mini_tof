/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>
#include <Arduino.h>

#include <Wire.h>
#include "SPI.h"
#include "platform_config.h"
// #include "stm32f4xx.h"

#ifndef DEFAULT_I2C_BUFFER_LEN
  #ifdef ARDUINO_SAM_DUE
    /* FIXME: It seems that an I2C buffer of BUFFER_LENGTH does not work on Arduino DUE. So, we need to decrease the size */
    #define DEFAULT_I2C_BUFFER_LEN (BUFFER_LENGTH - 2)
  #else
    #ifdef BUFFER_LENGTH
      #define DEFAULT_I2C_BUFFER_LEN BUFFER_LENGTH
    #else
      #define DEFAULT_I2C_BUFFER_LEN 32
    #endif
  #endif
#endif

/**
 * @brief Structure VL53LMZ_Platform needs to be filled by the customer,
 * depending on his platform. At least, it contains the VL53L5CX I2C address.
 * Some additional fields can be added, as descriptors, or platform
 * dependencies. Anything added into this structure is visible into the platform
 * layer.
 */

typedef struct {
  uint16_t  address;

  TwoWire *dev_i2c;
  SPIClass *dev_spi;

  int cs_pin;
  uint32_t spi_speed;

  int lpn_pin;

  int i2c_rst_pin;

} VL53LMZ_Platform;

/*
 * @brief The macro below is used to define the number of target per zone sent
 * through I2C. This value can be changed by user, in order to tune I2C
 * transaction, and also the total memory size (a lower number of target per
 * zone means a lower RAM). The value must be between 1 and 4.
 */

#define 	VL53LMZ_NB_TARGET_PER_ZONE		1U

/*
 * @brief The macro below can be used to avoid data conversion into the driver.
 * By default there is a conversion between firmware and user data. Using this macro
 * allows to use the firmware format instead of user format. The firmware format allows
 * an increased precision.
 */

//#define 	VL53LMZ_USE_RAW_FORMAT

/*
 * @brief All macro below are used to configure the sensor output. User can
 * define some macros if he wants to disable selected output, in order to reduce
 * I2C access.
 */

//#define VL53LMZ_DISABLE_AMBIENT_PER_SPAD
//#define VL53LMZ_DISABLE_NB_SPADS_ENABLED
//#define VL53LMZ_DISABLE_NB_TARGET_DETECTED
//#define VL53LMZ_DISABLE_SIGNAL_PER_SPAD
//#define VL53LMZ_DISABLE_RANGE_SIGMA_MM
//#define VL53LMZ_DISABLE_DISTANCE_MM
//#define VL53LMZ_DISABLE_REFLECTANCE_PERCENT
//#define VL53LMZ_DISABLE_TARGET_STATUS
//#define VL53LMZ_DISABLE_MOTION_INDICATOR

/**
 * @param (VL53LMZ_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of value to read.
 * @param (uint8_t) *p_values : Pointer of value to read.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t RdByte(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAddress,
		uint8_t *p_value);

/**
 * @brief Mandatory function used to write one single byte.
 * @param (VL53LMZ_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of value to read.
 * @param (uint8_t) value : Pointer of value to write.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t WrByte(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAddress,
		uint8_t value);

/**
 * @brief Mandatory function used to read multiples bytes.
 * @param (VL53LMZ_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of values to read.
 * @param (uint8_t) *p_values : Buffer of bytes to read.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t RdMulti(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAddress,
		uint8_t *p_values,
		uint32_t size);

/**
 * @brief Mandatory function used to write multiples bytes.
 * @param (VL53LMZ_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of values to write.
 * @param (uint8_t) *p_values : Buffer of bytes to write.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t WrMulti(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAddress,
		uint8_t *p_values,
		uint32_t size);


/**
 * @brief Mandatory function, used to swap a buffer. The buffer size is always a
 * multiple of 4 (4, 8, 12, 16, ...).
 * @param (uint8_t*) buffer : Buffer to swap, generally uint32_t
 * @param (uint16_t) size : Buffer size to swap
 */

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size);
/**
 * @brief Mandatory function, used to wait during an amount of time. It must be
 * filled as it's used into the API.
 * @param (VL53LMZ_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint32_t) TimeMs : Time to wait in ms.
 * @return (uint8_t) status : 0 if wait is finished.
 */

uint8_t WaitMs(
		VL53LMZ_Platform *p_platform,
		uint32_t TimeMs);





#endif	// _PLATFORM_H_
