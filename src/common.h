/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    common.h
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <nrfx_spim.h>
#include <nrfx_gpiote.h>
#include <hal/nrf_gpio.h>
#include <zephyr/logging/log.h>

#include "bhy2.h"

#ifdef __cplusplus
extern "C" {
#endif

// Structure to hold IMU specific data
typedef struct {
    uint8_t cs_pin;
    struct bhy2_dev bhy2;
    bool initialized;
    char name[32];  // Friendly name for logging
} imu_device_t;

// GPIO pin definitions for SPI bus
#define BSP_SPI_MISO   NRF_GPIO_PIN_MAP(1, 8)  
#define BSP_SPI_MOSI   NRF_GPIO_PIN_MAP(0, 30)  
#define BSP_SPI_CLK    NRF_GPIO_PIN_MAP(0, 31)  

// Error check macro
#define APP_ERROR_CHECK(err_code) \
    do { \
        if (err_code != NRFX_SUCCESS) { \
            LOG_ERR("Error %d at line %d", err_code, __LINE__); \
        } \
    } while (0)

const char *get_api_error(int8_t error_code);
const char *get_sensor_error_text(uint8_t sensor_error);
const char *get_sensor_name(uint8_t sensor_id);
void setup_SPI(imu_device_t *imu);
int8_t bhi360_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhi360_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

// Global SPI instance
extern const nrfx_spim_t m_spi;

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H_ */ 