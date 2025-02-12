#include "common.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(main);

// Define SPI instance
const nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(3);  // Using SPIM3

void setup_SPI(imu_device_t *imu)
{
    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(BSP_SPI_CLK, BSP_SPI_MOSI, BSP_SPI_MISO, imu->cs_pin);
    spim_config.ss_pin    = imu->cs_pin;
    spim_config.miso_pin  = BSP_SPI_MISO;
    spim_config.mosi_pin  = BSP_SPI_MOSI;
    spim_config.sck_pin   = BSP_SPI_CLK;

    spim_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    spim_config.frequency = NRF_SPIM_FREQ_32M;
    spim_config.mode      = NRF_SPIM_MODE_0;
    
    static bool spi_initialized = false;
    if (!spi_initialized) {
        APP_ERROR_CHECK(nrfx_spim_init(&m_spi, &spim_config, NULL, NULL));
        spi_initialized = true;
    }
}

int8_t bhi360_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    imu_device_t *imu = (imu_device_t *)intf_ptr;
    uint8_t m_tx_buf[1];
    uint8_t m_rx_buf[length + 1];
    size_t s_tx_buf = sizeof(m_tx_buf);
    size_t s_rx_buf = sizeof(m_rx_buf);

    volatile uint32_t * p_spim_event_end = (uint32_t *) nrfx_spim_end_event_get(&m_spi);

    memset(reg_data, 0xff, length);
    memset(m_tx_buf, 0xff, s_tx_buf);
    memset(m_rx_buf, 0xff, s_rx_buf);

    m_tx_buf[0] = reg_addr | 0x80;

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, s_tx_buf, m_rx_buf, s_rx_buf);

    int err_code = nrfx_spim_xfer(&m_spi, &xfer_desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err_code);

    if (err_code == NRFX_SUCCESS)
    {
        while (*p_spim_event_end == 0) {};
        *p_spim_event_end = 0; 
    }

    nrf_gpio_pin_set(imu->cs_pin);
    memcpy(reg_data, &m_rx_buf[1], length);

    return BHY2_INTF_RET_SUCCESS;
}

int8_t bhi360_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    imu_device_t *imu = (imu_device_t *)intf_ptr;
    uint8_t m_tx_buf[length + 1];

    volatile uint32_t * p_spim_event_end = (uint32_t *) nrfx_spim_end_event_get(&m_spi);

    memset(m_tx_buf, 0xff, length + 1);
    m_tx_buf[0] = reg_addr;
    memcpy(m_tx_buf + 1, reg_data, length);

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(m_tx_buf, length + 1);

    int err_code = nrfx_spim_xfer(&m_spi, &xfer_desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err_code);

    if (err_code == NRFX_SUCCESS)
    {
        while (*p_spim_event_end == 0) {};
        *p_spim_event_end = 0; 
    }

    nrf_gpio_pin_set(imu->cs_pin);
    return BHY2_INTF_RET_SUCCESS;
}

const char *get_api_error(int8_t error_code)
{
    switch (error_code)
    {
        case BHY2_OK: return "BHY2_OK";
        case BHY2_E_NULL_PTR: return "BHY2_E_NULL_PTR";
        case BHY2_E_INVALID_PARAM: return "BHY2_E_INVALID_PARAM";
        case BHY2_E_IO: return "BHY2_E_IO";
        case BHY2_E_MAGIC: return "BHY2_E_MAGIC";
        case BHY2_E_TIMEOUT: return "BHY2_E_TIMEOUT";
        case BHY2_E_BUFFER: return "BHY2_E_BUFFER";
        default: return "Unknown error code";
    }
}

const char *get_sensor_error_text(uint8_t sensor_error)
{
    switch (sensor_error)
    {
        case 0: return "No error";
        default: return "Unknown sensor error";
    }
}

const char *get_sensor_name(uint8_t sensor_id)
{
    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_RV: return "Rotation Vector";
        default: return "Unknown sensor";
    }
} 