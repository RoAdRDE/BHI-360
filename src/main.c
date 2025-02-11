#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <nrfx.h>
#include <hal/nrf_gpio.h>
#include <nrfx_spim.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_reset_reason.h>

#include "bhy2.h"
#include "bhy2_parse.h"
#include "common.h"
#include "nrfx_spim.h"
#include "nrfx_gpiote.h"

#define BHY2_RD_WR_LEN          256 
#define WORK_BUFFER_SIZE        2048

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Uncomment to upload firmware to flash instead of RAM */
/*#define UPLOAD_FIRMWARE_TO_FLASH*/

#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "BHY2-Sensor-API/firmware/bhi360/BHI260AP-flash.fw.h"
#else
#include "BHY2-Sensor-API/firmware/bhi360/BHI360_Aux_BMM150.fw.h"
#endif

#define WORK_BUFFER_SIZE  2048
#define QUAT_SENSOR_ID    BHY2_SENSOR_ID_GAMERV  // Use Game Rotation Vector instead
#define LACC_SENSOR_ID    BHY2_SENSOR_ID_ACC  // Use Linear Acceleration sensor ID

#define MAX_IMU_COUNT 4  // Maximum number of IMUs supported

// Structure to hold IMU specific data
typedef struct {
    uint8_t cs_pin;
    struct bhy2_dev bhy2;
    bool initialized;
    char name[32];  // Friendly name for logging
} imu_device_t;

// Global array of IMU devices - define your CS pins here
static imu_device_t imu_devices[] = {
    {
        .cs_pin = NRF_GPIO_PIN_MAP(1, 11),  // First IMU CS pin (P1.12)
        .initialized = false,
        .name = "IMU_1"
    }
    // {
    //     .cs_pin = NRF_GPIO_PIN_MAP(1, 11),  // Second IMU CS pin (P1.11)
    //     .initialized = false,
    //     .name = "IMU_2"
    // }
};

#define NUM_IMUS (sizeof(imu_devices) / sizeof(imu_devices[0]))

// Global device structures
static const struct device *spi_dev;
static const struct device *cs_gpio_dev;
// static struct spi_config spi_cfg = {
//     .frequency = 8000000,  // Reduced to 8MHz for reliability
//     .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB ,
//     .cs = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(bhi360), 0)
// };

// Use SPI_CS_CONTROL macro for CS control
static struct bhy2_dev bhy2;

// Add new function declarations
static void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_linear_acceleration(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK) {
        LOG_ERR("API error: %d", rslt);
        if ((rslt == BHY2_E_IO) && (dev != NULL)) {
            LOG_ERR("Interface error: %d", dev->hif.intf_rslt);
            dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
        }
    }
}

static int8_t upload_firmware(struct bhy2_dev *dev)
{
    uint32_t incr = 256; /* Max command packet size */
    uint32_t len = sizeof(bhy2_firmware_image);
    int8_t rslt = BHY2_OK;

    if ((incr % 4) != 0) /* Round off to higher 4 bytes */
    {
        incr = ((incr >> 2) + 1) << 2;
    }

    for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
    {
        if (incr > (len - i)) /* If last payload */
        {
            incr = len - i;
            if ((incr % 4) != 0) /* Round off to higher 4 bytes */
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }

#ifdef UPLOAD_FIRMWARE_TO_FLASH
        rslt = bhy2_upload_firmware_to_flash_partly(&bhy2_firmware_image[i], i, incr, dev);
#else
        rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);
#endif

        LOG_INF("%.2f%% complete", (float)(i + incr) / (float)len * 100.0f);
    }

    return rslt;
}

// Add error check macro
#define APP_ERROR_CHECK(err_code) \
    do { \
        if (err_code != NRFX_SUCCESS) { \
            LOG_ERR("Error %d at line %d", err_code, __LINE__); \
        } \
    } while (0)

static const nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(3);  // Using SPIM3

// GPIO pin definitions (using your existing pin numbers)
#define BSP_SPI_MISO   NRF_GPIO_PIN_MAP(1, 8)  
#define BSP_SPI_MOSI   NRF_GPIO_PIN_MAP(0, 30)  
#define BSP_SPI_CLK    NRF_GPIO_PIN_MAP(0, 31)  

static void setup_SPI(imu_device_t *imu)
{
    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(BSP_SPI_CLK, BSP_SPI_MOSI, BSP_SPI_MISO, imu->cs_pin);
    spim_config.ss_pin    = imu->cs_pin;  // Use the IMU-specific CS pin
    spim_config.miso_pin  = BSP_SPI_MISO;
    spim_config.mosi_pin  = BSP_SPI_MOSI;
    spim_config.sck_pin   = BSP_SPI_CLK;

    spim_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    spim_config.frequency = NRF_SPIM_FREQ_32M;
    spim_config.mode      = NRF_SPIM_MODE_0;
    
    // Only initialize SPI hardware once
    static bool spi_initialized = false;
    if (!spi_initialized) {
        APP_ERROR_CHECK(nrfx_spim_init(&m_spi, &spim_config, NULL, NULL));
        spi_initialized = true;
    }
}

static int8_t bhi360_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    imu_device_t *imu = (imu_device_t *)intf_ptr;
    uint8_t m_tx_buf[1];
    uint8_t m_rx_buf[length + 1];
    size_t s_tx_buf = sizeof(m_tx_buf);
    size_t s_rx_buf = sizeof(m_rx_buf);

    volatile uint32_t * p_spim_event_end = (uint32_t *) nrfx_spim_end_event_get(&m_spi);

    // Initialize buffers
    memset(reg_data, 0xff, length);
    memset(m_tx_buf,     0xff, s_tx_buf);
    memset(m_rx_buf,     0xff, s_rx_buf);

    m_tx_buf[0] = reg_addr | 0x80;  // Read

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, s_tx_buf, m_rx_buf, s_rx_buf);

    int err_code = nrfx_spim_xfer(&m_spi, &xfer_desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err_code);

    if (err_code == NRFX_SUCCESS)
    {
      while (*p_spim_event_end == 0)
                {};
      *p_spim_event_end = 0; 
    }
    // The driver doesn't release the ss_pin
    // So we need to do it ourselves here to tell the chip
    // that this SPI transfer is finished.
    nrf_gpio_pin_set(imu->cs_pin);

    memcpy(reg_data, &m_rx_buf[1], length);  // skip past NULL first byte in the reply. Will always be 0x00

    return BHY2_INTF_RET_SUCCESS;
}

static int8_t bhi360_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    imu_device_t *imu = (imu_device_t *)intf_ptr;
    uint8_t m_tx_buf[length + 1];

    volatile uint32_t * p_spim_event_end = (uint32_t *) nrfx_spim_end_event_get(&m_spi);

    // Initialize buffers
    memset(m_tx_buf, 0xff, length + 1);

    m_tx_buf[0] = reg_addr;
    memcpy(m_tx_buf + 1, reg_data, length);

    m_tx_buf[0] = reg_addr;

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(m_tx_buf, length + 1);

    int err_code = nrfx_spim_xfer(&m_spi, &xfer_desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err_code);

    if (err_code == NRFX_SUCCESS)
    {
      while (*p_spim_event_end == 0)
                {};
      *p_spim_event_end = 0; 
    }
    // The driver doesn't release the ss_pin
    // So we need to do it ourselves here to tell the chip
    // that this SPI transfer is finished.
    nrf_gpio_pin_set(imu->cs_pin);

    return BHY2_INTF_RET_SUCCESS;
}
// Delay function for BHY2 driver
static void bhi360_delay_us(uint32_t period_us, void *intf_ptr)
{
    k_usleep(period_us);
}

// Function to initialize a single IMU
static bool initialize_imu(imu_device_t *imu) {
    int8_t rslt;
    uint8_t product_id = 0;
    uint16_t version = 0;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;

    LOG_INF("%s: Starting initialization", imu->name);

    // Configure CS pin
    nrf_gpio_cfg_output(imu->cs_pin);
    nrf_gpio_pin_clear(imu->cs_pin);
    k_sleep(K_USEC(1));

    // Initialize BHY2 device
    rslt = bhy2_init(BHY2_SPI_INTERFACE,
                     bhi360_spi_read,
                     bhi360_spi_write,
                     bhi360_delay_us,
                     BHY2_RD_WR_LEN,
                     imu,  // Pass IMU struct as interface pointer
                     &imu->bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("%s: Initialization failed", imu->name);
        return false;
    }

    // Soft reset
    rslt = bhy2_soft_reset(&imu->bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("%s: Soft reset failed", imu->name);
        return false;
    }

    // Product ID check with retries
    bool id_read_success = false;
    for (int retry = 0; retry < 20; retry++) {
        rslt = bhy2_get_product_id(&product_id, &imu->bhy2);
        if (rslt == BHY2_OK && product_id == BHY2_PRODUCT_ID) {
            LOG_INF("%s: Product ID verified on attempt %d", imu->name, retry + 1);
            id_read_success = true;
            break;
        }
        k_msleep(10);
    }
    
    if (!id_read_success) {
        LOG_ERR("%s: Failed to verify product ID", imu->name);
        return false;
    }

    // Configure FIFO and interrupts
    hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &imu->bhy2);
    
    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &imu->bhy2);

    // Check boot status and upload firmware
    rslt = bhy2_get_boot_status(&boot_status, &imu->bhy2);
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
        LOG_INF("%s: Uploading firmware", imu->name);
        rslt = upload_firmware(&imu->bhy2);
        if (rslt != BHY2_OK) {
            LOG_ERR("%s: Firmware upload failed", imu->name);
            return false;
        }

        // Boot from RAM and verify
        rslt = bhy2_boot_from_ram(&imu->bhy2);
        rslt = bhy2_get_kernel_version(&version, &imu->bhy2);
        if (rslt != BHY2_OK || version == 0) {
            LOG_ERR("%s: Boot failed", imu->name);
            return false;
        }
        LOG_INF("%s: Boot successful, kernel version %u", imu->name, version);

        // Update virtual sensor list first
        LOG_INF("%s: Updating virtual sensor list", imu->name);
        rslt = bhy2_update_virtual_sensor_list(&imu->bhy2);
        print_api_error(rslt, &imu->bhy2);

        // Register callbacks
        LOG_INF("%s: Registering callbacks", imu->name);
        
        // Configure sensors
        float sample_rate = 100.0;
        uint32_t report_latency_ms = 0;

        // Configure quaternion sensor
        LOG_INF("%s: Configuring quaternion sensor...", imu->name);
        rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, &imu->bhy2);
        print_api_error(rslt, &imu->bhy2);
        LOG_INF("%s: Enable Quaternion at %.2fHz", imu->name, sample_rate);

        // Configure linear acceleration sensor
        LOG_INF("%s: Configuring linear acceleration sensor...", imu->name);
        rslt = bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, sample_rate, report_latency_ms, &imu->bhy2);
        print_api_error(rslt, &imu->bhy2);
        LOG_INF("%s: Enable Linear Acceleration at %.2fHz", imu->name, sample_rate);

        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, 
            parse_meta_event, imu, &imu->bhy2);
        print_api_error(rslt, &imu->bhy2);

        // Check quaternion availability
        LOG_INF("%s: Checking quaternion sensor availability...", imu->name);
        if (!bhy2_is_sensor_available(QUAT_SENSOR_ID, &imu->bhy2)) {
            LOG_ERR("%s: Quaternion sensor not available!", imu->name);
        } else {
            LOG_INF("%s: Quaternion sensor is available", imu->name);
            rslt = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, 
                parse_quaternion, imu, &imu->bhy2);
            print_api_error(rslt, &imu->bhy2);
        }

        // Check linear acceleration availability
        LOG_INF("%s: Checking linear acceleration sensor availability...", imu->name);
        if (!bhy2_is_sensor_available(LACC_SENSOR_ID, &imu->bhy2)) {
            LOG_ERR("%s: Linear acceleration sensor not available!", imu->name);
        } else {
            LOG_INF("%s: Linear acceleration sensor is available", imu->name);
            rslt = bhy2_register_fifo_parse_callback(LACC_SENSOR_ID, 
                parse_linear_acceleration, imu, &imu->bhy2);
            print_api_error(rslt, &imu->bhy2);
        }

        imu->initialized = true;
        LOG_INF("%s: Initialization complete", imu->name);
        return true;
    }

    LOG_ERR("%s: Host interface not ready", imu->name);
    return false;
}

void main(void)
{
    LOG_INF("Starting BHI360 firmware upload application");

    // Initialize all IMUs
    for (int i = 0; i < NUM_IMUS; i++) {
        setup_SPI(&imu_devices[i]);  // Setup SPI for each IMU
        initialize_imu(&imu_devices[i]);
    }

    while (1) {
        for (int i = 0; i < NUM_IMUS; i++) {
            if (!imu_devices[i].initialized) {
                continue;
            }

            uint8_t work_buffer[WORK_BUFFER_SIZE];
            int8_t rslt = bhy2_get_and_process_fifo(work_buffer, 
                                                   sizeof(work_buffer),
                                                   &imu_devices[i].bhy2);
            if (rslt != BHY2_OK) {
                LOG_WRN("%s: FIFO processing error", imu_devices[i].name);
            }
        }
        k_msleep(10);
    }
}

// Update callback functions to use IMU context
static void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    imu_device_t *imu = (imu_device_t *)callback_ref;
    struct bhy2_data_quaternion data;
    uint32_t s, ns;
    LOG_INF("%s Quaternion: x: %f, y: %f, z: %f, w: %f",
            imu->name,
            data.x / 16384.0f,
            data.y / 16384.0f,
            data.z / 16384.0f,
            data.w / 16384.0f);
    
    if (callback_info->data_size != 11) { // Check for valid payload size
        LOG_ERR("Invalid data size: %d", callback_info->data_size);
        return;
    }

    bhy2_parse_quaternion(callback_info->data_ptr, &data);

    uint64_t timestamp = *callback_info->time_stamp;
    timestamp = timestamp * 15625; // Convert to nanoseconds
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    LOG_INF("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f, w: %f; acc: %.2f",
            callback_info->sensor_id,
            s,
            ns,
            data.x / 16384.0f,
            data.y / 16384.0f,
            data.z / 16384.0f,
            data.w / 16384.0f,
            ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

static void parse_linear_acceleration(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref) {
    imu_device_t *imu = (imu_device_t *)callback_ref;
    struct bhy2_data_xyz data;
    bhy2_parse_xyz(callback_info->data_ptr, &data);
    LOG_INF("%s Linear Acceleration: x: %d, y: %d, z: %d",
            imu->name,
            data.x,
            data.y,
            data.z);
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    imu_device_t *imu = (imu_device_t *)callback_ref;
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    char *event_text;

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            LOG_INF("%s Flush complete for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            LOG_INF("%s Sample rate changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            LOG_INF("%s Power mode changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            LOG_INF("%s Algorithm event", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            LOG_INF("%s Accuracy for sensor id %u changed to %u", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            LOG_INF("%s BSX event (do steps main)", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            LOG_INF("%s BSX event (do steps calib)", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            LOG_INF("%s BSX event (get output signal)", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            LOG_INF("%s Sensor id %u reported error 0x%02X", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            LOG_INF("%s FIFO overflow", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            LOG_INF("%s Dynamic range changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            LOG_INF("%s FIFO watermark reached", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            LOG_INF("%s Firmware initialized. Firmware version %u", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            LOG_INF("%s Transfer cause for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            LOG_INF("%s Sensor framework event for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            LOG_INF("%s Reset event", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            LOG_INF("%s Unknown meta event with id: %u", event_text, meta_event_type);
            break;
    }
}
