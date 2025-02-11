#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include "bhy2.h"
#include "bhy2_parse.h"
#include "common.h"

#define BHY2_RD_WR_LEN          256 
#define WORK_BUFFER_SIZE        2048

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define BHI360_I2C_ADDR 0x28  // BHI360 I2C address
#define I2C_NODE DT_NODELABEL(i2c0)

/* Uncomment to upload firmware to flash instead of RAM */
/*#define UPLOAD_FIRMWARE_TO_FLASH*/

#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "BHY2-Sensor-API/firmware/bhi360/BHI260AP-flash.fw.h"
#else
#include "BHY2-Sensor-API/firmware/bhi360/BHI360_Aux_BMM150.fw.h"
#endif

#define WORK_BUFFER_SIZE  2048
#define QUAT_SENSOR_ID    BHY2_SENSOR_ID_RV  // Use Rotation Vector sensor ID
#define LACC_SENSOR_ID    BHY2_SENSOR_ID_ACC  // Use Linear Acceleration sensor ID

// Global device structures
static const struct device *i2c_dev;
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

// I2C read callback for BHY2 driver
static int8_t bhi360_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    const struct device *i2c = (const struct device *)intf_ptr;
    int ret;

    ret = i2c_write_read(i2c, BHI360_I2C_ADDR, &reg_addr, 1, reg_data, length);
    if (ret < 0) {
        LOG_ERR("Failed to read I2C data: %d", ret);
        return BHY2_E_IO;
    }

    return BHY2_OK;
}

// I2C write callback for BHY2 driver
static int8_t bhi360_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    const struct device *i2c = (const struct device *)intf_ptr;
    uint8_t tmp_buff[length + 1];
    int ret;

    tmp_buff[0] = reg_addr;
    memcpy(&tmp_buff[1], reg_data, length);

    ret = i2c_write(i2c, tmp_buff, length + 1, BHI360_I2C_ADDR);
    if (ret < 0) {
        LOG_ERR("Failed to write I2C data: %d", ret);
        return BHY2_E_IO;
    }

    return BHY2_OK;
}

// Delay function for BHY2 driver
static void bhi360_delay_us(uint32_t period_us, void *intf_ptr)
{
    k_usleep(period_us);
}

void main(void)
{
    uint8_t product_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;

    LOG_INF("Starting BHI360 firmware upload application");

    // Initialize I2C interface
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return;
    }

    // Initialize BHY2 device
    rslt = bhy2_init(BHY2_I2C_INTERFACE,
                     bhi360_i2c_read,
                     bhi360_i2c_write,
                     bhi360_delay_us,
                     BHY2_RD_WR_LEN,
                     (void *)i2c_dev,
                     &bhy2);
    print_api_error(rslt, &bhy2);

    rslt = bhy2_soft_reset(&bhy2);
    print_api_error(rslt, &bhy2);

    rslt = bhy2_get_product_id(&product_id, &bhy2);
    print_api_error(rslt, &bhy2);

    /* Check for a valid product ID */
    if (product_id != BHY2_PRODUCT_ID) {
        LOG_ERR("Product ID read %X. Expected %X", product_id, BHY2_PRODUCT_ID);
        return;
    }
    
    LOG_INF("BHI360 found. Product ID read %X", product_id);

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
    print_api_error(rslt, &bhy2);

    /* Check if the sensor is ready to load firmware */
    rslt = bhy2_get_boot_status(&boot_status, &bhy2);
    print_api_error(rslt, &bhy2);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
        uint8_t sensor_error;
        int8_t temp_rslt;
        LOG_INF("Loading firmware");

#ifdef UPLOAD_FIRMWARE_TO_FLASH
        if (boot_status & BHY2_BST_FLASH_DETECTED) {
            uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
            uint32_t end_addr = start_addr + sizeof(bhy2_firmware_image);
            LOG_INF("Flash detected. Erasing flash to upload firmware");

            rslt = bhy2_erase_flash(start_addr, end_addr, &bhy2);
            print_api_error(rslt, &bhy2);
        } else {
            LOG_ERR("Flash not detected");
            return;
        }
#endif

        rslt = upload_firmware(&bhy2);
        temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
        if (sensor_error) {
            LOG_ERR("Sensor error: %d", sensor_error);
        }

        print_api_error(rslt, &bhy2);
        print_api_error(temp_rslt, &bhy2);

#ifdef UPLOAD_FIRMWARE_TO_FLASH
        LOG_INF("Booting from Flash");
        rslt = bhy2_boot_from_flash(&bhy2);
#else
        LOG_INF("Booting from RAM");
        rslt = bhy2_boot_from_ram(&bhy2);
#endif

        temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
        if (sensor_error) {
            LOG_ERR("Sensor error: %d", sensor_error);
        }

        print_api_error(rslt, &bhy2);
        print_api_error(temp_rslt, &bhy2);

        rslt = bhy2_get_kernel_version(&version, &bhy2);
        print_api_error(rslt, &bhy2);
        if ((rslt == BHY2_OK) && (version != 0)) {
            LOG_INF("Boot successful. Kernel version %u", version);
        }

        // Add callback registration after successful boot
        uint8_t work_buffer[WORK_BUFFER_SIZE];
        
        LOG_INF("Registering callbacks...");
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(LACC_SENSOR_ID, parse_linear_acceleration, NULL, &bhy2);
        print_api_error(rslt, &bhy2);

        // Update the callback table
        LOG_INF("Updating virtual sensor list...");
        rslt = bhy2_update_virtual_sensor_list(&bhy2);
        print_api_error(rslt, &bhy2);

        // Configure quaternion sensor
        float sample_rate = 100.0; // Read out data measured at 100Hz
        uint32_t report_latency_ms = 0; // Report immediately
        LOG_INF("Configuring quaternion sensor...");
        rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
        print_api_error(rslt, &bhy2);
        LOG_INF("Enable %s at %.2fHz", "Quaternion", sample_rate);
        LOG_INF("Configuring linear acceleration sensor...");
        rslt = bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
        print_api_error(rslt, &bhy2);
        LOG_INF("Enable %s at %.2fHz", "Linear Acceleration", sample_rate);

        // Check if sensor is available
        LOG_INF("Checking if quaternion sensor is available...");
        if (!bhy2_is_sensor_available(QUAT_SENSOR_ID, &bhy2)) {
            LOG_ERR("Quaternion sensor not available!");
        } else {
                LOG_INF("Quaternion sensor is available");
        }

        LOG_INF("Checking if linear acceleration sensor is available...");
        if (!bhy2_is_sensor_available(LACC_SENSOR_ID, &bhy2)) {
            LOG_ERR("Linear acceleration sensor not available!");
        } else {
            LOG_INF("Linear acceleration sensor is available");
        }

        // Replace the simple while(1) with FIFO processing
        while (1) {
            // Process FIFO data periodically
            LOG_INF("Processing FIFO data...");
            rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
            LOG_INF("FIFO process result: %d", rslt);
            print_api_error(rslt, &bhy2);

            // Check FIFO status
            uint32_t bytes_remaining = 0;
            uint32_t bytes_read = 0;
            rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
            if (rslt == BHY2_OK) {
                LOG_INF("FIFO process complete");
            }

            k_sleep(K_MSEC(100)); // Sleep for 100ms to reduce log spam
        }
    } else {
        LOG_ERR("Host interface not ready. Exiting");
        return;
    }

    LOG_INF("Firmware upload complete");

    while (1) {
        k_sleep(K_MSEC(1000));
    }
}

// Add new callback functions
static void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy2_data_quaternion data;
    uint32_t s, ns;
    LOG_INF("Quaternion callback triggered!"); // Added debug print
    
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
    struct bhy2_data_xyz data;
    bhy2_parse_xyz(callback_info->data_ptr, &data);
    LOG_INF("Linear Acceleration: x: %d, y: %d, z: %d", data.x, data.y, data.z);
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    // ... copy the parse_meta_event function from quaternion.c but replace printf with LOG_INF ...
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
            printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            printf("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            printf("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            printf("%s Algorithm event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            printf("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            printf("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            printf("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            printf("%s FIFO overflow\r\n", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            printf("%s FIFO watermark reached\r\n", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            printf("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            printf("%s Reset event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}
