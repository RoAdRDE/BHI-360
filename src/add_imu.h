static imu_device_t imu_devices[] = {
    {
        .cs_pin = NRF_GPIO_PIN_MAP(1, 11),  // First IMU CS pin (P1.11)
        .initialized = false,
        .name = "IMU_1"
    },
    {
        .cs_pin = NRF_GPIO_PIN_MAP(1, 12),  // Second IMU CS pin (P1.12)
        .initialized = false,
        .name = "IMU_2"
    }
};

#define NUM_IMUS (sizeof(imu_devices) / sizeof(imu_devices[0]))
