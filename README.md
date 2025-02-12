# BHI360 IMU Driver for Zephyr RTOS

This repository contains drivers for the BHI360 IMU sensor using SPI interface on the nRF52840 platform running Zephyr RTOS.

## Features

- SPI communication using nrfx_spim driver
- Firmware upload capability (RAM/Flash)
- Quaternion and linear acceleration sensor data
- Configurable sensor sampling rates
- Error handling and logging
- Pin configuration through overlay files

## Important Notes

⚠️ **Initial Setup Required**: 
- Upon cloning, the `src/BHY2-Sensor-API` directory will be empty
- You must extract the contents of `BHY2_SensorAPI-master.zip` into `src/BHY2-Sensor-API/`
- This is a temporary solution; future commits will properly integrate the sensor API

## Project Structure

```
├── src/
│   ├── main.c                 # Current active driver
│   ├── common.h/c            # SPI interface and utilities
│   │   ├── SPI configuration and pin definitions
│   │   ├── SPI read/write functions
│   │   └── Common error handling
│   ├── add_imu.h            # IMU device definitions
│   │   ├── IMU structure definitions
│   │   └── Multiple IMU configuration
│   └── BHY2-Sensor-API/      # Extract BHY2_SensorAPI-master.zip contents here
├── boards/
│   └── nrf52840dk_nrf52840.overlay  # Board-specific pin configurations
├── prj.conf                   # Active project configuration
└── CMakeLists.txt            # Build system configuration
```

## Code Organization

### SPI Implementation (common.h/c)
```c
// SPI pin definitions
#define BSP_SPI_MISO   NRF_GPIO_PIN_MAP(1, 8)
#define BSP_SPI_MOSI   NRF_GPIO_PIN_MAP(0, 30)
#define BSP_SPI_CLK    NRF_GPIO_PIN_MAP(0, 31)

// Core SPI functions
void setup_SPI(imu_device_t *imu);
int8_t bhi360_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhi360_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
```

### IMU Configuration (add_imu.h)
```c
// IMU device configuration
static imu_device_t imu_devices[] = {
    {
        .cs_pin = NRF_GPIO_PIN_MAP(1, 11),  // First IMU
        .name = "IMU_1"
    },
    {
        .cs_pin = NRF_GPIO_PIN_MAP(1, 12),  // Second IMU
        .name = "IMU_2"
    }
};
```

## Setup Instructions

1. Clone the repository
2. Extract `BHY2_SensorAPI-master.zip` to `src/BHY2-Sensor-API/`
3. Choose your communication mode (I2C or SPI)

### I2C Mode Setup
```bash
# Copy I2C main file
cp src/main_i2c.c src/main.c

# Copy I2C overlay
cp overlay_i2c boards/nrf52840dk_nrf52840.overlay

# Use I2C configuration
cp .conf_i2c prj.conf
```

### SPI Mode Setup
SPI mode uses NRFX drivers and supports multiple IMUs on the same SPI bus.

```bash
# Copy SPI main file
cp src/main_SPI.c src/main.c

# Use SPI configuration
cp .conf_spi prj.conf
```

1. Default SPI pins (defined in common.h):
```c
#define BSP_SPI_MISO   NRF_GPIO_PIN_MAP(1, 8)   // P1.08
#define BSP_SPI_MOSI   NRF_GPIO_PIN_MAP(0, 30)  // P0.30
#define BSP_SPI_CLK    NRF_GPIO_PIN_MAP(0, 31)  // P0.31
```

2. For multiple IMUs, modify the `imu_devices` array in `add_imu.h`:
```c
static imu_device_t imu_devices[] = {
    {
        .cs_pin = NRF_GPIO_PIN_MAP(1, 11),  // First IMU CS pin
        .initialized = false,
        .name = "IMU_1"
    },
    // Add more IMUs as needed
};
```

## Building and Running

1. Build the project:
```bash
west build -b nrf52840dk_nrf52840
```

2. Flash to your device:
```bash
west flash
```

## Configuration Files

- `.conf_i2c`: Configuration for I2C mode
- `.conf_spi`: Configuration for SPI mode
- `prj.conf`: Active project configuration (copy from either .conf_i2c or .conf_spi)

## Available Implementations

### I2C Implementation
- Located in `src/main_i2c.c`
- Uses Zephyr I2C drivers
- Single IMU support
- Configuration via overlay file

### SPI Implementation
- Located in `src/main_SPI.c`
- Uses NRFX SPIM drivers
- Multiple IMU support
- Pin configuration in code

## Future Improvements

1. Code cleanup and organization
2. Proper integration of sensor API (remove dependency on zip file)
3. Better documentation
4. Pure Zephyr driver implementation for SPI mode
5. More examples and use cases
6. Performance optimizations
7. Better error handling and recovery mechanisms

