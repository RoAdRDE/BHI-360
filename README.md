# BHI360 IMU Driver for Zephyr RTOS

This repository contains drivers for the BHI360 IMU sensor supporting both I2C and SPI interfaces on the nRF52840 platform running Zephyr RTOS.

## Important Notes

⚠️ **Initial Setup Required**: 
- Upon cloning, the `src/BHY2-Sensor-API` directory will be empty
- You must extract the contents of `BHY2_SensorAPI-master.zip` into `src/BHY2-Sensor-API/`
- This is a temporary solution; future commits will properly integrate the sensor API

🔧 **Work in Progress**:
- Code cleanup is pending
- Better organization of sensor API integration is planned
- Documentation improvements are coming
- More features and optimizations will be added in future commits

## Project Structure

```
├── src/
│   ├── main.c                 # Current active driver
│   ├── main_SPI.c            # SPI mode implementation
│   ├── main_i2c.c            # I2C mode implementation
│   └── BHY2-Sensor-API/      # Extract BHY2_SensorAPI-master.zip contents here
├── boards/
│   └── nrf52840dk_nrf52840.overlay  # Board-specific pin configurations
├── .conf_i2c                  # I2C mode configuration
├── .conf_spi                  # SPI mode configuration
├── prj.conf                   # Active project configuration
└── CMakeLists.txt            # Build system configuration
```

## Setup Instructions

1. Clone the repository
2. Extract `BHY2_SensorAPI-master.zip` to `src/BHY2-Sensor-API/`
3. Choose your communication mode (I2C or SPI)

### I2C Mode Setup
I2C mode uses pure Zephyr drivers with full Zephyr sensor subsystem integration.

1. Replace `src/main.c` with `src/main_i2c.c`:
   ```bash
   cp src/main_i2c.c src/main.c
   ```

2. Copy I2C overlay to your board configuration:
   ```bash
   cp overlay_i2c boards/nrf52840dk_nrf52840.overlay
   ```

3. Use I2C configuration:
   ```bash
   cp .conf_i2c prj.conf
   ```

### SPI Mode Setup
SPI mode uses NRFX drivers and supports multiple IMUs on the same SPI bus.

1. Replace `src/main.c` with `src/main_SPI.c`:
   ```bash
   cp src/main_SPI.c src/main.c
   ```

2. Use SPI configuration:
   ```bash
   cp .conf_spi prj.conf
   ```

3. Default SPI pins:
   ```c
   #define BSP_SPI_MISO   NRF_GPIO_PIN_MAP(1, 8)   // P1.08
   #define BSP_SPI_MOSI   NRF_GPIO_PIN_MAP(0, 30)  // P0.30
   #define BSP_SPI_CLK    NRF_GPIO_PIN_MAP(0, 31)  // P0.31
   ```

4. For multiple IMUs, modify the `imu_devices` array in `main.c`:
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

## Mode Comparison

### I2C Mode
- Full Zephyr driver implementation
- Uses Zephyr sensor subsystem
- Single IMU support
- Configuration in overlay file
- Pure Zephyr approach

### SPI Mode
- Uses NRFX drivers directly
- Multiple IMU support
- Pin configuration in code
- Zephyr driver implementation coming soon
- More flexible for multiple sensors

## Configuration Files

- `.conf_i2c`: Configuration for I2C mode
- `.conf_spi`: Configuration for SPI mode
- `prj.conf`: Active project configuration (copy from either .conf_i2c or .conf_spi)

## Future Improvements

1. Code cleanup and organization
2. Proper integration of sensor API (remove dependency on zip file)
3. Better documentation
4. Pure Zephyr driver implementation for SPI mode
5. More examples and use cases
6. Performance optimizations
7. Better error handling and recovery mechanisms

