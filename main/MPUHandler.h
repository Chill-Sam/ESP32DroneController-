/**
 * @file MPUHandler.h
 * @brief Declaration of the MPUHandler class and associated functions.
 *
 * @details
 * This header file contains the declaration of the MPUHandler class and
 * related functions. The MPUHandler class provides functionality for
 * handling the MPU6000 chip for the ESP32 drone controller.
 */

#include <cstdint>
#include <array>

/**
 * @class MPUHandler
 *
 * @brief Handles communication with the MPU6000 sensor.
 *
 * @details
 * This class provides comprehensive management of the MPU6000 sensor, including initialization
 * and reading of accelerometer and gyroscope data. It uses the SPI to communicate
 * with the sensor.
 */
class MPUHandler {
private:
  float accelXOffset, accelYOffset, accelZOffset, gyroXOffset, gyroYOffset, gyroZOffset = 0; //< Variables for offsets
  
  /**
   * @brief Writes a byte of data to a specific register of the MPU6000.
   *
   * @param reg The register address to write to.
   * @param value The data byte to write to the register.
   */
  void writeRegister(uint8_t reg, uint8_t value); 

  /**
   * @brief Reads a byte of data from a specific register of the MPU6000.
   *
   * @param reg The register address to read from.
   * @return The byte value read from the register.
   */
  uint8_t readRegister(uint8_t reg);

  /**
   * @brief Reads raw accelerometer and gyroscope data from the MPU6000 sensor.
   *
   * @param rawAccelX Reference to store raw accelerometer X-axis data.
   * @param rawAccelY Reference to store raw accelerometer Y-axis data.
   * @param rawAccelZ Reference to store raw accelerometer Z-axis data.
   * @param rawGyroX Reference to store raw gyroscope X-axis data.
   * @param rawGyroY Reference to store raw gyroscope Y-axis data.
   * @param rawGyroZ Reference to store raw gyroscope Z-axis data.
   */
  void getRawData(int16_t &rawAccelX, int16_t &rawAccelY, int16_t &rawAccelZ, int16_t &rawGyroX, int16_t &rawGyroY, int16_t &rawGyroZ);

public:
  /**
   * @brief Constructor for the MPUHandler class.
   */
  MPUHandler();

  /**
   * @brief Initializes the MPU6000 sensor.
   *
   * @details
   * Sets up SPI communication and configures the sensor.
   */
  void initializeMPU();

  /**
   * @brief Reads accelerometer and gyroscope data from the MPU6000 sensor.
   *
   * @return An array containing accelerometer and gyroscope data: [accelX_mps2, accelY_mps2, accelZ_mps2, gyroX_rads, gyroY_rads, gyroZ_rads].
   */
  std::array<float, 6> getData();

  /**
   * @brief Calculates offsets for the accelerometer and gyroscope data.
   *
   * @details
   * Takes multiple samples of raw data to determine average offsets, which are then used to correct
   * the sensor readings.
   */
  void calculateOffsets();
};