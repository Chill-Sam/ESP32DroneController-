/**
 * @file AHRS.h
 * @brief Declaration of the AHRS class and associated functions.
 *
 * @details
 * This header file contains the declaration of the AHRS class and
 * related functions. The AHRS class provides functionality for
 * handling the attitude and heading reference system for the 
 * ESP32 drone controller.
 *
 * ### Dependencies:
 * - MPUHandler.h: A handler to get data from a MPU6000 chip
 * - QMC5883LCompass.h: A handler to get data from a QMC5883L chip
 * - MahonyAHRS.h: Provides a mahony system to calculate pitch, roll and yaw
 */


#include <QMC5883LCompass.h> // QMC5883LCompass declaration
#include "MPUHandler.h" // MPUHandler declaration
#include <MahonyAHRS.h> // Mahony declaration

/**
 * @class AHRS
 * @brief Attitude and heading reference system
 *
 * @details
 * AHRS is responsible for calculating the attitude and heading reference
 * system for a drone. Provides methods to initialize a MPU6000 and QMC5883L
 * as well as calculating pitch, roll and yaw.
 */
class AHRS {
private:
  float alpha = 0.9;  ///< Smoothing factor (between 0 and 1)
  float gyro_x_filtered = 0.0; ///< Filtered gyro X 
  float gyro_y_filtered = 0.0; ///< Filtered gyro Y
  float gyro_z_filtered = 0.0; ///< Filtered gyro Z

  QMC5883LCompass compass; ///< QMC5883L compass object
  MPUHandler mpu; ///< MPU6000 handler object
  Mahony filter; ///< Mahony filter object
  float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ; ///< Variables for accelerometer, gyroscope and magnetometer in xyz 

public:
  float pitch, roll, yaw; ///< Public pitch, roll and yaw variables

  /**
   * @brief Constructor to initialize AHRS.
   */
  AHRS();

  /**
   * @brief Initializes the attitude and heading reference system
   * 
   * @details
   * Initializes and calibrates the MPU6000 and QMC5883L chip.
   */
  void init();

  /**
   * @brief Reads data from MPU6000 and QMC5883L
   *
   * @details
   * Communicates via the SPI communication interface to update the
   * acceleromter, gyroscope and magnetometer data.
   */
  void readData();

  /** 
   * @brief Updates the pitch, roll and yaw values
   *
   * @details
   * Calculates the pitch, roll and yaw with the most recent read values
   * with a mahony filter to ensure accuracy.
   *
   * @note Does not call readData() on its own and will use the most recently read values.
   */
  void update();
};