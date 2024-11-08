/**-
 * @file AHRS.cpp
 * @brief Implementation of the AHRS class and associated functions.
 *
 * @details
 * This header file contains the implementation of the AHRS class and
 * related functions. The AHRS class provides functionality for
 * handling the attitude and heading reference system for the 
 * ESP32 drone controller.
 *
 * ### Dependencies:
 * - AHRS.h: Declaration of the AHRS class and its members
 * - MPUHandler.h: A handler to get data from a MPU6000 chip
 * - QMC5883LCompass.h: A handler to get data from a QMC5883L chip
 * - MahonyAHRS.h: Provides a mahony system to calculate pitch, roll and yaw
 */

// #undef twoKpDef
// #undef twoKiDef
// #define twoKpDef	3.0	// 2 * proportional gain
// #define twoKiDef	0.01	// 2 * integral gain

#include "AHRS.h" // AHRS class and member definitions
#include <QMC5883LCompass.h> // QMC5883LCompass declaration
#include <MahonyAHRS.h> // Mahony declaration

#define SAMPLE_FREQ 64 //< Frequency that mahony filter expects data


AHRS::AHRS() {}

/**
 * @brief Initializes the attitude and heading reference systemcompass.setCalibrationOffsets(312.00, 502.00, 312.00);
  compass.setCalibrationScales(1.06, 0.98, 0.94);

 * 
 * @details
 * Starts by initializing the MPU6000 with the MPUHandler object, using the
 * initializeMPU method. Calibrates it with the method calculateOffsets().
 * Then it initializes the compass with the QMC5883LCompass object with the
 * .init() method. Then it starts the mahony filter with the given sample
 * frequency.
 *
 * @see SAMPLE_FREQ
 */
void AHRS::init() {
  mpu.initializeMPU();
  mpu.calculateOffsets();

  compass.init();
  compass.setCalibrationOffsets(312.00, 502.00, 312.00);
  compass.setCalibrationScales(1.06, 0.98, 0.94);

  filter.begin(SAMPLE_FREQ);
}


/**
 * @brief Reads data from MPU6000 and QMC5883L
 *
 * @details
 * Starts by calling the getData() method on the MPUHandler object to get data
 * from the MPU6000 chip. It then reads the compass data with the read() method 
 * and gets X, Y and Z. It does this via the SPI communication protocol. 
 */
void AHRS::readData() {
  std::array<float, 6> data = mpu.getData();
  accelX = data[0];
  accelY = data[1];
  accelZ = data[2];
  gyroX = data[3];
  gyroY = data[4];
  gyroZ = data[5];
  
  gyro_x_filtered = alpha * gyro_x_filtered + (1.0 - alpha) * gyroX;
  gyro_y_filtered = alpha * gyro_y_filtered + (1.0 - alpha) * gyroY;
  gyro_z_filtered = alpha * gyro_z_filtered + (1.0 - alpha) * gyroZ;
  
  Serial.print("GyroX:");
  Serial.println(gyro_x_filtered);
  Serial.print("GyroY:");
  Serial.println(gyro_y_filtered);
  Serial.print("GyroZ:");
  Serial.println(gyro_z_filtered);

  compass.read();
  float tmagX = compass.getX();
  float tmagY = compass.getY();
  float tmagZ = compass.getZ();

  magX = -tmagY;
  magY = magX;
  magZ = -tmagZ;

  Serial.print("MagX:");
  Serial.println(magX);
  Serial.print("MagY:");
  Serial.println(magY);
  Serial.print("MagZ:");
  Serial.println(magZ);
}

/** 
 * @brief Updates the pitch, roll and yaw values
 *
 * @details
 * Uses the mahony algorithm to estimate the pitch, roll and yaw of a
 * system using gyroscope, accelerometer and magnetometer data. 
 */
void AHRS::update() {
  filter.update(accelX, accelY, accelZ, gyro_x_filtered, gyro_y_filtered, gyro_z_filtered, magX, magY, magZ);
  pitch = filter.getPitch();
  roll = filter.getRoll();
  yaw = filter.getYaw();
}
