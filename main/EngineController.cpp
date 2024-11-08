/**
 * @file EngineController.cpp
 * @brief Implementation of the EngineController class and associated functions.
 *
 * @details
 * This header file contains the implementation of the EngineController class and
 * related functions. The EngineController class provides functionality for
 * handling the four motors of the drone.
 *
 * ### Dependencies:
 * - EngineController.h: Declaration of EngineController class and its members
 * - array: Array library
 * - math.h: Provides mathematical functions
 */

#include "EngineController.h" // EngineController class and member definitions
#include <array>
#include <math.h>
#include <Arduino.h>

/**
 * @brief Constructs an EngineController with given parameters.
 *
 * @param escPins Array of pin numbers connected to the ESC signal inputs.
 * @param frequency PWM frequency for ESC signaling.
 * @param resolution Bit resolution for PWM signal.
 * @param minPulseWidth Minimum pulse width corresponding to 0% throttle.
 * @param maxPulseWidth Maximum pulse width corresponding to 100% throttle.
 */
EngineController::EngineController(std::array<const int, 4> &escPins, const int frequency, const int resolution, const int minPulseWidth, const int maxPulseWidth) :
  escPins(escPins), frequency(frequency), resolution(resolution), minPulseWidth(minPulseWidth), maxPulseWidth(maxPulseWidth) {}

/**
 * @brief Converts a pulse width in microseconds to a corresponding duty cycle value.
 *
 * @param pulseWidth The pulse width in microseconds.
 * @return The calculated duty cycle value based on the current resolution.
 */
float EngineController::pulseWidthToDutyCycle(float pulseWidth) {
  float maxDutyCycle = pow(2, resolution) - 1;
  return map(pulseWidth, 0, 20000, 0, maxDutyCycle);
}

/**
 * @brief Initializes ESCs by setting up PWM channels and attaching them to the specified pins.
 *
 * @details
 * The method uses the ledcAttach method to attach all the pins defined with escPins. Must be
 * called before using motors.
 *
 * @see EngineController
 */
void EngineController::initializeESC() {
  for (int i = 0; i <= 3; i++) {
    ledcAttach(escPins[i], frequency, resolution);
  }
}

/**
 * @brief Stops all engines by setting their throttle to the minimum pulse width.
 *
 * @details
 * Loops through all escPins[] and writes 0 to them. Then waits for delayMilliseconds.
 *
 * @param delayMilliseconds Time in milliseconds to delay after stopping the engines.
 * This is useful for ensuring all operations requiring the stop to complete.
 */
void EngineController::stopEngines(int delayMilliseconds) {
  for (int i = 0; i <= 3; i++) {
    ledcWrite(escPins[i] , pulseWidthToDutyCycle(minPulseWidth));
  }
  Serial.print("Engines stopped");

  delay(delayMilliseconds);
}

/**
 * @brief Sets the speed (throttle) of specified engines.
 *
 * @details
 * Goes through the different edge cases to make sure it is a valid input. Then writes to escPins[engineNumber -1]
 * for 1-4 and loops through all escPins for 5. Sets them to throttle.
 *
 * @param engineNumber Engine identifier (1-4 for individual engines, 5 for all engines).
 * @param throttle Throttle percentage from 0% to 100%.
 *
 * @note If an invalid engine number or throttle percentage is specified, an error message is printed.
 */
void EngineController::setEngineSpeed(int engineNumber, float throttle) {
  if (engineNumber < 1 || engineNumber > 5 ) {
    Serial.println("Engine number should be 1-5, 1-4 individual, 5 all.");
    return;
  }
  if (throttle < 0 || throttle > 100 ) {
    Serial.println("Throttle should be 0-100.");
    return;
  }

  if (engineNumber >= 1 && engineNumber <= 4) {
    float pulseWidth = map(throttle, 0, 100, minPulseWidth, maxPulseWidth);
    ledcWrite(escPins[engineNumber - 1], pulseWidthToDutyCycle(pulseWidth));
    //Serial.println("Engine " + String(engineNumber) + ": Throttle set to " + String(throttle) + "%");
  }

  else if (engineNumber == 5) {
    for(int i = 0; i <= 3; i++) {
      float pulseWidth = map(throttle, 0, 100, minPulseWidth, maxPulseWidth);
      ledcWrite(escPins[i], pulseWidthToDutyCycle(pulseWidth));
    }
    Serial.println("Engines set to " + String(throttle) + "% throttle");
    
  }
}
