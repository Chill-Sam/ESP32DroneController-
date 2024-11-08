/**
 * @file EngineController.h
 * @brief Declaration of the EngineController class and associated functions.
 *
 * @details
 * This header file contains the declaration of the EngineController class and
 * related functions. The EngineController class provides functionality for
 * handling the four motors of the drone.
 *
 * ### Dependencies:
 * - array: Array library
 */

#include <array>

/**
 * @class EngineController
 *
 * @brief Manages the control of the four engines using electronic speed controllers (ESCs).
 *
 * @details
 * The EngineController class allows precise management of engine speeds through ESCs,
 * utilizing PWM (Pulse Width Modulation). It supports setting individual engine speeds
 * as well as synchronous control of all engines.
 */
class EngineController {
private:
  std::array<const int, 4> escPins; ///< Pins connected to ESCs.
  const int frequency;              ///< PWM frequency.
  const int resolution;             ///< PWM resolution.
  const int minPulseWidth;          ///< Minimum pulse width (0% throttle).
  const int maxPulseWidth;          ///< Maximum pulse width (100% throttle)
  
  /**
   * @brief Converts a pulse width in microseconds to a corresponding duty cycle value.
   *
   * @param pulseWidth The pulse width in microseconds.
   * @return The calculated duty cycle value based on the current resolution.
   */
  float pulseWidthToDutyCycle(float pulseWidth);

public:
  /**
   * @brief Constructs an EngineController with given parameters.
   *
   * @param escPins Array of pin numbers connected to the ESC signal inputs.
   * @param frequency PWM frequency for ESC signaling.
   * @param resolution Bit resolution for PWM signal.
   * @param minPulseWidth Minimum pulse width corresponding to 0% throttle.
   * @param maxPulseWidth Maximum pulse width corresponding to 100% throttle.
   */
  EngineController(std::array<const int, 4> &escPins, const int frequency, const int resolution, const int minPulseWidth, const int maxPulseWidth);
  
  /**
   * @brief Initializes ESCs by setting up PWM channels and attaching them to the specified pins.
   *
   * @details
   * This method must be called before setting engine speeds to ensure that the PWM generators
   * are correctly configured and linked to the correct output pins.
   */
  void initializeESC();

  /**
   * @brief Stops all engines by setting their throttle to the minimum pulse width.
   *
   * @param delayMilliseconds Time in milliseconds to delay after stopping the engines.
   * This is useful for ensuring all operations requiring the stop to complete.
   */
  void stopEngines(int delayMilliseconds);

  /**
   * @brief Sets the speed (throttle) of specified engines.
   *
   * @param engineNumber Engine identifier (1-4 for individual engines, 5 for all engines).
   * @param throttle Throttle percentage from 0% to 100%.
   *
   * @note If an invalid engine number or throttle percentage is specified, an error message is printed.
   */
  void setEngineSpeed(int engineNumber, float throttle);
};
