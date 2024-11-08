/**
 * @file ConnectionManager.h
 * @brief Declaration of the ConnectionManager class and associated functions.
 *
 * @details
 * This header file contains the declaration of the ConnectionManager class and
 * related functions. The ConnectionManager class provides functionality for
 * handling the wifi and http requests for the ESP32 drone controller.
 *
 * ### Dependencies:
 * - WiFi.h: Manages WiFi connections.
 * - HTTPClient.h: Handles HTTP requests.
 */
#include <WiFi.h> // WiFi connection library
#include <HTTPClient.h> // HTTP request library

/**
 * @class ConnectionManager
 * @brief Manages network connections for the ESP32 drone controller
 *
 * @details
 * ConnectionManager is responsible for handling the http requests of the
 * ESP32 drone controller to get control data.
 */
class ConnectionManager {
private:
  const char* ssid;      ///< WiFi SSID to connect to
  const char* password;  ///< WiFi password
  const char* address;   ///< HTTP server URL
  const char* address2;
  HTTPClient http;       ///< HTTP client instance
  HTTPClient http2;       ///< HTTP client instance

public:
  /**
   * @brief Constructor to initialize ConnectionManager with WiFi and server details.
   * 
   * @param ssid The WiFi network SSID.
   * @param password The WiFi network password.
   * @param address The URL of the HTTP server to connect to.
   */
  ConnectionManager(const char* ssid, const char* password, const char* address, const char* address2);

  /**
   * @brief Connects to the specified WiFi network.
   *
   * @details
   * This method attempts to connect to the WiFi network specified during the class instantiation.
   * It will print the connection status to the Serial monitor.
   */
  void connectToWifi();

  /**
   * @brief Initializes the HTTP client with the server address.
   *
   * @details
   * Prepares the HTTPClient object to connect to the provided server URL.
   */
  void connectToWebsite();

  /**
   * @brief Fetches the payload from the server address.
   *
   * @details
   * Gets the JSON payload from the address. JSON data has the XY coordinates of
   * both joysticks as well as the on/off switch.
   *
   * @return String The response from the server as a string. Returns an empty string if the request fails.
   */
  String getPayloadFromAddress();

  void postToAddress(float mot1, float mot2, float mot3, float mot4, float pitch, float roll, float yaw);
};
