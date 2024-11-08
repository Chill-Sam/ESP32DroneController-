/**
 * @file ConnectionManager.cpp
 * @brief Implementation of the ConnectionManager class and associated functions.
 *
 * @details
 * This file contains the implementation of the ConnectionManager class and
 * related functions. The ConnectionManager class provides functionality for
 * handling the wifi and http requests for the ESP32 drone controller.
 *
 * ### Dependencies:
 * - ConnectionManager.h: Declaration of the ConnectionManager class and its members
 * - WiFi.h: Manages WiFi connections.
 * - HTTPClient.h: Handles HTTP requests.
 */

#include "ConnectionManager.h" // ConnectionManager class and member definitions
#include <WiFi.h> // WiFi connection library
#include <HTTPClient.h> // HTTP request library
#include <string>

/**
 * @brief Constructor to initialize ConnectionManager with WiFi and server details.
 * 
 * @details
 * This constructor initializes the ConnectionManager object by setting its member
 * variables to the given SSID, password and address. 
 *
 * @param ssid The WiFi network SSID.
 * @param password The WiFi network password.
 * @param address The URL of the HTTP server to connect to.
 */
ConnectionManager::ConnectionManager(const char* ssid, const char* password, const char* address, const char* address2) :
  ssid(ssid), password(password), address(address), address2(address2) {}

/**
 * @brief Connects to the specified WiFi network.
 *
 * @details
 * This method attempts to connect to the WiFi network specified during the class instantiation.
 * It will print the connection status to the Serial monitor.
 */
void ConnectionManager::connectToWifi() {
  Serial.print("Attempting WiFi connection to " + String(ssid));
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
}

/**
 * @brief Initializes the HTTP client with the server address.
 *
 * @details
 * Prepares the HTTPClient object to connect to the provided server URL.
 */
void ConnectionManager::connectToWebsite() {
    http.begin(address);
}

/**
 * @brief Fetches the payload from the server address.
 *
 * @details
 * Starts by checking if wifi is connected. Then attemps a HTTP get request to
 * the address and checks that it is HTTP_CODE_OK. Finally if HTTP get request
 * succeded returns a string. If HTTP request fails returns an empty string.
 *
 * @return String The response from the server as a string. Returns an empty string if the request fails.
 */
String ConnectionManager::getPayloadFromAddress() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected.");
    return "";
  }
  http.begin(address);

  int http_code = http.GET();
  if (http_code != HTTP_CODE_OK) {
    Serial.println("HTTP Get Request failed. Code: " + http_code);
    return "";
  }

  http.end();
  return http.getString();
}

void ConnectionManager::postToAddress(float mot1, float mot2, float mot3, float mot4, float pitch, float roll, float yaw) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected.");
    return;
  }

  http.begin(address2);

  http.addHeader("Content-Type", "application/json");
  String jsonPayload = "{\"motors\":{\"1\":" + String(mot1) + 
                         ",\"2\":" + String(mot2) + 
                         ",\"3\":" + String(mot3) + 
                         ",\"4\":" + String(mot4) + 
                         "},\"pos\":{\"pitch\":" + String(pitch) + 
                         ",\"roll\":" + String(roll) + 
                         ",\"yaw\":" + String(yaw) + "}}";

  int httpResponseCode = http.POST(jsonPayload);
  http.end();

  if (httpResponseCode != HTTP_CODE_OK) {
    Serial.println("HTTP Get Request failed. Code: " + httpResponseCode);
    return;
  }
  return;
}
