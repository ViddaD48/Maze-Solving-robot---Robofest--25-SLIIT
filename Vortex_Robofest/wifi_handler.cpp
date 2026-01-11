#include "wifi_handler.h"

/*// External variables from main program
extern int posX, posY, dir;
extern long encoderValueLeft, encoderValueRight;
extern int gameState, mouseState;
extern std::vector<int> directions;
extern std::vector<int> compressedPath;*/

// Web server on port 80
WebServer server(80);

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='2'></head><body>";
  html += "<h1>Micromouse Status</h1>";
  html += "<p><b>Position:</b> (" + String(posX) + ", " + String(posY) + ")</p>";
  html += "<p><b>Direction:</b> " + String(dir) + "</p>";
  html += "<p><b>Encoder Left:</b> " + String(encoderValueLeft) + "</p>";
  html += "<p><b>Encoder Right:</b> " + String(encoderValueRight) + "</p>";
  html += "<p><b>Sensor Front:</b> " + String(sensorFront) + "</p>";
  html += "<p><b>Sensor Right:</b> " + String(sensorRight) + "</p>";
  html += "<p><b>Sensor Left:</b> " + String(sensorLeft) + "</p>";
  //html += "<p><b>Game State:</b> " + String(gameState) + "</p>";
  //html += "<p><b>Mouse State:</b> " + String(mouseState) + "</p>";

  html += "<p><b>Directions:</b> ";
  for (int d : directions) html += String(d) + " ";
  html += "</p>";

  html += "<p><b>Compressed Path:</b> ";
    for (auto &p : compressedPath) {
      html += "(" + String(p.first) + "," + String(p.second) + ") ";
    }
  html += "</p>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);  // Station mode

  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());  // Shows assigned IP

  // Start web server
  server.on("/", handleRoot);
  server.begin();
}

void handleWiFi() {
  server.handleClient();
}
