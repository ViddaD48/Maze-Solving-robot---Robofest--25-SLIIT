#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <WiFi.h>
#include <WebServer.h>
#include <vector>

// ==== External variables from main program ====
extern int posX, posY, dir;
extern int sensorFront, sensorRight, sensorLeft;
extern volatile long encoderValueLeft;
extern volatile long encoderValueRight;
//extern enum GameStates gameState;
//extern enum MouseStates mouseState;
extern std::vector<int> directions;
extern std::vector<std::pair<int,int>> compressedPath;

// ==== Web server object (shared) ====
extern WebServer server;

// ==== WiFi credentials ====
#define WIFI_SSID "FOE_Students"
#define WIFI_PASS "FOE@30st"

// ==== Function prototypes ====
void initWiFi();
void handleWiFi();

#endif