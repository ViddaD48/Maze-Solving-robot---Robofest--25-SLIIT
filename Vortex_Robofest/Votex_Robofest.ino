#include <Arduino.h>
#include <WiFi.h>
#include "encoders.h"
#include "sensors.h"
#include "gyro.h"
#include "wifi_handler.h"
#include <vector>

std::vector<int> directions;

// ------ Constants and Hardware Pins ------
const int LED_BUILTIN = 2;
const int PWM_A = 25;
const int PWM_B = 13;   
const int INA1 = 26;
const int INA2 = 27;
const int INB1 = 14;
const int INB2 = 12;
const int RESET_PIN = 23;

// ------ Constants & Global variables ------
const int MAX_SPEED = 200;
const int MIN_SPEED = 50;
const float WHEEL_DIAMETER = 3.2;
int lastEncoderCell = 0;

unsigned long lastTime = 0;
unsigned long stuckTimer = 0;
const unsigned long STUCK_TIMEOUT = 5000; // 5 seconds timeout

const float wheelCircumference = 3.14159 * WHEEL_DIAMETER;
const float distPerEncoderStep = wheelCircumference / SPR;

bool isTurning = false;
bool justTurned180 = false;

int sensorFront;
int sensorRight;
int sensorLeft;

std::vector<std::pair<int, int>> compressedPath;
int idx = 0;

String directionToString(int direction)
{
    switch (direction)
    {
    case 0: return "N";
    case 1: return "E";
    case 2: return "S";
    case 3: return "W";
    default: return "?";
    }
}

// ------ Motor Class ------
class Motors
{
public:
    void setLeftMotorPWM(int pwm)
    {
        if (pwm == 0)
        {
            digitalWrite(INA1, LOW);
            digitalWrite(INA2, LOW);
            analogWrite(PWM_A, 0);
        }
        else if (pwm > 0)
        {
            pwm = constrain(pwm, MIN_SPEED, MAX_SPEED);
            digitalWrite(INA1, LOW);
            digitalWrite(INA2, HIGH);
            analogWrite(PWM_A, pwm);
        }
        else
        {
            pwm = constrain(pwm, -MAX_SPEED, -MIN_SPEED);
            digitalWrite(INA1, HIGH);
            digitalWrite(INA2, LOW);
            analogWrite(PWM_A, abs(pwm));
        }
    }

    void setRightMotorPWM(int pwm)
    {
        if (pwm == 0)
        {
            digitalWrite(INB1, LOW);
            digitalWrite(INB2, LOW);
            analogWrite(PWM_B, 0);
        }
        else if (pwm > 0)
        {
            pwm = constrain(pwm, MIN_SPEED, MAX_SPEED);
            digitalWrite(INB1, LOW);
            digitalWrite(INB2, HIGH);
            analogWrite(PWM_B, pwm);
        }
        else
        {
            pwm = constrain(pwm, -MAX_SPEED, -MIN_SPEED);
            digitalWrite(INB1, HIGH);
            digitalWrite(INB2, LOW);
            analogWrite(PWM_B, abs(pwm));
        }
    }

    void brake_abruptly(int delay_time)
    {
        digitalWrite(INA1, HIGH);
        digitalWrite(INA2, HIGH);
        digitalWrite(INB1, HIGH);
        digitalWrite(INB2, HIGH);
        delay(delay_time);
        digitalWrite(INA1, LOW);
        digitalWrite(INA2, LOW);
        digitalWrite(INB1, LOW);
        digitalWrite(INB2, LOW);
    }

    void stopMotors()
    {
        digitalWrite(INA1, LOW);
        digitalWrite(INA2, LOW);
        digitalWrite(INB1, LOW);
        digitalWrite(INB2, LOW);
        analogWrite(PWM_A, 0);
        analogWrite(PWM_B, 0);
    }
};

Sensors sensors;
Motors motors;
float current_angle_gyro = 0.0;
float previous_angle_gyro = 0.0;

#define N 16

int maze[N][N];
unsigned char walls[N][N];

const int NORTH = 0x1;
const int EAST = 0x2;
const int SOUTH = 0x4;
const int WEST = 0x8;

enum GameStates
{
    WAITING_FOR_START,
    CALIBRATING_START_POSITION,
    MAPPING,
    POSITIONING_FOR_RETURN_TO_START,
    RETURN_TO_START,
    POSITIONING_FOR_SPEEDRUN,
    WAITING_FOR_SPEEDRUN,
    SPEEDRUN,
    END,
    ERROR
};

enum MouseStates
{
    MOVING_FORWARD,
    TURNING_IN_GIVEN_DIRECTION,
    RUN,
    STOPPED
};

int posX = 0;
int posY = 0;
int dir = 0;
int startX = 0;
int startY = 0;
int startDir = 0;

std::vector<std::pair<int,int>> goals = {{7,7}, {7,8}, {8,7}, {8,8}};

void setWall(int x, int y, int direction);
void initializeMaze();
void updateWalls();
void floodFillMultipleGoals();
int getNextDirection();
void moveToNextCell(int nextDir, int nbSteps = 1);
void printMazeMatrix();
void resetMazeRun();
void detectStartCorner();

void printMazeMatrix()
{
    Serial.println("=== WALLS ===");
    for (int y = N-1; y >= 0; y--)
    {
        for (int x = 0; x < N; x++)
        {
            Serial.print(walls[x][y]);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    Serial.println("=== DISTANCES ===");
    for (int y = N-1; y >= 0; y--)
    {
        for (int x = 0; x < N; x++)
        {
            if (maze[x][y] == N * N) {
                Serial.print("X ");
            } else {
                Serial.print(maze[x][y]);
                Serial.print(" ");
            }
        }
        Serial.println();
    }
    Serial.println("########");
}

void initializeMaze()
{
    for (int x = 0; x < N; x++)
    {
        for (int y = 0; y < N; y++)
        {
            maze[x][y] = N * N;
            walls[x][y] = 0;
        }
    }
    // Set outer walls
    for (int i = 0; i < N; i++)
    {
        walls[i][0] |= SOUTH;
        walls[i][N - 1] |= NORTH;
        walls[0][i] |= WEST;
        walls[N - 1][i] |= EAST;
    }
}

void updateWalls(int sensorFront, int sensorRight, int sensorLeft)
{
    int x = posX;
    int y = posY;
    int d = dir;

    // Take stable sensor readings
    delay(150);
    int frontSensorReal = 0;
    int rightSensorReal = 0;
    int leftSensorReal = 0;
    
    // Average multiple readings
    for (int i = 0; i < 5; i++)
    {
        int f = sensors.getFrontDistance();
        int r = sensors.getRightDistance();
        int l = sensors.getLeftDistance();
        
        if (f > 0 && f < 300) frontSensorReal = f;
        if (r > 0 && r < 300) rightSensorReal = r;
        if (l > 0 && l < 300) leftSensorReal = l;
        
        delay(30);
    }

    Serial.print("Sensors @ (");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(") Dir:");
    Serial.print(directionToString(d));
    Serial.print(" F:");
    Serial.print(frontSensorReal);
    Serial.print(" R:");
    Serial.print(rightSensorReal);
    Serial.print(" L:");
    Serial.println(leftSensorReal);

    // Front wall detection
    if (frontSensorReal > 0 && frontSensorReal < STOP_CENTER_THRESHOLD)
    {
        Serial.print("Wall FRONT detected, facing ");
        Serial.println(directionToString(d));
        
        if (d == 0 && y < N - 1) { 
            setWall(x, y, NORTH); 
            setWall(x, y + 1, SOUTH); 
        }
        else if (d == 1 && x < N - 1) { 
            setWall(x, y, EAST); 
            setWall(x + 1, y, WEST); 
        }
        else if (d == 2 && y > 0) { 
            setWall(x, y, SOUTH); 
            setWall(x, y - 1, NORTH); 
        }
        else if (d == 3 && x > 0) { 
            setWall(x, y, WEST); 
            setWall(x - 1, y, EAST); 
        }
    }

    // Right wall detection
    if (rightSensorReal > 0 && rightSensorReal < SIDE_THRESHOLD)
    {
        if (d == 0 && x < N - 1) { 
            setWall(x, y, EAST); 
            setWall(x + 1, y, WEST); 
        }
        else if (d == 1 && y > 0) { 
            setWall(x, y, SOUTH); 
            setWall(x, y - 1, NORTH); 
        }
        else if (d == 2 && x > 0) { 
            setWall(x, y, WEST); 
            setWall(x - 1, y, EAST); 
        }
        else if (d == 3 && y < N - 1) { 
            setWall(x, y, NORTH); 
            setWall(x, y + 1, SOUTH); 
        }
    }

    // Left wall detection
    if (leftSensorReal > 0 && leftSensorReal < SIDE_THRESHOLD)
    {
        if (d == 0 && x > 0) { 
            setWall(x, y, WEST); 
            setWall(x - 1, y, EAST); 
        }
        else if (d == 1 && y < N - 1) { 
            setWall(x, y, NORTH); 
            setWall(x, y + 1, SOUTH); 
        }
        else if (d == 2 && x < N - 1) { 
            setWall(x, y, EAST); 
            setWall(x + 1, y, WEST); 
        }
        else if (d == 3 && y > 0) { 
            setWall(x, y, SOUTH); 
            setWall(x, y - 1, NORTH); 
        }
    }
}

void setWall(int x, int y, int direction)
{
    if (x >= 0 && x < N && y >= 0 && y < N) {
        walls[x][y] |= direction;
    }
}

void floodFillMultipleGoals(std::vector<std::pair<int,int>> goals)
{
    // Reset all distances
    for (int x = 0; x < N; x++)
        for (int y = 0; y < N; y++)
            maze[x][y] = N * N;

    // Set goal cells to 0
    for (auto g : goals)
        maze[g.first][g.second] = 0;

    bool changed = true;
    int iterations = 0;
    while (changed && iterations < 100) // Prevent infinite loops
    {
        changed = false;
        iterations++;
        
        for (int x = 0; x < N; x++)
        {
            for (int y = 0; y < N; y++)
            {
                if (maze[x][y] == 0) continue; // Skip goal cells
                
                int minNeighbour = maze[x][y];
                
                // Check all 4 directions
                if (!(walls[x][y] & NORTH) && y < N - 1 && maze[x][y + 1] + 1 < minNeighbour)
                    minNeighbour = maze[x][y + 1] + 1;
                if (!(walls[x][y] & EAST) && x < N - 1 && maze[x + 1][y] + 1 < minNeighbour)
                    minNeighbour = maze[x + 1][y] + 1;
                if (!(walls[x][y] & SOUTH) && y > 0 && maze[x][y - 1] + 1 < minNeighbour)
                    minNeighbour = maze[x][y - 1] + 1;
                if (!(walls[x][y] & WEST) && x > 0 && maze[x - 1][y] + 1 < minNeighbour)
                    minNeighbour = maze[x - 1][y] + 1;

                if (minNeighbour < maze[x][y])
                {
                    maze[x][y] = minNeighbour;
                    changed = true;
                }
            }
        }
    }
    
    Serial.print("FloodFill completed in ");
    Serial.print(iterations);
    Serial.println(" iterations");
}

bool isGoal(int x, int y)
{
    return ((x == 3 && y == 3) || (x == 3 && y == 4) ||
            (x == 4 && y == 3) || (x == 4 && y == 4));
}

int getNextDirection()
{
    int x = posX;
    int y = posY;
    int currentDist = maze[x][y];
    int minDist = currentDist;
    int nextDir = -1;

    Serial.print("Finding next from (");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(") Dist:");
    Serial.print(currentDist);
    Serial.print(" Walls:");
    Serial.println(walls[x][y], BIN);

    // Check all 4 directions
    // NORTH (0): y+1
    if (!(walls[x][y] & NORTH) && y < N - 1) {
        Serial.print("  N:");
        Serial.print(maze[x][y + 1]);
        if (maze[x][y + 1] < minDist) {
            minDist = maze[x][y + 1];
            nextDir = 0;
            Serial.print(" *");
        }
        Serial.println();
    }
    // EAST (1): x+1
    if (!(walls[x][y] & EAST) && x < N - 1) {
        Serial.print("  E:");
        Serial.print(maze[x + 1][y]);
        if (maze[x + 1][y] < minDist) {
            minDist = maze[x + 1][y];
            nextDir = 1;
            Serial.print(" *");
        }
        Serial.println();
    }
    // SOUTH (2): y-1
    if (!(walls[x][y] & SOUTH) && y > 0) {
        Serial.print("  S:");
        Serial.print(maze[x][y - 1]);
        if (maze[x][y - 1] < minDist) {
            minDist = maze[x][y - 1];
            nextDir = 2;
            Serial.print(" *");
        }
        Serial.println();
    }
    // WEST (3): x-1
    if (!(walls[x][y] & WEST) && x > 0) {
        Serial.print("  W:");
        Serial.print(maze[x - 1][y]);
        if (maze[x - 1][y] < minDist) {
            minDist = maze[x - 1][y];
            nextDir = 3;
            Serial.print(" *");
        }
        Serial.println();
    }

    Serial.print("Selected: ");
    Serial.println(directionToString(nextDir));

    return nextDir;
}

void moveToNextCell(int nextDir, int nbSteps)
{
    // Update direction
    dir = nextDir;

    // Update position
    for (int i = 0; i < nbSteps; i++)
    {
        if (dir == 0) posY++;      // NORTH
        else if (dir == 1) posX++; // EAST
        else if (dir == 2) posY--; // SOUTH
        else if (dir == 3) posX--; // WEST
    }
    
    // Keep position within bounds
    posX = constrain(posX, 0, N - 1);
    posY = constrain(posY, 0, N - 1);
    
    Serial.print("Moved to (");
    Serial.print(posX);
    Serial.print(",");
    Serial.print(posY);
    Serial.print(") facing ");
    Serial.println(directionToString(dir));
}

float nextDirToAngle(int nextDir)
{
    int deltaDir = (nextDir - dir + 4) % 4;
    float targetAngle = 0;
    
    if (deltaDir == 1) targetAngle = 90;       // Turn right
    else if (deltaDir == 3) targetAngle = -90; // Turn left
    else if (deltaDir == 2) targetAngle = 180; // Turn around
    else targetAngle = 0;                       // Go straight
    
    Serial.print("Turn from ");
    Serial.print(directionToString(dir));
    Serial.print(" to ");
    Serial.print(directionToString(nextDir));
    Serial.print(" = ");
    Serial.print(targetAngle);
    Serial.println("°");
    
    return targetAngle;
}

enum GameStates gameState = WAITING_FOR_START;
enum MouseStates mouseState = STOPPED;

void resetMazeRun()
{
    motors.stopMotors();
    initializeMaze();
    posX = 0;
    posY = 0;
    dir = 0;
    startX = 0;
    startY = 0;
    startDir = 0;
    encoderValueLeft = 0;
    encoderValueRight = 0;
    lastEncoderCell = 0;
    idx = 0;
    directions.clear();
    compressedPath.clear();
    gameState = WAITING_FOR_START;
    mouseState = STOPPED;
    stuckTimer = 0;
    Serial.println("=== RESET COMPLETE ===");
}

void detectStartCorner()
{
    Serial.println("=== DETECTING START CORNER ===");
    
    delay(300);
    
    // Take multiple readings
    int frontDist = 0;
    int rightDist = 0;
    int leftDist = 0;
    
    for (int i = 0; i < 5; i++)
    {
        int f = sensors.getFrontDistance();
        int r = sensors.getRightDistance();
        int l = sensors.getLeftDistance();
        
        if (f > 0 && f < 300) frontDist = f;
        if (r > 0 && r < 300) rightDist = r;
        if (l > 0 && l < 300) leftDist = l;
        
        delay(50);
    }
    
    Serial.print("Sensor readings - F:");
    Serial.print(frontDist);
    Serial.print(" R:");
    Serial.print(rightDist);
    Serial.print(" L:");
    Serial.println(leftDist);
    
    bool frontWall = (frontDist > 0 && frontDist < STOP_CENTER_THRESHOLD);
    bool rightWall = (rightDist > 0 && rightDist < SIDE_THRESHOLD);
    bool leftWall = (leftDist > 0 && leftDist < SIDE_THRESHOLD);
    
    int wallCount = (frontWall ? 1 : 0) + (rightWall ? 1 : 0) + (leftWall ? 1 : 0);
    
    Serial.print("Walls: F=");
    Serial.print(frontWall);
    Serial.print(" R=");
    Serial.print(rightWall);
    Serial.print(" L=");
    Serial.print(leftWall);
    Serial.print(" Total=");
    Serial.println(wallCount);
    
    // Determine starting position based on walls
    if (frontWall && rightWall && !leftWall) {
        posX = N - 1;
        posY = 0;
        dir = 3; // WEST
        Serial.println("Config: Bottom-Right (7,0) facing WEST");
    }
    else if (frontWall && leftWall && !rightWall) {
        posX = 0;
        posY = 0;
        dir = 1; // EAST
        Serial.println("Config: Bottom-Left (0,0) facing EAST");
    }
    else if (leftWall && rightWall && !frontWall) {
        posX = 0;
        posY = 0;
        dir = 0; // NORTH
        Serial.println("Config: Bottom (0,0) facing NORTH");
    }
    else if (frontWall && !leftWall && !rightWall) {
        posX = 0;
        posY = 0;
        dir = 1; // EAST
        Serial.println("Config: Edge (0,0) facing EAST");
    }
    else if (rightWall && !frontWall && !leftWall) {
        posX = N - 1;
        posY = 0;
        dir = 0; // NORTH
        Serial.println("Config: Right edge (7,0) facing NORTH");
    }
    else if (leftWall && !frontWall && !rightWall) {
        posX = 0;
        posY = 0;
        dir = 0; // NORTH
        Serial.println("Config: Left edge (0,0) facing NORTH");
    }
    else {
        posX = 0;
        posY = 0;
        dir = 0; // NORTH
        Serial.println("Config: Default (0,0) facing NORTH");
    }
    
    startX = posX;
    startY = posY;
    startDir = dir;
    
    Serial.print("Start: (");
    Serial.print(posX);
    Serial.print(",");
    Serial.print(posY);
    Serial.print(") facing ");
    Serial.println(directionToString(dir));
}

void performSimpleTurn(float targetAngle)
{
    if (abs(targetAngle) < 1) {
        Serial.println("No turn needed");
        return;
    }

    Serial.print("Turning ");
    Serial.print(targetAngle);
    Serial.println("°");
    
    int targetLeftSteps, targetRightSteps;
    
    if (targetAngle == 90) {
        targetLeftSteps = +241;
        targetRightSteps = 0;
    }
    else if (targetAngle == -90) {
        targetLeftSteps = 0;
        targetRightSteps = +241;
    }
    else if (abs(targetAngle) == 180) {
        targetLeftSteps = 492;   // Double the 90° steps
        targetRightSteps = -492; // Double the 90° steps
        
        encoderValueLeft = 0;
        encoderValueRight = 0;
        
        int pwm = 100;
        motors.setLeftMotorPWM(pwm);
        motors.setRightMotorPWM(-pwm);
        
        unsigned long turnStart = millis();
        while (abs(encoderValueLeft) < abs(targetLeftSteps) || 
               abs(encoderValueRight) < abs(targetRightSteps)) {
            
            if (millis() - turnStart > 4000) {
                Serial.println("180° turn timeout!");
                break;
            }
            delay(10);
        }
        
        motors.brake_abruptly(200);
        delay(300);
        Serial.println("180° turn complete");
        return;
    }
    else {
        return;
    }
    
    encoderValueLeft = 0;
    encoderValueRight = 0;
    
    int pwm = 100;
    motors.setLeftMotorPWM(targetLeftSteps > 0 ? pwm : -pwm);
    motors.setRightMotorPWM(targetRightSteps > 0 ? pwm : -pwm);
    
    unsigned long turnStart = millis();
    while (abs(encoderValueLeft) < abs(targetLeftSteps) || 
           abs(encoderValueRight) < abs(targetRightSteps)) {
        
        // Timeout protection
        if (millis() - turnStart > 3000) {
            Serial.println("Turn timeout!");
            break;
        }
        delay(10);
    }
    
    motors.brake_abruptly(150);
    delay(250);
    
    Serial.println("Turn complete");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== MICROMOUSE STARTING ===");
    
    initializeMaze();
    //initWiFi();

    Wire.begin();
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(RESET_PIN, INPUT_PULLUP);

    pinMode(PWM_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);
    pinMode(INA1, OUTPUT);
    pinMode(INA2, OUTPUT);
    pinMode(INB1, OUTPUT);
    pinMode(INB2, OUTPUT);

    initEncoders();
    initGyro();
    sensors.initSensors();
    
    delay(500);

    Serial.println("Place hand in front to start...");
    
    while (gameState == WAITING_FOR_START)
    {
        int frontDist = (int)sensors.getFrontDistance();
        if (frontDist > 0 && frontDist < 100)
        {
            Serial.println("START TRIGGERED!");
            gameState = CALIBRATING_START_POSITION;
            delay(1000);
        }
        delay(100);
    }
    
    if (gameState == CALIBRATING_START_POSITION)
    {
        detectStartCorner();
        gameState = MAPPING;
        delay(1000);
    }
    
    floodFillMultipleGoals(goals);
    printMazeMatrix();
    delay(1000);
    
    stuckTimer = millis();
}

void loop()
{
    //handleWiFi();
    
    // Reset button check
    if (digitalRead(RESET_PIN) == LOW)
    {
        Serial.println("RESET PRESSED");
        resetMazeRun();
        delay(1000);
        return;
    }

    // Stuck detection - if in same state too long
    if (gameState == MAPPING || gameState == RETURN_TO_START || gameState == SPEEDRUN) {
        if (mouseState != STOPPED && millis() - stuckTimer > STUCK_TIMEOUT) {
            Serial.println("STUCK DETECTED - STOPPING");
            motors.stopMotors();
            mouseState = STOPPED;
            gameState = ERROR;
            return;
        }
    }

    sensorFront = sensors.getFrontDistance();
    sensorRight = sensors.getRightDistance();
    sensorLeft = sensors.getLeftDistance();

    if (gameState == MAPPING)
    {
        Serial.print("MAPPING - State: ");
        Serial.println(mouseState == STOPPED ? "STOPPED" : "MOVING");
        
        if (mouseState == STOPPED)
        {
            stuckTimer = millis(); // Reset stuck timer
            
            if (isGoal(posX, posY))
            {
                Serial.println("=== GOAL REACHED ===");
                motors.stopMotors();
                updateWalls(sensorFront, sensorRight, sensorLeft);
                printMazeMatrix();

                // Turn around to face back toward start
                performSimpleTurn(180);
                dir = (dir + 2) % 4;  // Update direction (opposite direction)
                justTurned180 = true; 
    
                encoderValueLeft = 0;
                encoderValueRight = 0;
                gameState = RETURN_TO_START;
                mouseState = STOPPED;
                delay(2000);

            }
            else
            {
                encoderValueLeft = 0;
                encoderValueRight = 0;
                motors.stopMotors();
                delay(200); // Settle time

                updateWalls(sensorFront, sensorRight, sensorLeft);
                floodFillMultipleGoals(goals);
                printMazeMatrix();

                int nextDir = getNextDirection();
                if (nextDir == -1)
                {
                    Serial.println("ERROR: No path found!");
                    motors.stopMotors();
                    gameState = ERROR;
                }
                else
                {
                    float targetAngle = nextDirToAngle(nextDir);
                    
                    if (abs(targetAngle) > 1) {
                        mouseState = TURNING_IN_GIVEN_DIRECTION;
                        performSimpleTurn(targetAngle);
                        delay(200);
                    }
                    
                    moveToNextCell(nextDir);
                    
                    encoderValueLeft = 0;
                    encoderValueRight = 0;
                    mouseState = MOVING_FORWARD;
                    stuckTimer = millis(); // Start movement timer
                }
            }
        }
        else if (mouseState == MOVING_FORWARD)
        {
            int avgEnc = (abs(encoderValueLeft) + abs(encoderValueRight)) / 2;
            
            const int BASE_SPEED = 70;
            
            // Check if reached target distance OR front wall
            if (avgEnc >= CM_18_ENCODER_STEPS || (sensorFront > 0 && sensorFront < 60))
            {
                motors.brake_abruptly(150);
                delay(100);
                mouseState = STOPPED;
                Serial.print("Cell reached - Enc:");
                Serial.println(avgEnc);
            }
            else
            {
                motors.setLeftMotorPWM(BASE_SPEED);
                motors.setRightMotorPWM(BASE_SPEED);
            }
        }
    }
    else if (gameState == RETURN_TO_START)
    {
        Serial.println("RETURN_TO_START");
        
        if (mouseState == STOPPED)
        {
            stuckTimer = millis();
            
            if (posX == startX && posY == startY)
            {
                Serial.println("=== BACK AT START ===");
                motors.stopMotors();
                gameState = POSITIONING_FOR_SPEEDRUN;
                mouseState = STOPPED;
                justTurned180 = false;
                delay(2000);
            }
            else
            {
            encoderValueLeft = 0;
            encoderValueRight = 0;
            motors.stopMotors();
            delay(200);

            // Only update walls and recalculate if we're not coming from 180° turn
            if (justTurned180) {
                Serial.println("Moving straight after 180° turn");
                justTurned180 = false;  // Clear flag after first move
                
                // Move to next cell in current direction (no turn needed)
                moveToNextCell(dir);
                
                encoderValueLeft = 0;
                encoderValueRight = 0;
                mouseState = MOVING_FORWARD;
                stuckTimer = millis();
            }
            else {
                // Normal path planning
                updateWalls(sensorFront, sensorRight, sensorLeft);
                std::vector<std::pair<int,int>> startGoal = {{startX, startY}};
                floodFillMultipleGoals(startGoal);

                int nextDir = getNextDirection();
                if (nextDir == -1)
                {
                    Serial.println("ERROR in return!");
                    motors.stopMotors();
                    gameState = ERROR;
                }
                else
                {
                    float targetAngle = nextDirToAngle(nextDir);
                    
                    if (abs(targetAngle) > 1) {
                        mouseState = TURNING_IN_GIVEN_DIRECTION;
                        performSimpleTurn(targetAngle);
                        delay(200);
                    }
                    
                    moveToNextCell(nextDir);
                    
                    encoderValueLeft = 0;
                    encoderValueRight = 0;
                    mouseState = MOVING_FORWARD;
                    stuckTimer = millis();
                }
            }
        }
        }
        else if (mouseState == MOVING_FORWARD)
        {
            int avgEnc = (abs(encoderValueLeft) + abs(encoderValueRight)) / 2;
            const int BASE_SPEED = 70;
            
            if (avgEnc >= CM_18_ENCODER_STEPS || (sensorFront > 0 && sensorFront < 60))
            {
                motors.brake_abruptly(150);
                delay(100);
                mouseState = STOPPED;
            }
            else
            {
                motors.setLeftMotorPWM(BASE_SPEED);
                motors.setRightMotorPWM(BASE_SPEED);
            }
        }
    }
    else if (gameState == POSITIONING_FOR_SPEEDRUN)
    {
        Serial.println("POSITIONING_FOR_SPEEDRUN");
        
        if (mouseState == STOPPED)
        {
            posX = startX;
            posY = startY;
            dir = startDir;

            floodFillMultipleGoals(goals);
            
            int firstDir = getNextDirection();
            if (firstDir != -1 && firstDir != dir) {
                float targetAngle = nextDirToAngle(firstDir);
                performSimpleTurn(targetAngle);
                dir = firstDir;
            }
            
            encoderValueLeft = 0;
            encoderValueRight = 0;
            
            delay(500);
            gameState = WAITING_FOR_SPEEDRUN;
            mouseState = STOPPED;
            Serial.println("=== READY FOR SPEEDRUN ===");
            Serial.println("Place hand in front to start...");
        }
    }
    else if (gameState == WAITING_FOR_SPEEDRUN)
    {
        int frontDist = (int)sensors.getFrontDistance();
        if (frontDist > 0 && frontDist < 100)
        {
            Serial.println("=== SPEEDRUN STARTING ===");
            delay(1000);
            posX = startX;
            posY = startY;
            gameState = SPEEDRUN;
            mouseState = STOPPED;
            stuckTimer = millis();
        }
        delay(100);
    }
    else if (gameState == SPEEDRUN)
    {
        Serial.println("SPEEDRUN");
        
        if (mouseState == STOPPED)
        {
            stuckTimer = millis();
            
            if (isGoal(posX, posY))
            {
                Serial.println("=== SPEEDRUN COMPLETE ===");
                motors.stopMotors();
                gameState = END;
            }
            else
            {
                encoderValueLeft = 0;
                encoderValueRight = 0;
                motors.stopMotors();
                delay(150);

                floodFillMultipleGoals(goals);

                int nextDir = getNextDirection();
                if (nextDir == -1)
                {
                    Serial.println("ERROR in speedrun!");
                    motors.stopMotors();
                    gameState = ERROR;
                }
                else
                {
                    float targetAngle = nextDirToAngle(nextDir);
                    
                    if (abs(targetAngle) > 1) {
                        mouseState = TURNING_IN_GIVEN_DIRECTION;
                        performSimpleTurn(targetAngle);
                        delay(150);
                    }
                    
                    moveToNextCell(nextDir);
                    
                    encoderValueLeft = 0;
                    encoderValueRight = 0;
                    mouseState = MOVING_FORWARD;
                    stuckTimer = millis();
                }
            }
        }
        else if (mouseState == MOVING_FORWARD)
        {
            int avgEnc = (abs(encoderValueLeft) + abs(encoderValueRight)) / 2;
            
            const int SPEEDRUN_SPEED = 90;
            
            if (avgEnc >= CM_18_ENCODER_STEPS || (sensorFront > 0 && sensorFront < 60))
            {
                motors.brake_abruptly(200);
                delay(100);
                mouseState = STOPPED;
            }
            else
            {
                motors.setLeftMotorPWM(SPEEDRUN_SPEED);
                motors.setRightMotorPWM(SPEEDRUN_SPEED);
            }
        }
    }
    else if (gameState == ERROR)
    {
        Serial.println("=== ERROR STATE ===");
        Serial.println("Press RESET button to restart");
        motors.stopMotors();
        
        // Blink LED rapidly to indicate error
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(200);
    }
    else if (gameState == END)
    {
        Serial.println("=== MISSION COMPLETE ===");
        motors.stopMotors();
        
        // Victory blink sequence
        for (int i = 0; i < 5; i++)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
        
        Serial.println("Press RESET to run again");
        delay(5000);
    }
    
    delay(10);
}