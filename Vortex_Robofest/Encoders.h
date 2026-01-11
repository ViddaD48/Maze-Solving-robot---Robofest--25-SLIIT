#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

#define SIGNAL_FEEDBACK_YELLOW_LEFT 35
#define SIGNAL_FEEDBACK_GREEN_LEFT 34
#define SIGNAL_FEEDBACK_YELLOW_RIGHT 19
#define SIGNAL_FEEDBACK_GREEN_RIGHT 18

extern volatile long encoderValueLeft;
extern volatile long encoderValueRight;

#define CM_18_ENCODER_STEPS 703 // Steps for 18 cm, 530 RPM : 300, 750 RPM : 205, 750 RPM FELT: 215
#define SPR 353 // Steps per revolution 530 RPM : 205 SPR, 750 RPM : 143 SPR

void initEncoders();
void updateEncoderLeft();
void updateEncoderRight();

#endif // ENCODERS_H