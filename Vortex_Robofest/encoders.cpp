#include "Encoders.h"

volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;

void initEncoders() {
    pinMode(SIGNAL_FEEDBACK_YELLOW_LEFT, INPUT);
    pinMode(SIGNAL_FEEDBACK_GREEN_LEFT, INPUT);
    pinMode(SIGNAL_FEEDBACK_YELLOW_RIGHT, INPUT);
    pinMode(SIGNAL_FEEDBACK_GREEN_RIGHT, INPUT);

    attachInterrupt(digitalPinToInterrupt(SIGNAL_FEEDBACK_YELLOW_LEFT), updateEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_FEEDBACK_YELLOW_RIGHT), updateEncoderRight, RISING);
}

// ISR for the left encoder
void updateEncoderLeft()
{
    if (digitalRead(SIGNAL_FEEDBACK_YELLOW_LEFT) > digitalRead(SIGNAL_FEEDBACK_GREEN_LEFT))
        encoderValueLeft++;
    else
        encoderValueLeft--;
}

// ISR for the right encoder
void updateEncoderRight()
{
    if (digitalRead(SIGNAL_FEEDBACK_YELLOW_RIGHT) > digitalRead(SIGNAL_FEEDBACK_GREEN_RIGHT))
        encoderValueRight--; // Reverse direction
    else
        encoderValueRight++;
}