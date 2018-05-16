#ifndef VESC_H
#define VESC_H

#include "Arduino.h"
#include "datatypes.h"

void VESC_init();
void VESC_getValues();
void VESC_setRxValueFunc(void(*func)(mc_values *values));

// Setters
void VESC_setDutyCycle(float dutyCycle);
void VESC_setCurrent(float current);
void VESC_setCurrentBrake(float current);
void VESC_setRpm(int rpm);
void VESC_setPos(float pos);
void VESC_setServoPos(float pos);
void VESC_setChuckData(const chuck_data *chuckdata);

// Other functions
void VESC_sendAlive();

// Helpers
const char* VESC_faultToString(mc_fault_code fault);

#endif
