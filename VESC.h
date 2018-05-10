#ifndef VESC_H
#define VESC_H

#include "Arduino.h"
#include "datatypes.h"

void VESC_init();
void VESC_getValues();
void bldc_val_received(mc_values *val);

#endif
