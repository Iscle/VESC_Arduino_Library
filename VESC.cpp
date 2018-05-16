#include "Arduino.h"
#include "VESC.h"

#include <stdio.h>
#include <string.h>

#include "comm_uart.h"
#include "bldc_interface.h"

void VESC_init() {
  comm_uart_init();
}

void VESC_getValues() {
	bldc_interface_get_values();
}

void VESC_setRxValueFunc(void(*func)(mc_values *values)) {
  bldc_interface_set_rx_value_func(func);
}

// Setters
void VESC_setDutyCycle(float dutyCycle) {
  bldc_interface_set_duty_cycle(dutyCycle);
}

void VESC_setCurrent(float current) {
  bldc_interface_set_current(current);
}

void VESC_setCurrentBrake(float current) {
  bldc_interface_set_current_brake(current);
}

void VESC_setRpm(int rpm) {
  bldc_interface_set_rpm(rpm);
}

void VESC_setPos(float pos) {
  bldc_interface_set_pos(pos);
}

void VESC_setServoPos(float pos) {
  bldc_interface_set_servo_pos(pos);
}

void VESC_setChuckData(const chuck_data *chuckdata) {
  bldc_interface_set_chuck_data(chuckdata);
}

// Other functions
void VESC_sendAlive() {
  bldc_interface_send_alive();
}

// Helpers
const char* VESC_faultToString(mc_fault_code fault) {
  return bldc_interface_fault_to_string(fault);
}

