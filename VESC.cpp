#include "Arduino.h"
#include "VESC.h"

#include <stdio.h>
#include <string.h>

#include "comm_uart.h"
#include "bldc_interface.h"

void VESC_init() {
  // For the UART interface
  comm_uart_init();

  // Give bldc_interface a function to call when valus are received.
  bldc_interface_set_rx_value_func(bldc_val_received);
}

// Aquesta funcio era teoricament per quan li introduien "val" per la linia de comandes poder executar-ho i despres imprimir amb la funcio bldc_val_received
void VESC_getValues() {
	bldc_interface_get_values();
}

void bldc_val_received(mc_values *val) {
	//main_printf("Duty cycle:    %.1f %%\r\n", val->duty_now * 100.0);
	//main_printf("Fault Code:    %s\r\n", bldc_interface_fault_to_string(val->fault_code));
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

