/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * bldc_interface.c
 *
 * Compatible Firmware Versions
 * 2.16
 *
 */

#include "Arduino.h"
#include "bldc_interface.h"
#include "buffer.h"
#include <string.h>

// Private variables
static unsigned char send_buffer[512];
static int32_t can_fwd_vesc = -1;

// Private variables for received data
static mc_values values;
static int fw_major;
static int fw_minor;
static float rotor_pos;
static float detect_cycle_int_limit;
static float detect_coupling_k;
static signed char detect_hall_table[8];
static signed char detect_hall_res;
static float dec_ppm;
static float dec_ppm_len;
static float dec_adc;
static float dec_adc_voltage;
static float dec_chuk;

// Private functions
void send_packet_no_fwd(unsigned char *data, unsigned int len);
static void fwd_can_append(uint8_t *data, int32_t *ind);

// Function pointers
static void(*send_func)(unsigned char *data, unsigned int len) = 0;
static void(*forward_func)(unsigned char *data, unsigned int len) = 0;

// Function pointers for received data
static void(*rx_value_func)(mc_values *values) = 0;
static void(*rx_printf_func)(char *str) = 0;
static void(*rx_fw_func)(int major, int minor) = 0;
static void(*rx_rotor_pos_func)(float pos) = 0;
static void(*rx_dec_ppm_func)(float val, float ms) = 0;
static void(*rx_dec_adc_func)(float val, float voltage) = 0;
static void(*rx_dec_chuk_func)(float val) = 0;

/**
 * Initialize bldc_interface.
 *
 * @param func
 * A function to be used when sending packets. Null (0) means that no packets will be sent.
 */
void bldc_interface_init(void(*func)(unsigned char *data, unsigned int len)) {
	can_fwd_vesc = -1;
	send_func = func;
}

/**
 * Enable or disable can forwarding to other VESCs.
 *
 * @param vesc_id
 * The VESC ID to forward to. Setting this to -1 disables this feature.
 */
void bldc_interface_set_forward_can(int32_t vesc_id) {
	can_fwd_vesc = vesc_id;
}

/**
 * Provide a function to forward received data to instead of processing it and calling handlers.
 * This will also prevent data from being sent.
 *
 * @param func
 * The forward function. Null (0) to disable forwarding.
 */
void bldc_interface_set_forward_func(void(*func)(unsigned char *data, unsigned int len)) {
	forward_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void bldc_interface_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void bldc_interface_process_packet(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	if (forward_func) {
		forward_func(data, len);
		return;
	}

	int32_t ind = 0;
	int i = 0;
	unsigned char id = data[0];
	data++;
	len--;

	switch (id) {
	case COMM_FW_VERSION:
		if (len == 2) {
			ind = 0;
			fw_major = data[ind++];
			fw_minor = data[ind++];
		} else {
			fw_major = -1;
			fw_minor = -1;
		}
		break;

	case COMM_GET_VALUES:
		ind = 0;
		values.temp_mos = buffer_get_float16(data, 1e1, &ind);
		values.temp_pcb = buffer_get_float16(data, 1e1, &ind);
		values.current_motor = buffer_get_float32(data, 1e2, &ind);
		values.current_in = buffer_get_float32(data, 1e2, &ind);
		values.id = buffer_get_float32(data, 1e2, &ind);
		values.iq = buffer_get_float32(data, 1e2, &ind);
		values.duty_now = buffer_get_float16(data, 1e3, &ind);
		values.rpm = buffer_get_float32(data, 1e0, &ind);
		values.v_in = buffer_get_float16(data, 1e1, &ind);
		values.amp_hours = buffer_get_float32(data, 1e4, &ind);
		values.amp_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values.watt_hours = buffer_get_float32(data, 1e4, &ind);
		values.watt_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values.tachometer = buffer_get_int32(data, &ind);
		values.tachometer_abs = buffer_get_int32(data, &ind);
		values.fault_code = (mc_fault_code)data[ind++];
		values.pid_pos_now = buffer_get_float32(data, 1e6, &ind);

		if (rx_value_func) {
			rx_value_func(&values);
		}
		break;

	case COMM_GET_DECODED_PPM:
		ind = 0;
		dec_ppm = buffer_get_float32(data, 1000000.0, &ind);
		dec_ppm_len = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_ppm_func) {
			rx_dec_ppm_func(dec_ppm, dec_ppm_len);
		}
		break;

	case COMM_GET_DECODED_ADC:
		ind = 0;
		dec_adc = buffer_get_float32(data, 1000000.0, &ind);
		dec_adc_voltage = buffer_get_float32(data, 1000000.0, &ind);
		// TODO for adc2

		if (rx_dec_adc_func) {
			rx_dec_adc_func(dec_adc, dec_adc_voltage);
		}
		break;

	case COMM_GET_DECODED_CHUK:
		ind = 0;
		dec_chuk = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_chuk_func) {
			rx_dec_chuk_func(dec_chuk);
		}
		break;

	default:
		break;
	}
}

/**
 * Function pointer setters. When data that is requested with the get functions
 * is received, the corresponding function pointer will be called with the
 * received data.
 *
 * @param func
 * A function to be called when the corresponding data is received.
 */

void bldc_interface_set_rx_value_func(void(*func)(mc_values *values)) {
	rx_value_func = func;
}

void bldc_interface_set_rx_printf_func(void(*func)(char *str)) {
	rx_printf_func = func;
}

void bldc_interface_set_rx_fw_func(void(*func)(int major, int minor)) {
	rx_fw_func = func;
}

void bldc_interface_set_rx_rotor_pos_func(void(*func)(float pos)) {
	rx_rotor_pos_func = func;
}

void bldc_interface_set_rx_dec_ppm_func(void(*func)(float val, float ms)) {
	rx_dec_ppm_func = func;
}

void bldc_interface_set_rx_dec_adc_func(void(*func)(float val, float voltage)) {
	rx_dec_adc_func = func;
}

void bldc_interface_set_rx_dec_chuk_func(void(*func)(float val)) {
	rx_dec_chuk_func = func;
}

// Setters
void bldc_interface_terminal_cmd(char* cmd) {
	int32_t send_index = 0;
	int len = strlen(cmd);
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_TERMINAL_CMD;
	memcpy(send_buffer + send_index, cmd, len);
	send_index += len;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_duty_cycle(float dutyCycle) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_DUTY;
	buffer_append_float32(send_buffer, dutyCycle, 100000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_current(float current) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_CURRENT;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_current_brake(float current) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_rpm(int rpm) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_RPM;
	buffer_append_int32(send_buffer, rpm, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_pos(float pos) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_POS;
	buffer_append_float32(send_buffer, pos, 1000000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_handbrake(float current) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_HANDBRAKE;
	buffer_append_float32(send_buffer, current, 1e3, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_servo_pos(float pos) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_SET_SERVO_POS;
	buffer_append_float16(send_buffer, pos, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_chuck_data(const chuck_data *chuckdata) {
  int32_t send_index = 0;
  fwd_can_append(send_buffer, &send_index);
  send_buffer[send_index++] = COMM_SET_CHUCK_DATA;
  send_buffer[send_index++] = chuckdata -> js_x;
  send_buffer[send_index++] = chuckdata -> js_y;
  buffer_append_bool(send_buffer, chuckdata -> bt_c, &send_index);
  buffer_append_bool(send_buffer, chuckdata -> bt_z, &send_index);
  buffer_append_int32(send_buffer, chuckdata -> acc_x, &send_index);
  buffer_append_int32(send_buffer, chuckdata -> acc_y, &send_index);
  buffer_append_int32(send_buffer, chuckdata -> acc_z, &send_index);
  send_packet_no_fwd(send_buffer, send_index);
}

// Getters
void bldc_interface_get_fw_version(void) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_FW_VERSION;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_values(void) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_GET_VALUES;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_decoded_ppm(void) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_GET_DECODED_PPM;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_decoded_adc(void) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_GET_DECODED_ADC;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_decoded_chuk(void) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_GET_DECODED_CHUK;
	send_packet_no_fwd(send_buffer, send_index);
}

// Other functions

void bldc_interface_reboot(void) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_REBOOT;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_send_alive(void) {
	int32_t send_index = 0;
	fwd_can_append(send_buffer, &send_index);
	send_buffer[send_index++] = COMM_ALIVE;
	send_packet_no_fwd(send_buffer, send_index);
}

// Helpers
const char* bldc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE";
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE";
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE";
	case FAULT_CODE_DRV: return "FAULT_CODE_DRV";
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT";
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET";
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR";
	default: return "Unknown fault";
	}
}

// Private functions
void send_packet_no_fwd(unsigned char *data, unsigned int len) {
	if (!forward_func) {
		bldc_interface_send_packet(data, len);
	}
}

static void fwd_can_append(uint8_t *data, int32_t *ind) {
	if (can_fwd_vesc >= 0) {
		data[(*ind)++] = COMM_FORWARD_CAN;
		data[(*ind)++] = can_fwd_vesc;
	}
}
