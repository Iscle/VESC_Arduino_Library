/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

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
 * packet.c
 *
 *  Created on: 21 mar 2013
 *      Author: benjamin
 */

#include "Arduino.h"
#include <string.h>
#include "packet.h"
#include "crc.h"

typedef struct {
	volatile unsigned char rx_state;
	volatile unsigned char rx_timeout;
	void(*send_func)(unsigned char *data, unsigned int len);
	void(*process_func)(unsigned char *data, unsigned int len);
	unsigned int payload_length;
	unsigned char rx_buffer[PACKET_MAX_PL_LEN];
	unsigned char tx_buffer[PACKET_MAX_PL_LEN + 6];
	unsigned int rx_data_ptr;
	unsigned char crc_low;
	unsigned char crc_high;
} PACKET_STATE_t;

static PACKET_STATE_t handler_states;

void packet_init(void (*s_func)(unsigned char *data, unsigned int len),
		void (*p_func)(unsigned char *data, unsigned int len)) {
	handler_states.send_func = s_func;
	handler_states.process_func = p_func;
}

void packet_send_packet(unsigned char *data, unsigned int len) {
	if (len > PACKET_MAX_PL_LEN) {
		return;
	}

	int b_ind = 0;

	if (len <= 256) {
		handler_states.tx_buffer[b_ind++] = 2;
		handler_states.tx_buffer[b_ind++] = len;
	} else {
		handler_states.tx_buffer[b_ind++] = 3;
		handler_states.tx_buffer[b_ind++] = len >> 8;
		handler_states.tx_buffer[b_ind++] = len & 0xFF;
	}

	memcpy(handler_states.tx_buffer + b_ind, data, len);
	b_ind += len;

	unsigned short crc = crc16(data, len);
	handler_states.tx_buffer[b_ind++] = (uint8_t)(crc >> 8);
	handler_states.tx_buffer[b_ind++] = (uint8_t)(crc & 0xFF);
	handler_states.tx_buffer[b_ind++] = 3;

	if (handler_states.send_func) {
		handler_states.send_func(handler_states.tx_buffer, b_ind);
	}
}

/**
 * Call this function every millisecond.
 */
void packet_timerfunc(void) {
	int i = 0;
	if (handler_states.rx_timeout) {
		handler_states.rx_timeout--;
	} else {
		handler_states.rx_state = 0;
	}
}

void packet_process_byte(uint8_t rx_data) {
	switch (handler_states.rx_state) {
	case 0:
		if (rx_data == 2) {
			// 1 byte PL len
			handler_states.rx_state += 2;
			handler_states.rx_timeout = PACKET_RX_TIMEOUT;
			handler_states.rx_data_ptr = 0;
			handler_states.payload_length = 0;
		} else if (rx_data == 3) {
			// 2 byte PL len
			handler_states.rx_state++;
			handler_states.rx_timeout = PACKET_RX_TIMEOUT;
			handler_states.rx_data_ptr = 0;
			handler_states.payload_length = 0;
		} else {
			handler_states.rx_state = 0;
		}
		break;

	case 1:
		handler_states.payload_length = (unsigned int)rx_data << 8;
		handler_states.rx_state++;
		handler_states.rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 2:
		handler_states.payload_length |= (unsigned int)rx_data;
		if (handler_states.payload_length > 0 &&
				handler_states.payload_length <= PACKET_MAX_PL_LEN) {
			handler_states.rx_state++;
			handler_states.rx_timeout = PACKET_RX_TIMEOUT;
		} else {
			handler_states.rx_state = 0;
		}
		break;

	case 3:
		handler_states.rx_buffer[handler_states.rx_data_ptr++] = rx_data;
		if (handler_states.rx_data_ptr == handler_states.payload_length) {
			handler_states.rx_state++;
		}
		handler_states.rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 4:
		handler_states.crc_high = rx_data;
		handler_states.rx_state++;
		handler_states.rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 5:
		handler_states.crc_low = rx_data;
		handler_states.rx_state++;
		handler_states.rx_timeout = PACKET_RX_TIMEOUT;
		break;

	case 6:
		if (rx_data == 3) {
			if (crc16(handler_states.rx_buffer, handler_states.payload_length)
					== ((unsigned short)handler_states.crc_high << 8
							| (unsigned short)handler_states.crc_low)) {
				// Packet received!
				if (handler_states.process_func) {
					handler_states.process_func(handler_states.rx_buffer,
							handler_states.payload_length);
				}
			}
		}
		handler_states.rx_state = 0;
		break;

	default:
		handler_states.rx_state = 0;
		break;
	}
}
