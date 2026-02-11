/*
 Copyright (c) 2023 Alexander Curtiss (apcurtiss@gmail.com)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OXYSTAT_SERVICE_H_
#define OXYSTAT_SERVICE_H_

#include <stdint.h>
#include "channels.h"

void set_config_updated_callback(void (*config_updated_callback)(void));

bool buffer_potentiostat_reading(oxygenation_channels_t channel, uint16_t CHxP_mv, uint16_t CHxN_mv, int32_t i_adc, uint64_t timestamp_ms);
void flush_potentiostat_buffer();
void resend_last_data_packet();

void oxystat_set_data_notify_enabled(uint8_t enabled);
void oxystat_set_config_notify_enabled(uint8_t enabled);
void oxystat_update_config();

void notify_config_update();

void refresh_reference_time_request();

void set_connection_handle(uint8_t connection);
uint8_t get_connection_handle();

uint16_t get_potentiostat_cursor();

#endif /* OXYSTAT_SERVICE_H_ */
