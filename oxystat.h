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

#ifndef POTENTIOSTAT_H
#define POTENTIOSTAT_H

#include <stdint.h>
#include <stdbool.h>
#include "channels.h"

#define MEASUREMENT_BUFFER_SIZE 256

void oxystat_init();
void handle_config_update();
bool process_measurement_buffer();
void oxystat_enter_low_power_mode();
bool is_measurement_in_progress();
uint16_t get_unsent_measurements_count();
void pause_measurements(bool pause);

#endif  // POTENTIOSTAT_H_
