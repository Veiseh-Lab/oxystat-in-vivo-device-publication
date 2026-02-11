/*
 * utilities.c
 *
 *  Created on: Jan 23, 2024
 *      Author: apcur
 */

#include "utilities.h"
#include "sl_sleeptimer.h"

uint64_t get_uptime_ms()
{
    uint64_t uptime_ticks = sl_sleeptimer_get_tick_count64();
    uint64_t uptime_ms = 0;
    sl_sleeptimer_tick64_to_ms(uptime_ticks, &uptime_ms);
    return uptime_ms;
}
