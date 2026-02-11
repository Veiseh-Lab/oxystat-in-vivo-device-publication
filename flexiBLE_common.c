/*
 * flexiBLE_common.c
 *
 *  Created on: Oct 10, 2022
 *      Author: apcur
 */

#include "flexiBLE_common.h"

static uint64_t _flexiBLE_reference_time = 0;
static reference_time_status_t _reference_time_status = REFERENCE_TIME_STATUS_NORMAL;

static uint8_t _connected = 0;

uint64_t get_flexiBLE_reference_time()
{
    return _flexiBLE_reference_time;
}

void set_flexiBLE_reference_time(uint64_t reference_time)
{
    _flexiBLE_reference_time = reference_time;
    set_reference_time_status(REFERENCE_TIME_STATUS_REFRESHED);
}

reference_time_status_t get_reference_time_status()
{
    reference_time_status_t status = _reference_time_status;
    
    if (status == REFERENCE_TIME_STATUS_REFRESHED)
    {
        set_reference_time_status(REFERENCE_TIME_STATUS_NORMAL);
    }
    
    return status;
}

void set_reference_time_status(reference_time_status_t status)
{
    _reference_time_status = status;
}

uint8_t is_connected()
{
    return _connected;
}

void set_connected(uint8_t connected)
{
    _connected = connected;
}
