/*
 * flexiBLE_common.h
 *
 *  Created on: Oct 10, 2022
 *      Author: apcur
 */

#ifndef FLEXIBLE_COMMON_H_
#define FLEXIBLE_COMMON_H_

#include <stdint.h>

#define MAX_NOTIFY_BUF_SIZE 240

typedef enum
{
    REFERENCE_TIME_STATUS_NORMAL = 0,
    REFERENCE_TIME_STATUS_REFRESH_REQUESTED = 1,
    REFERENCE_TIME_STATUS_REFRESHED = 2
} reference_time_status_t;

uint64_t get_flexiBLE_reference_time();
void set_flexiBLE_reference_time(uint64_t reference_time);
reference_time_status_t get_reference_time_status();
void set_reference_time_status(reference_time_status_t status);

uint8_t is_connected();
void set_connected(uint8_t connected);

#endif /* FLEXIBLE_COMMON_H_ */
