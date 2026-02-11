/*
 * channels.h
 *
 *  Created on: Sep 18, 2024
 *      Author: apcur
 */

#ifndef CHANNELS_H_
#define CHANNELS_H_

#include <stdbool.h>
#include "em_gpio.h"

#define NUM_DACS 2
#define NUM_MAX5535_CHANNELS 2

// Enum for oxygenation channels
typedef enum
{
    CH_1,
    CH_2,
    CH_3,
    CH_4,
    NUM_CHANNELS
} oxygenation_channels_t;

typedef enum
{
  MAX5535_CH_A,
  MAX5535_CH_B
} max5535_ch_t;

typedef struct
{
  uint8_t cs_port;
  uint8_t cs_pin;
  bool enabled;
  bool dutycycle_off;
  uint16_t ch_a_mv;
  uint16_t ch_b_mv;
} DAC_t;

typedef struct
{
    oxygenation_channels_t ch;
    uint16_t CHxP_counts;
    uint16_t CHxN_counts;
    int32_t i_adc;
    uint32_t timestamp;
} channel_measurement_t;


#endif /* CHANNELS_H_ */
