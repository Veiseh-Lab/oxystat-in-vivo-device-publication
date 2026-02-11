#ifndef OXYCONFIG_H_
#define OXYCONFIG_H_

#include <stdint.h>
#include "channels.h"

#define CONFIG_LENGTH 21

/**
 * @brief configuration byte array for managing sensor
 * @note byte 0: state of sensor (0: disabled, 1: streaming, ...)
 * @note byte 1-2: desired measurement period (s) (ignored unless state = 1)
 * @note byte 3-4: desired measurement duration (ms) (ignored unless state = 1)
 * @note byte 5-6: desired measurement Hz (ignored unless state = 1)
 * @note byte 7-8: ch1 voltage setpoint (mV)
 * @note byte 9-10: ch2 voltage setpoint (mV)
 * @note byte 11-12: ch3 voltage setpoint (mV)
 * @note byte 13-14: ch4 voltage setpoint (mV)
 * @note byte 15: desired LED state (0: off, 1: on)
 * @note byte 16-17: desired oxygenation period (s)
 * @note byte 18: desired oxygenation duty cycle (0%-100%)
 * @note byte 19: enabled channels (bitfield: ch1=1, ch2=2, ch3=4, ch4=8)
 * @note byte 20: average samples (0: off, 1: on)
 */
#define STATE 0
#define MEASUREMENT_PERIOD_HIGH 1
#define MEASUREMENT_PERIOD_LOW 2
#define MEASUREMENT_SAMPLES_HIGH 3
#define MEASUREMENT_SAMPLES_LOW 4
#define MEASUREMENT_HZ_HIGH 5
#define MEASUREMENT_HZ_LOW 6
#define CH1_VOLTAGE_HIGH 7
#define CH1_VOLTAGE_LOW 8
#define CH2_VOLTAGE_HIGH 9
#define CH2_VOLTAGE_LOW 10
#define CH3_VOLTAGE_HIGH 11
#define CH3_VOLTAGE_LOW 12
#define CH4_VOLTAGE_HIGH 13
#define CH4_VOLTAGE_LOW 14
#define LED_STATE 15
#define OXYGENATION_PERIOD_HIGH 16
#define OXYGENATION_PERIOD_LOW 17
#define OXYGENATION_DUTY_CYCLE 18
#define ENABLED_CHANNELS 19
#define AVERAGE_SAMPLES 20

typedef struct
{
    uint8_t state;
    uint16_t measurement_period;
    uint16_t measurement_samples;
    uint16_t measurement_hz;
    uint16_t ch_mv[NUM_CHANNELS];
    uint8_t led_state;
    uint16_t oxygenation_period;
    uint8_t oxygenation_duty_cycle;

    struct enabled_channels_t
    {
        uint8_t ch1 : 1;
        uint8_t ch2 : 1;
        uint8_t ch3 : 1;
        uint8_t ch4 : 1;
    } enabled_channels;

    uint8_t average_samples;
} potentiostat_config_t;

void init_config();

uint8_t set_desired_config(uint8_t* config_bytes);

potentiostat_config_t get_desired_config();
void get_desired_config_bytes(uint8_t* config_bytes);

uint16_t get_potentiostat_measurement_period_s();
uint16_t get_potentiostat_measurement_samples();
uint16_t get_potentiostat_measurement_hz();
uint8_t get_potentiostat_state();
uint16_t get_channel_mv(oxygenation_channels_t channel);
uint8_t get_channel_state(oxygenation_channels_t channel);
uint8_t get_led_enabled();
uint32_t get_oxygenation_period_ms();
uint8_t get_oxygenation_duty_cycle();
uint32_t get_oxygenation_on_time_ms();
uint8_t get_average_samples_enabled();

void set_potentiostat_measurement_period_s(uint16_t period_s);

#endif  // OXYCONFIG_H_