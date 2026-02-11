#include "oxyconfig.h"
#include <stdint.h>
#include "app_log.h"

static potentiostat_config_t desired_config;

void init_config()
{
    // initialize with default settings
    desired_config.state = 0;
    desired_config.measurement_period = 600; // seconds
    desired_config.measurement_samples = 10; // milliseconds
    desired_config.measurement_hz = 300;
    desired_config.ch_mv[CH_1] = 0;
    desired_config.ch_mv[CH_2] = 0;
    desired_config.ch_mv[CH_3] = 0;
    desired_config.ch_mv[CH_4] = 0;
    desired_config.led_state = 0;
    desired_config.oxygenation_period = 0;
    desired_config.oxygenation_duty_cycle = 0;
    desired_config.enabled_channels.ch1 = 1;
    desired_config.enabled_channels.ch2 = 1;
    desired_config.enabled_channels.ch3 = 1;
    desired_config.enabled_channels.ch4 = 1;
    desired_config.average_samples = 1;
}

uint8_t set_desired_config(uint8_t* config_bytes)
{
    uint8_t updated = 0;

    if (config_bytes[STATE] != desired_config.state)
    {
        desired_config.state = config_bytes[STATE];
        updated = 1;
        app_log_info("update state: %d\r\n", desired_config.state);
    }

    if ((config_bytes[MEASUREMENT_PERIOD_HIGH] << 8 | config_bytes[MEASUREMENT_PERIOD_LOW]) != desired_config.measurement_period)
    {
        desired_config.measurement_period = (config_bytes[MEASUREMENT_PERIOD_HIGH] << 8 | config_bytes[MEASUREMENT_PERIOD_LOW]);
        updated = 1;
        app_log_info("update measurement period: %d\r\n", desired_config.measurement_period);
    }

    if ((config_bytes[MEASUREMENT_SAMPLES_HIGH] << 8 | config_bytes[MEASUREMENT_SAMPLES_LOW]) != desired_config.measurement_samples)
    {
        desired_config.measurement_samples = (config_bytes[MEASUREMENT_SAMPLES_HIGH] << 8 | config_bytes[MEASUREMENT_SAMPLES_LOW]);
        updated = 1;
        app_log_info("update measurement samples: %d\r\n", desired_config.measurement_samples);
    }

    if ((config_bytes[MEASUREMENT_HZ_HIGH] << 8 | config_bytes[MEASUREMENT_HZ_LOW]) != desired_config.measurement_hz)
    {
        desired_config.measurement_hz = (config_bytes[MEASUREMENT_HZ_HIGH] << 8 | config_bytes[MEASUREMENT_HZ_LOW]);
        updated = 1;
        app_log_info("update measurement hz: %d\r\n", desired_config.measurement_hz);
    }

    if ((config_bytes[CH1_VOLTAGE_HIGH] << 8 | config_bytes[CH1_VOLTAGE_LOW]) != desired_config.ch_mv[CH_1])
    {
        desired_config.ch_mv[CH_1] = (config_bytes[CH1_VOLTAGE_HIGH] << 8 | config_bytes[CH1_VOLTAGE_LOW]);
        updated = 1;
        app_log_info("update ch1 mv: %d\r\n", desired_config.ch_mv[CH_1]);
    }

    if ((config_bytes[CH2_VOLTAGE_HIGH] << 8 | config_bytes[CH2_VOLTAGE_LOW]) != desired_config.ch_mv[CH_2])
    {
        desired_config.ch_mv[CH_2] = (config_bytes[CH2_VOLTAGE_HIGH] << 8 | config_bytes[CH2_VOLTAGE_LOW]);
        updated = 1;
        app_log_info("update ch2 mv: %d\r\n", desired_config.ch_mv[CH_2]);
    }

    if ((config_bytes[CH3_VOLTAGE_HIGH] << 8 | config_bytes[CH3_VOLTAGE_LOW]) != desired_config.ch_mv[CH_3])
    {
        desired_config.ch_mv[CH_3] = (config_bytes[CH3_VOLTAGE_HIGH] << 8 | config_bytes[CH3_VOLTAGE_LOW]);
        updated = 1;
        app_log_info("update ch3 mv: %d\r\n", desired_config.ch_mv[CH_3]);
    }

    if ((config_bytes[CH4_VOLTAGE_HIGH] << 8 | config_bytes[CH4_VOLTAGE_LOW]) != desired_config.ch_mv[CH_4])
    {
        desired_config.ch_mv[CH_4] = (config_bytes[CH4_VOLTAGE_HIGH] << 8 | config_bytes[CH4_VOLTAGE_LOW]);
        updated = 1;
        app_log_info("update ch4 mv: %d\r\n", desired_config.ch_mv[CH_4]);
    }

    if (config_bytes[LED_STATE] != desired_config.led_state)
    {
        desired_config.led_state = config_bytes[LED_STATE];
        updated = 1;
        app_log_info("update led state: %d\r\n", desired_config.led_state);
    }

    if ((config_bytes[OXYGENATION_PERIOD_HIGH] << 8 | config_bytes[OXYGENATION_PERIOD_LOW]) != desired_config.oxygenation_period)
    {
        desired_config.oxygenation_period = (config_bytes[OXYGENATION_PERIOD_HIGH] << 8 | config_bytes[OXYGENATION_PERIOD_LOW]);
        updated = 1;
        app_log_info("update oxygenation period: %d\r\n", desired_config.oxygenation_period);
    }

    if (config_bytes[OXYGENATION_DUTY_CYCLE] != desired_config.oxygenation_duty_cycle)
    {
        desired_config.oxygenation_duty_cycle = config_bytes[OXYGENATION_DUTY_CYCLE];
        updated = 1;
        app_log_info("update oxygenation duty cycle: %d\r\n", desired_config.oxygenation_duty_cycle);
    }

    if ((config_bytes[ENABLED_CHANNELS] & 0x01) != desired_config.enabled_channels.ch1)
    {
        desired_config.enabled_channels.ch1 = config_bytes[ENABLED_CHANNELS] & 0x01;
        updated = 1;
        app_log_info("update ch1 enabled: %d\r\n", desired_config.enabled_channels.ch1);
    }

    if ((config_bytes[ENABLED_CHANNELS] >> 1 & 0x01) != desired_config.enabled_channels.ch2)
    {
        desired_config.enabled_channels.ch2 = config_bytes[ENABLED_CHANNELS] >> 1 & 0x01;
        updated = 1;
        app_log_info("update ch2 enabled: %d\r\n", desired_config.enabled_channels.ch2);
    }

    if ((config_bytes[ENABLED_CHANNELS] >> 2 & 0x01) != desired_config.enabled_channels.ch3)
    {
        desired_config.enabled_channels.ch3 = config_bytes[ENABLED_CHANNELS] >> 2 & 0x01;
        updated = 1;
        app_log_info("update ch3 enabled: %d\r\n", desired_config.enabled_channels.ch3);
    }

    if ((config_bytes[ENABLED_CHANNELS] >> 3 & 0x01) != desired_config.enabled_channels.ch4)
    {
        desired_config.enabled_channels.ch4 = config_bytes[ENABLED_CHANNELS] >> 3 & 0x01;
        updated = 1;
        app_log_info("update ch4 enabled: %d\r\n", desired_config.enabled_channels.ch4);
    }

    if (config_bytes[AVERAGE_SAMPLES] != desired_config.average_samples)
    {
        desired_config.average_samples = config_bytes[AVERAGE_SAMPLES];
        updated = 1;
        app_log_info("update average samples: %d\r\n", desired_config.average_samples);
    }

    return updated;
}

void get_desired_config_bytes(uint8_t* config_bytes)
{
    config_bytes[STATE] = desired_config.state;
    config_bytes[MEASUREMENT_PERIOD_HIGH] = (desired_config.measurement_period & 0xFF00) >> 8;
    config_bytes[MEASUREMENT_PERIOD_LOW] = desired_config.measurement_period & 0xFF;
    config_bytes[MEASUREMENT_SAMPLES_HIGH] = (desired_config.measurement_samples & 0xFF00) >> 8;
    config_bytes[MEASUREMENT_SAMPLES_LOW] = desired_config.measurement_samples & 0xFF;
    config_bytes[MEASUREMENT_HZ_HIGH] = (desired_config.measurement_hz & 0xFF00) >> 8;
    config_bytes[MEASUREMENT_HZ_LOW] = desired_config.measurement_hz & 0xFF;
    config_bytes[CH1_VOLTAGE_HIGH] = (desired_config.ch_mv[CH_1] & 0xFF00) >> 8;
    config_bytes[CH1_VOLTAGE_LOW] = desired_config.ch_mv[CH_1] & 0xFF;
    config_bytes[CH2_VOLTAGE_HIGH] = (desired_config.ch_mv[CH_2] & 0xFF00) >> 8;
    config_bytes[CH2_VOLTAGE_LOW] = desired_config.ch_mv[CH_2] & 0xFF;
    config_bytes[CH3_VOLTAGE_HIGH] = (desired_config.ch_mv[CH_3] & 0xFF00) >> 8;
    config_bytes[CH3_VOLTAGE_LOW] = desired_config.ch_mv[CH_3] & 0xFF;
    config_bytes[CH4_VOLTAGE_HIGH] = (desired_config.ch_mv[CH_4] & 0xFF00) >> 8;
    config_bytes[CH4_VOLTAGE_LOW] = desired_config.ch_mv[CH_4] & 0xFF;
    config_bytes[LED_STATE] = desired_config.led_state;
    config_bytes[OXYGENATION_PERIOD_HIGH] = (desired_config.oxygenation_period & 0xFF00) >> 8;
    config_bytes[OXYGENATION_PERIOD_LOW] = desired_config.oxygenation_period & 0xFF;
    config_bytes[OXYGENATION_DUTY_CYCLE] = desired_config.oxygenation_duty_cycle;
    config_bytes[ENABLED_CHANNELS] = desired_config.enabled_channels.ch1 | (desired_config.enabled_channels.ch2 << 1) | (desired_config.enabled_channels.ch3 << 2) | (desired_config.enabled_channels.ch4 << 3);
    config_bytes[AVERAGE_SAMPLES] = desired_config.average_samples;
}

potentiostat_config_t get_desired_config()
{
    return desired_config;
}

uint8_t get_potentiostat_state()
{
    return desired_config.state;
}

uint16_t get_potentiostat_measurement_period_s()
{
    return desired_config.measurement_period;
}

uint16_t get_potentiostat_measurement_samples()
{
    return desired_config.measurement_samples;
}

uint16_t get_potentiostat_measurement_hz()
{
    return desired_config.measurement_hz;
}

uint8_t get_channel_state(oxygenation_channels_t channel)
{
    switch (channel)
    {
    case CH_1:
        return desired_config.enabled_channels.ch1;
    case CH_2:
        return desired_config.enabled_channels.ch2;
    case CH_3:
        return desired_config.enabled_channels.ch3;
    case CH_4:
        return desired_config.enabled_channels.ch4;
    default:
        return 0;
    }
}

uint16_t get_channel_mv(oxygenation_channels_t channel)
{
    return desired_config.ch_mv[channel];
}

uint32_t get_oxygenation_period_ms()
{
    return desired_config.oxygenation_period * 1000;
}

uint8_t get_oxygenation_duty_cycle()
{
    return desired_config.oxygenation_duty_cycle;
}

uint32_t get_oxygenation_on_time_ms()
{
    return (uint16_t)((float)get_oxygenation_period_ms() * (float)get_oxygenation_duty_cycle() / 100.0f);
}

uint8_t get_led_enabled()
{
    return desired_config.led_state;
}

uint8_t get_average_samples_enabled()
{
    return desired_config.average_samples;
}

void set_potentiostat_measurement_period_s(uint16_t period)
{
    desired_config.measurement_period = period;
}