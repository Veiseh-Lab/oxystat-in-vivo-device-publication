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

#include "max5535.h"
#include "em_usart.h"
#include "sl_spidrv_instances.h"
#include <math.h>
#include "app_log.h"
#include "em_gpio.h"
#include "channels.h"
#include "pin_config.h"

#define MAX5535_CMD_LOAD_INPUT_A    0b00010000
#define MAX5535_CMD_LOAD_INPUT_B    0b00100000
#define MAX5535_CMD_LOAD_DAC_AB     0b10000000
#define MAX5535_CMD_LOAD_INPUT_DAC_A    0b10010000
#define MAX5535_CMD_LOAD_INPUT_DAC_B    0b10100000
#define MAX5535_CMD_STANDBY         0b11000000
#define MAX5535_CMD_POWER_ON        0b11010000
#define MAX5535_CMD_POWER_DOWN      0b11100000
#define MAX5535_CMD_LOAD_ALL        0b11110000

// note vdd must be kept a minimum of 200mV higher above v_ref for proper operation
// these are the codes necessary to set the reference

const uint16_t MAX5535_REF_1V214 = 0x0000;
const uint16_t MAX5535_REF_1V940 = 0x0400;
const uint16_t MAX5535_REF_2V425 = 0x0800;
const uint16_t MAX5535_REF_3V885 = 0x0c00;

static const uint16_t ref_voltage = MAX5535_REF_1V940; // I'm just hardcoding this for now since we don't anticipate another voltage range

static void max5535_spi_transfer(DAC_t* dac, uint8_t command, uint16_t data)
{
    // enable SPI clock
    CMU_ClockEnable(cmuClock_USART0, true);

    // delay a few microseconds to let the clock stabilize
    uint32_t ticks = sl_sleeptimer_get_tick_count();
    while (sl_sleeptimer_get_tick_count() - ticks < 100);

    GPIO_PinOutClear(dac->cs_port, dac->cs_pin);

    volatile uint16_t tx_buffer =  ((uint16_t)command << 8) | data; // 4-bit command and 12-bit data

    USART_TxDouble(sl_spidrv_dac_handle->peripheral.usartPort, tx_buffer);

    // wait for transfer to complete
    while (!(sl_spidrv_dac_handle->peripheral.usartPort->STATUS & USART_STATUS_TXC));

    GPIO_PinOutSet(dac->cs_port, dac->cs_pin);

    // disable SPI clock
    CMU_ClockEnable(cmuClock_USART0, false);
}

static uint16_t max5535_millivolts_to_dac_code(uint16_t millivolts)
{
    uint16_t dac_code = 0;

    switch(ref_voltage)
    {
        case MAX5535_REF_1V214:
            dac_code = round((float)millivolts * 4096.0 / 1214.0);
            break;
        case MAX5535_REF_1V940:
            dac_code = round((float)millivolts * 4096.0 / 1940.0);
            break;
        case MAX5535_REF_2V425:
            dac_code = round((float)millivolts * 4096.0 / 2425.0);
            break;
        case MAX5535_REF_3V885:
            dac_code = round((float)millivolts * 4096.0 / 3885.0);
            break;
    }

    return dac_code;
}

void max5535_init(DAC_t* dac)
{
    // CMU_ClockEnable(cmuClock_USART0, true);

    // initalize the DAC CS pin
    GPIO_PinModeSet(dac->cs_port, dac->cs_pin, gpioModePushPull, 0);
    GPIO_PinOutSet(dac->cs_port, dac->cs_pin);

    sl_sleeptimer_delay_millisecond(5);

    max5535_spi_transfer(dac, MAX5535_CMD_LOAD_ALL, 0); // power on, DAC outputs to zero

    sl_sleeptimer_delay_millisecond(5);

    max5535_spi_transfer(dac, MAX5535_CMD_POWER_DOWN, MAX5535_REF_1V940); // shutdown, set reference

    sl_sleeptimer_delay_millisecond(5);

    dac->enabled = false;
}

void set_channel_voltage(DAC_t* dac, max5535_ch_t channel, uint16_t millivolts)
{
    uint16_t dac_code = max5535_millivolts_to_dac_code(millivolts);

    uint8_t cmd = channel == MAX5535_CH_A ? MAX5535_CMD_LOAD_INPUT_DAC_A : MAX5535_CMD_LOAD_INPUT_DAC_B;

    max5535_spi_transfer(dac, cmd, dac_code);

    dac->enabled = true;

    if (channel == MAX5535_CH_A)
    {
        dac->ch_a_mv = millivolts;
    }
    else
    {
        dac->ch_b_mv = millivolts;
    }

    app_log_debug("DAC %s Channel %i set to %u mV, dac code = %u\r\n", dac->cs_pin == CS_DAC12_PIN ? "DAC12" : "DAC34", channel, millivolts, dac_code);
}

void max5535_enable(DAC_t* dac, bool enable)
{
    if (enable)
    {
        max5535_spi_transfer(dac, MAX5535_CMD_POWER_ON, MAX5535_REF_1V940); // doesn't hurt to always set the reference
        dac->enabled = true;
    }
    else
    {
        max5535_spi_transfer(dac, MAX5535_CMD_POWER_DOWN, MAX5535_REF_1V940); // doesn't hurt to always set the reference
        dac->enabled = false;
    }
}

