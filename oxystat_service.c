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

#include "oxystat_service.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "flexiBLE_common.h"
#include "app_log.h"
#include "utilities.h"
#include "channels.h"
#include "oxyconfig.h"
#include "blink.h"

static uint8_t data_notify_enabled = 0;
static uint8_t config_notify_enabled = 0;

static uint16_t potentiostat_data_cursor = 0;
static uint16_t last_packet_data_cursor = 0;
static uint8_t potentiostat_data[MAX_NOTIFY_BUF_SIZE] = { 0 };

static uint64_t last_record_ms = 0;

static uint8_t _connection = 0;

static void (*config_updated_callback_)();

void set_config_updated_callback(void (*config_updated_callback)(void))
{
    config_updated_callback_ = config_updated_callback;
}

void oxystat_set_data_notify_enabled(uint8_t enabled)
{
    switch (enabled)
    {
        case 0x00:
            app_log_info("data notify disabled\r\n");
            data_notify_enabled = 0;
            break;
        case 0x01:
            app_log_info("data notify enabled\r\n");
            data_notify_enabled = 1;
            break;
        case 0x02:
            app_log_info("data indication enabled\r\n");
            data_notify_enabled = 1;
            break;
        case 0x03:
            app_log_info("data indication and notification enabled\r\n");
            data_notify_enabled = 1;
            break;
        default:
            app_log_warning("invalid data notify value: %u\r\n", enabled);
            data_notify_enabled = 0;
            return;
    }

    app_log_info("set data notify: 0x%0x\r\n", enabled);
}

void oxystat_set_config_notify_enabled(uint8_t enabled)
{
  config_notify_enabled = enabled;
  app_log_info("set config notify: %s\r\n", enabled == 1 ? "enabled" : "disabled");
}

void oxystat_update_config()
{
    app_log_info("writing potentiostat config\r\n");

    uint8_t config_bytes[CONFIG_LENGTH] = {0};

    size_t value_len = 0;
    sl_bt_gatt_server_read_attribute_value(gattdb_oxystat_config, 0, CONFIG_LENGTH, &value_len, config_bytes);
    if (value_len > CONFIG_LENGTH)
    {
        app_log_warning("fw config not large enough: %u vs %d\r\n", value_len, CONFIG_LENGTH);
    }

    app_log_debug("config_bytes: \r\n");
    for (uint8_t i = 0; i < value_len; i++)
    {
        app_log_debug("[%u] = %u\r\n", i, config_bytes[i]);
    }

    // update the desired config
    uint8_t updated = set_desired_config(config_bytes);

    if (updated)
    {
        // now notify oxystat of the change using the callback
        config_updated_callback_();
    }
}

void refresh_reference_time_request()
{
    app_log_debug("sending reference time refresh request\r\n");
    uint8_t reference_time_update = 1;
    sl_status_t status = sl_bt_gatt_server_notify_all(gattdb_refresh_reference_time, 1, &reference_time_update);
    if (status != SL_STATUS_OK)
    {
        app_log_warning("Reference time refresh notification error: 0x%lx\r\n", status);
    }
    else
    {
        set_reference_time_status(REFERENCE_TIME_STATUS_REFRESH_REQUESTED);
    }
}

void notify_config_update()
{
    // get the current config from memory
    uint8_t config_bytes[CONFIG_LENGTH] = {0};
    get_desired_config_bytes(config_bytes);

    // commit config to GATT
    sl_status_t status = sl_bt_gatt_server_write_attribute_value(gattdb_oxystat_config, 0, CONFIG_LENGTH, config_bytes);
    if (status != SL_STATUS_OK)
    {
        app_log_warning("GATT config write error: 0x%lx\r\n", status);
    }
    else
    {
        app_log_info("GATT config updated\r\n");
    }

    //log the config
    for (uint8_t i = 0; i < CONFIG_LENGTH; i++)
    {
        app_log_debug("config[%u] = %u\r\n", i, config_bytes[i]);
    }

    if (config_notify_enabled)
        {
        sl_bt_gatt_server_notify_all(gattdb_oxystat_config, CONFIG_LENGTH, config_bytes);
        app_log_info("Config notification sent\r\n");
        }
    else
        {
        app_log_error("config notify not enabled!\r\n");
        }
}

/**
 * @brief buffer a potentiostat reading for notification
 * @returns true if the buffer was sent to the central
 */
bool buffer_potentiostat_reading(oxygenation_channels_t channel, uint16_t CHxP_mv, uint16_t CHxN_mv, int32_t i_adc, uint64_t timestamp_ms)
{
    bool sent = false;

    if (get_potentiostat_state() == 0)
    {
        return false;
    }

    if (potentiostat_data_cursor + 17 >= MAX_NOTIFY_BUF_SIZE)  // before we add a new record, check that we won't exceed the maximum size
    {
        if (data_notify_enabled)
        {
            last_packet_data_cursor = potentiostat_data_cursor;

            sl_status_t status = sl_bt_gatt_server_send_indication(get_connection_handle(), gattdb_oxystat_data, potentiostat_data_cursor, potentiostat_data);
            // sl_status_t status = sl_bt_gatt_server_notify_all(gattdb_oxystat_data, potentiostat_data_cursor, potentiostat_data); // if we do, kick off a BLE offload
            if (status != SL_STATUS_OK)
            {
                app_log_warning("Potentiostat data indication error: 0x%lx\r\n", status);
            }
            else
            {
                app_log_info("Potentiostat data indicated connection\r\n");
                sent = true;
            }
        }
        else
        {
            app_log_error("trying to send, but data notify not enabled!\r\n");
        }

        potentiostat_data_cursor = 0;
    }

    int64_t record_offset = 0;

    if (potentiostat_data_cursor == 0) // initialize with anchor_ms
    {
        // there is a potential underflow condition if timestamp_ms is less than the reference time.
        // this should be handled by the BLE connection check not running while a measurement is in progress
        // but we'll check here and set to zero to cover edge cases

        uint32_t anchor_ms;

        if (timestamp_ms < get_flexiBLE_reference_time())
        {
            app_log_warning("timestamp: %lu < reference: %lu--setting anchor to zero\r\n", (uint32_t)timestamp_ms, (uint32_t)get_flexiBLE_reference_time());
            anchor_ms = 0;
        }
        else
        {
            anchor_ms = timestamp_ms - get_flexiBLE_reference_time();
        }
        potentiostat_data[potentiostat_data_cursor++] = anchor_ms & 0xFF;
        potentiostat_data[potentiostat_data_cursor++] = (anchor_ms >> 8) & 0xFF;
        potentiostat_data[potentiostat_data_cursor++] = (anchor_ms >> 16) & 0xFF;
        potentiostat_data[potentiostat_data_cursor++] = (anchor_ms >> 24) & 0xFF;

        record_offset = 0;

        app_log_info("New potentiostat data record initialized, anchor_ms=%lu\r\n", anchor_ms);
    }
    else
    {
        record_offset = timestamp_ms - last_record_ms;
        if (record_offset < 0)
        {
            app_log_warning("Negative record offset: %ld\r\n", (int32_t)record_offset);
            record_offset = 0;
        }
        else if (record_offset > 600000)
        {
            app_log_warning("LARGE RECORD OFFSET: %ld\r\n", (int32_t)record_offset);
        }
    }

    potentiostat_data[potentiostat_data_cursor++] = (record_offset >> 24) & 0xFF;
    potentiostat_data[potentiostat_data_cursor++] = (record_offset >> 16) & 0xFF;
    potentiostat_data[potentiostat_data_cursor++] = (record_offset >> 8) & 0xFF;
    potentiostat_data[potentiostat_data_cursor++] = (record_offset) & 0xFF;

    last_record_ms = timestamp_ms;

    potentiostat_data[potentiostat_data_cursor++] = (uint8_t)channel;

    potentiostat_data[potentiostat_data_cursor++] = (CHxP_mv >> 8) & 0xFF;
    potentiostat_data[potentiostat_data_cursor++] = (CHxP_mv) & 0xFF;

    potentiostat_data[potentiostat_data_cursor++] = (CHxN_mv >> 8) & 0xFF;
    potentiostat_data[potentiostat_data_cursor++] = (CHxN_mv) & 0xFF;

    potentiostat_data[potentiostat_data_cursor++] = (i_adc >> 24) & 0xFF;
    potentiostat_data[potentiostat_data_cursor++] = (i_adc >> 16) & 0xFF;
    potentiostat_data[potentiostat_data_cursor++] = (i_adc >> 8) & 0xFF;
    potentiostat_data[potentiostat_data_cursor++] = (i_adc) & 0xFF;

    app_log_debug("Potentiostat measurement recorded, potentiostat_data_cursor=%u, record_offset=%ld\r\n", potentiostat_data_cursor, (int32_t)record_offset);

    return sent;
}

void flush_potentiostat_buffer()
{
    if (potentiostat_data_cursor > 0)
    {
        if (data_notify_enabled)
        {
            last_packet_data_cursor = potentiostat_data_cursor;

            sl_bt_gatt_server_send_indication(get_connection_handle(), gattdb_oxystat_data, potentiostat_data_cursor, potentiostat_data);

            // sl_bt_gatt_server_notify_all(gattdb_oxystat_data, potentiostat_data_cursor, potentiostat_data);
            app_log_info("Potentiostat data indicated connection\r\n");
        }

        potentiostat_data_cursor = 0;
    }
}

void resend_last_data_packet()
{
    if (last_packet_data_cursor > 0)
    {
        
        sl_bt_gatt_server_send_indication(get_connection_handle(), gattdb_oxystat_data, last_packet_data_cursor, potentiostat_data);

        // sl_bt_gatt_server_notify_all(gattdb_oxystat_data, last_packet_data_cursor, potentiostat_data);
        app_log_info("Potentiostat data resent to connection\r\n");
    }
}

void set_connection_handle(uint8_t connection)
{
    app_log_info("Connection handle set: %u\r\n", connection);
    _connection = connection;
}

uint8_t get_connection_handle()
{
    return _connection;
}

uint16_t get_potentiostat_cursor()
{
    return potentiostat_data_cursor;
}