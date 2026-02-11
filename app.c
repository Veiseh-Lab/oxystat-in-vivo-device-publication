/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include "em_common.h"
#include "em_system.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "app_log.h"
#include "sl_power_manager_debug.h"
#include "em_wdog.h"
#include "em_cmu.h"
#include <stdio.h>
#include "utilities.h"
#include "sl_sleeptimer.h"
#include "pin_config.h"

#include "oxystat.h"
#include "oxystat_service.h"
#include "oxyconfig.h"
#include "version.h"
#include "blink.h"
#include "flexiBLE_common.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// The connection parameters with the central
static uint16_t connection_interval = 0;
static uint16_t connection_latency = 0;
static uint16_t connection_timeout = 0;

// sleeptimer handle for checking connection parameters
static sl_sleeptimer_timer_handle_t ble_parameter_check_timer;
static uint8_t close_connection_flag = 0;

// app state for processing measurements
static app_state_t app_state = APP_STATE_IDLE;
static uint8_t last_measurement_buffer_size = 0;
static bool data_indication_received = false;

// Desired BLE connection check period
#define BLE_CHECK_MS 5000
static sl_sleeptimer_timer_handle_t ble_connection_timer;
static uint8_t refresh_reference_time_flag = 0;
static uint8_t num_refresh_attempts = 0;

// function prototypes
void ble_parameter_check_cb(sl_sleeptimer_timer_handle_t *handle, void *data);
void ble_connection_cb(sl_sleeptimer_timer_handle_t *handle, void *data);
void wdog_init();

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  app_log_info("OXYSTAT INITIALIZING...\r\n");
  app_log_info("VERSION %u.%u\r\n", VERSION_MAJOR, VERSION_MINOR);

  // flash LED to indicate startup
  flash(3, 10, 100);

  blink_init();
  init_config();
  set_config_updated_callback(handle_config_update);
  oxystat_init();

  // initialize magnetic switch
  GPIO_PinModeSet(MAG_SW_PORT, MAG_SW_PIN, gpioModeInput, 1); // input with glitch filter

  wdog_init();

  app_log_info("OXYSTAT INITIALIZED SUCCESSFULLY\r\n");
  app_log_info("PROGRAM STARTING\r\n");
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
  
  if (get_unsent_measurements_count() - last_measurement_buffer_size != 0)
  {
    app_log_info("UNSENT MEASUREMENTS: %u\r\n", get_unsent_measurements_count());
    last_measurement_buffer_size = get_unsent_measurements_count();
  }

  if (is_connected())
  {
    // periodic data upload
    app_state_t next_app_state = app_state;
    switch (app_state)
    {
      case APP_STATE_IDLE:
        if (get_unsent_measurements_count() > MEASUREMENT_BUFFER_SIZE - get_potentiostat_measurement_samples() && !is_measurement_in_progress())
        {
          next_app_state = APP_STATE_BUFFER_FULL;
        }
        break;
      case APP_STATE_BUFFER_FULL:
        pause_measurements(true); // this gets unpaused when we receive a new reference time. if we don't, we'll disconnect after a few attempts.
        next_app_state = APP_STATE_SEND_PACKET;
        break;
      case APP_STATE_SEND_PACKET:
        process_measurement_buffer();
        data_indication_received = false;
        next_app_state = APP_STATE_AWAIT_ACK;
        break;
      case APP_STATE_AWAIT_ACK:
        if (data_indication_received)
        {
          if (get_unsent_measurements_count() == 0)
          {
            next_app_state = APP_STATE_REFRESH_REF_TIME;
          }
          else
          {
            next_app_state = APP_STATE_SEND_PACKET;
          }
        }
        break;
      case APP_STATE_REFRESH_REF_TIME:
        refresh_reference_time_request();
        sl_sleeptimer_start_timer_ms(&ble_connection_timer, BLE_CHECK_MS, ble_connection_cb, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
        next_app_state = APP_STATE_IDLE;
        break;
      default:
        next_app_state = APP_STATE_IDLE;
        break;
    }

    if (app_state != next_app_state)
    {
      app_log_info("APP STATE CHANGED FROM %d TO %d\r\n", app_state, next_app_state);
      app_state = next_app_state;
    }

  }

  if (close_connection_flag)
  {
    close_connection_flag = 0;
    sl_bt_connection_close(get_connection_handle());
  }

  if (refresh_reference_time_flag)
  {
    refresh_reference_time_flag = 0;
    refresh_reference_time_request();
  }

  // check magnet switch, if low then put device into low power mode
  if (GPIO_PinInGet(MAG_SW_PORT, MAG_SW_PIN) == false)
  {
    oxystat_enter_low_power_mode();
  }

  WDOGn_Feed(DEFAULT_WDOG);  
}

void ble_parameter_check_cb(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

  app_log_info("CHECKING CONNECTION PARAMETERS\r\n");
  // check to see if connection parameters have updated with a config write, set slower parameters if not
  if (connection_interval == 12)
  {
    app_log_warning("CONNECTION PARAMETER UPDATE FAILED. DISCONNECTING.\r\n");
    close_connection_flag = 1; // handle this in the main loop
    return;
  }

  app_log_info("connection parameters updated: interval %d, latency %d, timeout %d\r\n", connection_interval, connection_latency, connection_timeout);

}

void ble_connection_cb(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

  app_log_info("BLE CONNECTION CHECK TIMER FIRED\r\n");
  reference_time_status_t status = get_reference_time_status();
  switch(status)
  {
    case REFERENCE_TIME_STATUS_NORMAL: // we don't have to do anything; let the timer expire
    case REFERENCE_TIME_STATUS_REFRESHED: // we got a response, so we're good
      num_refresh_attempts = 0;
      break;
    case REFERENCE_TIME_STATUS_REFRESH_REQUESTED: // we didn't get a response, so we'll try again
      app_log_warning("REFERENCE TIME NOT REFRESHED -- SENDING ANOTHER REQUEST\r\n");
      refresh_reference_time_flag = 1;
      sl_sleeptimer_start_timer_ms(&ble_connection_timer, BLE_CHECK_MS, ble_connection_cb, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
      num_refresh_attempts++;
      break;
  }

  if (num_refresh_attempts > 4)
  {
    app_log_warning("REFERENCE TIME NOT REFRESHED AFTER 4 ATTEMPTS -- DISCONNECTING\r\n");
    close_connection_flag = 1;
  }
}


/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];
  char device_name[14];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      // create unique device name from system unique id
      uint64_t unique_id = SYSTEM_GetUnique();
      uint8_t uid[2] = { (unique_id & 0xFF), (unique_id >> 8) & 0xFF };
      snprintf(device_name, 14, "uOS-%x%x", uid[0], uid[1]);
      app_log_info("device name: %s\r\n", device_name);
      sc = sl_bt_gatt_server_write_attribute_value(gattdb_device_name, 0, 15, (unsigned char*)device_name);

      app_assert_status(sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        2056, // min. adv. interval (milliseconds * 1.6)
        2056, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);

      // Reduce Tx Power to 0 dBm
      int16_t pwr_min = 0, pwr_max = 0;
      sl_bt_system_set_tx_power(0, 0, &pwr_min, &pwr_max);

      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);

      app_log_info("BEGAN BLE ADVERTISING\r\n");

      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log_info("CONNECTION OPENED\r\n");

      // Set default MTU to 247 bytes.
      uint16_t max_mtu = 0;
      sc = sl_bt_gatt_server_set_max_mtu(247, &max_mtu);
      app_assert_status(sc);
      
      sl_bt_connection_set_parameters(evt->data.evt_connection_opened.connection, 12, 12, 0, 1080, 0, 0xffff);

      // start timer to check that these connection parameters successfully update after config write
      sl_sleeptimer_start_timer_ms(&ble_parameter_check_timer, 5000, ble_parameter_check_cb, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);

      set_connection_handle(evt->data.evt_connection_opened.connection);

      set_connected(1);

      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:

      set_connected(0);
      set_connection_handle(0);

      // cancel any active timers
      sl_sleeptimer_stop_timer(&ble_parameter_check_timer);
      sl_sleeptimer_stop_timer(&ble_connection_timer);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    case sl_bt_evt_connection_parameters_id :
    {
     connection_interval = evt->data.evt_connection_parameters.interval;
     connection_latency = evt->data.evt_connection_parameters.latency;
     connection_timeout = evt->data.evt_connection_parameters.timeout;
     app_log_info("CONNECTION PARAMETERS: INTERVAL %d, LATENCY %d, TIMEOUT %d ms\r\n", connection_interval, connection_latency, connection_timeout * 10);
    }
    break;
    case sl_bt_evt_gatt_server_attribute_value_id:
    {
        uint16_t attribute = evt->data.evt_gatt_server_attribute_value.attribute;

        switch(attribute)
        {
            case gattdb_oxystat_config:
            {
                oxystat_update_config();

                if (connection_interval == 12)
                {
                  sl_status_t conn_param_status = sl_bt_connection_set_parameters(evt->data.evt_gatt_server_attribute_value.connection, 400, 424, 8, 1800, 0, 0xffff);
                
                  // if this fails, we need to close the connection
                  if (conn_param_status != SL_STATUS_OK)
                  {
                      app_log_warning("failed to set connection parameters: %lu\r\n", conn_param_status);
                      sl_bt_connection_close(evt->data.evt_gatt_server_attribute_value.connection);
                  }
                  else
                  {
                      app_log_info("connection parameters set successfully\r\n");
                  }
  
                  sl_sleeptimer_stop_timer(&ble_parameter_check_timer); // no need to check connection parameters anymore
                }
                
                break;
            }
            case gattdb_ble_reference_time:
            {
                uint64_t uptime_ms = get_uptime_ms();
                set_flexiBLE_reference_time(uptime_ms);
                pause_measurements(false);
                app_log_info("REFERENCE TIME RECORDED: %lu ms\r\n", (uint32_t)uptime_ms);
                break;
            }
            default:
                app_log_info("unhandled attribute value id: %u\r\n", attribute);
                break;
        }
        break;
    }
    case sl_bt_evt_gatt_server_characteristic_status_id:
    {
        uint16_t characteristic = evt->data.evt_gatt_server_characteristic_status.characteristic;
        uint8_t status_flags = evt->data.evt_gatt_server_characteristic_status.client_config_flags;
        uint16_t client_config_flags = evt->data.evt_gatt_server_characteristic_status.client_config_flags;

        app_log_debug("CHARACTERISTIC EVT: %u, STATUS: 0x%x, CONFIG: 0x%x\r\n", characteristic, status_flags, client_config_flags);

        switch(characteristic)
        {
            case gattdb_oxystat_data:
                if (status_flags == sl_bt_gatt_server_confirmation)
                {
                    data_indication_received = true;
                    app_log_info("DATA INDICATION RECEIVED\r\n");
                    oxystat_set_data_notify_enabled(client_config_flags);
                }
                else if (status_flags == sl_bt_gatt_server_client_config)
                {
                    app_log_info("DATA NOTIFICATION ENABLED\r\n");
                    oxystat_set_data_notify_enabled(client_config_flags);
                }
                else
                {
                    app_log_info("UNHANDLED DATA STATUS FLAG: %u\r\n", status_flags);
                }
                break;
            case gattdb_service_changed_char:
                break;
            case gattdb_refresh_reference_time:
                break;
            case gattdb_oxystat_config:
                if (status_flags == sl_bt_gatt_server_client_config)
                {
                    app_log_info("CONFIG NOTIFICATION ENABLED\r\n");
                    oxystat_set_config_notify_enabled(status_flags);
                }
                else
                {
                  app_log_info("UNHANDLED CONFIG STATUS FLAG: %u\r\n", status_flags);
                }

                break;
            default:
                app_log_warning("UNHANDLED CHARACTERISTIC EVT: %u\r\n", characteristic);
                break;
        }
        break;
    }
    case sl_bt_evt_gatt_server_indication_timeout_id:
    {
        app_log_warning("INDICATION TIMEOUT -- closing connection\r\n");
        sl_bt_connection_close(evt->data.evt_gatt_server_indication_timeout.connection);
        break;
    }

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

void wdog_init(void)
{
    // Enabling clock to the interface of the low energy modules (including the Watchdog)
    CMU_ClockEnable(cmuClock_HFLE, true);

    // Watchdog Initialize settings
    WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
    wdogInit.debugRun = true;
    wdogInit.em3Run = true;
    wdogInit.clkSel = wdogClkSelULFRCO;
    wdogInit.perSel = wdogPeriod_16k; // 16384 clock cycles of a 1kHz clock  ~16 seconds period

    // Initializing watchdog with chosen settings
    WDOGn_Init(DEFAULT_WDOG, &wdogInit);
}
