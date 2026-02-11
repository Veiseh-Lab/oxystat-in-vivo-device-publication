/***************************************************************************//**
 * @file
 * @brief Blink examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "sl_sleeptimer.h"
#include "oxyconfig.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#ifndef LED_INSTANCE
#define LED_INSTANCE    sl_led_debug_led
#endif

#define LED_ON_MS 5
#define LED_OFF_MS 5000

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

static sl_sleeptimer_timer_handle_t timer;
static bool led_state = 0;

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

static void on_blink_timeout(sl_sleeptimer_timer_handle_t *handle,
                       void *data);

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * Initialize blink example.
 ******************************************************************************/
void blink_init(void)
{
  // Create timer for waking up the system periodically.
  sl_sleeptimer_start_timer_ms(&timer,
                            LED_OFF_MS,
                            on_blink_timeout, NULL,
                            0,
                            SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

/***************************************************************************//**
 * Sleeptimer timeout callback.
 ******************************************************************************/
static void on_blink_timeout(sl_sleeptimer_timer_handle_t *handle,
                       void *data)
{
  (void)handle;
  (void)data;
  
  if (get_led_enabled()) // led enabled is from the ble config
  {
      if (led_state == 1)
      {
          led_state = 0;
          sl_led_turn_off(&LED_INSTANCE);
          sl_sleeptimer_start_timer_ms(&timer, LED_OFF_MS, on_blink_timeout, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
      }
      else
      {
          led_state = 1;
          sl_led_turn_on(&LED_INSTANCE);
          sl_sleeptimer_start_timer_ms(&timer, LED_ON_MS, on_blink_timeout, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
      }
  }
  else
  {
      if (led_state == 1)
      {
          led_state = 0;
          sl_led_turn_off(&LED_INSTANCE);
      }

      sl_sleeptimer_start_timer_ms(&timer, LED_OFF_MS, on_blink_timeout, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
  }
}

/***************************************************************************//**
 * Flash the LED n times
 ******************************************************************************/
void flash(uint8_t count, uint16_t ms_on, uint16_t ms_off)
{
  for (int i = 0; i < count; i++)
  {
    sl_led_turn_on(&LED_INSTANCE);
    sl_sleeptimer_delay_millisecond(ms_on);
    sl_led_turn_off(&LED_INSTANCE);
    sl_sleeptimer_delay_millisecond(ms_off);
  }
}