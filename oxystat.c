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

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_prs.h"
#include "em_ldma.h"
#include "em_letimer.h"
#include "em_cryotimer.h"
#include "em_gpio.h"
#include "pin_config.h"
#include "app_log.h"
#include <stdlib.h>
#include <math.h>
#include "sl_bt_api.h"
#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "sl_power_manager.h"
#include "sl_sleeptimer.h"
#include "dmadrv.h"
#include "ecode.h"

#include "flexiBLE_common.h"
#include "oxystat.h"
#include "max5535.h"
#include "utilities.h"
#include "channels.h"
#include "oxystat_service.h"
#include "oxyconfig.h"

// DACs
static DAC_t dac12 = { .cs_port = CS_DAC12_PORT, .cs_pin = CS_DAC12_PIN, .enabled = false, .dutycycle_off = false, .ch_a_mv = 0, .ch_b_mv = 0 };
static DAC_t dac34 = { .cs_port = CS_DAC34_PORT, .cs_pin = CS_DAC34_PIN, .enabled = false, .dutycycle_off = false, .ch_a_mv = 0, .ch_b_mv = 0 };

// MUX
static oxygenation_channels_t current_channel = CH_1;

/* Definess for ADC */
#define ADC_CLOCK               1000000                 /* ADC conversion clock */
#define ADC_ASYNC_CLOCK         cmuAUXHFRCOFreq_4M0Hz   /* Clock for ASYNC mode */
#define ADC_CHxP                adcPosSelAPORT3XCH10    /* PA2 - CHxP */
#define ADC_CHxN                adcPosSelAPORT3XCH12    /* PA3 - CHxN */
#define ADC_REFDAC12            adcPosSelAPORT1XCH22    /* PD10 */
#define ADC_REFDAC34            adcPosSelAPORT2XCH23    /* PD11 */
#define ADC_PRS_CH_SELECT       adcPRSSELCh3
#define ADC_SCAN_DVL            2
#define ADC_BUFFER_SIZE         256
#define ADC_SE_VFS              2.5                     /* AVDD */
#define ADC_12BIT_MAX           4096                    /* 2^12 */
#define ADC_16BIT_MAX           65536                   /* 2^16 */

volatile uint32_t adc_buffer[NUM_CHANNELS][ADC_BUFFER_SIZE] = {0};

channel_measurement_t measurement_buffer[MEASUREMENT_BUFFER_SIZE] = {0};
volatile uint16_t measurement_buffer_read_cursor = 0;
volatile uint16_t measurement_buffer_write_cursor = 0;

// Hardware defines
#define FEEDBACK_RESISTOR 24000.0 // 24kOhm

// Measurement timers
sl_sleeptimer_timer_handle_t start_measurement_timer;

// Oxygenation timer
sl_sleeptimer_timer_handle_t oxygenation_period_timer;
sl_sleeptimer_timer_handle_t oxygenation_duty_cycle_timer;

// LDMA
LDMA_Descriptor_t ldma_descriptor;
unsigned int dma_channel;
typedef struct ldma_callback_data
{
  oxygenation_channels_t channel;
  volatile uint32_t* buffer;
  uint8_t size;
  uint64_t first_timestamp;
} ldma_callback_data_t;
ldma_callback_data_t ldma_callback_data;

// Function Prototypes // 
static void process_dma_buffer_(ldma_callback_data_t* ldma_data);
static bool adc_dma_cb_(unsigned int channel, unsigned int sequenceNo, void* userParam);
static void letimer_init_prs_();
static void adc_init_();
static void ldma_init_(ldma_callback_data_t* ldma_data);
static void mux_init_();
static void mux_select_channel_(oxygenation_channels_t channel);
static void start_measurement_timer_cb(sl_sleeptimer_timer_handle_t *handle, void *data);
static void enable_measurement_(bool enable);
static void enable_adc_timer_(bool enable);
static void set_measurement_hz_(uint16_t hz);
static void set_measurement_period_(uint16_t period);
static void set_oxygenation_period_(uint16_t period);
static void oxygenation_period_timer_cb(sl_sleeptimer_timer_handle_t *handle, void *data);
static void oxygenation_duty_cycle_timer_cb(sl_sleeptimer_timer_handle_t *handle, void *data);
static void refresh_dac_channel_states_();
static void set_oxy_channel_voltage_(oxygenation_channels_t channel, uint16_t millivolts);
static uint32_t adc_offset_calibration_(void);
static void dma_err_parser_(Ecode_t err_code);

// Public Functions //

/**
 * @brief initialize required features
 */
void oxystat_init()
{
  // Calibrate ADC 
  adc_offset_calibration_();

  // Initialize ADC with correct device settings 
  adc_init_();

  // Initialize LETIMER
  letimer_init_prs_();

  // initialize MUX
  mux_init_();

  // initialize DACs
  max5535_init(&dac12);
  max5535_init(&dac34);

  // allocate DMA channel
  Ecode_t err = DMADRV_AllocateChannel(&dma_channel, NULL);
  dma_err_parser_(err);

  app_log_info("OXYSTAT INITIALIZED\r\n");
}

/**
 * @brief transmit data from the measurement buffer to the BLE central
 * @returns true if we've sent all the data, false if there's more to send
 */
bool process_measurement_buffer()
{
  uint16_t original_read_cursor;
  uint16_t write_cursor;

  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_ATOMIC();
    original_read_cursor = measurement_buffer_read_cursor;
    write_cursor = measurement_buffer_write_cursor;
  CORE_EXIT_ATOMIC();

  if (original_read_cursor == write_cursor)
  {
    return true;
  }

  while(true)
  {
    channel_measurement_t reading;

    CORE_ENTER_ATOMIC(); // protect shared variables from being modified while reading
    {
      uint16_t read_cursor = measurement_buffer_read_cursor;
      uint16_t write_cursor = measurement_buffer_write_cursor;

      if (read_cursor == write_cursor)
      {
        CORE_EXIT_ATOMIC();
        // flush the ble buffer at the end to keep the timing accurate
        flush_potentiostat_buffer(false);
        app_log_debug("buffer flushed. read from %u to %u\r\n", original_read_cursor, write_cursor);
        return true;
      }

      reading = measurement_buffer[read_cursor];

      measurement_buffer_read_cursor = (read_cursor + 1) % MEASUREMENT_BUFFER_SIZE;
    }
    CORE_EXIT_ATOMIC();

    // send the reading to the BLE service
    if (buffer_potentiostat_reading(reading.ch, reading.CHxP_counts, reading.CHxN_counts, reading.i_adc, reading.timestamp))
    {
      app_log_debug("buffer sent. read from %u to %u\r\n", original_read_cursor, write_cursor);
      break;
    }
  }

  return false;
}

/**
 * @brief callback for when the configuration is updated via BLE
 */
void handle_config_update() // TODO how do we handle a config update that takes place during a measurement? we assume some things about the data we process based on the current config settings.
{
    // Update state
    enable_measurement_(get_potentiostat_state());

    // Update measurement period
    // An important check to make is that 1/measurement_frequency * num_samples < measurement_period/4. 
    // This ensures that the measurement period is long enough to capture the desired number of samples at the desired frequency.
    // If this condition is not met, the measurement period will be set to the minimum value that satisfies the condition.

    if (get_potentiostat_measurement_period_s() < (uint16_t)((float)get_potentiostat_measurement_samples() / (float)get_potentiostat_measurement_hz()) * 4.0)
    {
        set_potentiostat_measurement_period_s((uint16_t)((float)get_potentiostat_measurement_samples() / (float)get_potentiostat_measurement_hz()) * 4.0 + 8); // add 8s to allow time for tasks that can't happen during a measurement (e.g. reference time refresh)
        app_log_warning("Measurement period too short for desired samples and frequency, setting to %u s\r\n", get_potentiostat_measurement_period_s());
    }
    set_measurement_period_(get_potentiostat_measurement_period_s());

    // Update measurement frequency
    set_measurement_hz_(get_potentiostat_measurement_hz());

    // Update channel voltages
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
      set_oxy_channel_voltage_((oxygenation_channels_t)i, get_channel_mv((oxygenation_channels_t)i));
    }

    // Update oxygenation period
    set_oxygenation_period_(get_oxygenation_period_ms());

    // Update channel states
    refresh_dac_channel_states_();

    // Notify the BLE central of the updated configuration
    notify_config_update();
}


void oxystat_enter_low_power_mode()
{
    app_log_info("MAGNETIC SWITCH TRIPPED -- HIBERNATING\r\n");

    // turn off radio
    uint8_t connection = get_connection_handle();
    sl_bt_connection_close(connection);

    sl_bt_advertiser_stop(0xff); // 0xff is the default advertising set handle that we're using

    // turn off oxygenators and measurement pins
    max5535_enable(&dac12, false);
    max5535_enable(&dac34, false);

    // initialize EM4
    EMU_EM4Init_TypeDef em4init;
    em4init.em4State = emuEM4Shutoff;
    em4init.retainLfxo = false;
    em4init.retainLfrco = false;
    em4init.retainUlfrco = false;
    em4init.pinRetentionMode = emuPinRetentionEm4Exit;

    EMU_EM4Init(&em4init);

    // enable EM4 pin wakeup from magnetic switch pin
    GPIO_EM4WUExtIntConfig(MAG_SW_PORT, MAG_SW_PIN, 8, 1, true);

    // Flash LED 5 times to signal magnetic shutdown
    for (int i = 0; i < 5; i++)
    {
        sl_led_turn_on(&sl_led_debug_led);
        sl_sleeptimer_delay_millisecond(5);
        sl_led_turn_off(&sl_led_debug_led);
        sl_sleeptimer_delay_millisecond(50);
    }

    // Enter EM4S
    GPIO_IntClear(GPIO_IntGet());
    EMU_EnterEM4S();
}

bool is_measurement_in_progress()
{
  bool transfer_active;
  DMADRV_TransferActive(dma_channel, &transfer_active);

  return transfer_active;
}

uint16_t get_unsent_measurements_count()
{
  // Buffer size is 256, so we can use a mask of 0xFF (which is equivalent to MEASUREMENT_BUFFER_SIZE - 1)
  return (measurement_buffer_write_cursor - measurement_buffer_read_cursor) & 0xFF;
}

void pause_measurements(bool pause)
{
  enable_measurement_(!pause && get_potentiostat_state());
}

// Private Functions //

static void process_dma_buffer_(ldma_callback_data_t* data)
{
  double chxp_counts_sum = 0;
  double chxn_counts_sum = 0;

  uint16_t num_samples = data->size + 1; // +1 because we throw out the first two measurements (adc warmup/dummy samples)
  app_log_debug("Processing %u samples\r\n", num_samples);

  for (uint8_t i = 1; i < num_samples; i++) // start at 1 to skip the first two samples
  {
    uint16_t CHxP_counts = 0;
    uint16_t CHxN_counts = 0;

    for (uint8_t j = 0; j < 2; j++)
    {
      uint32_t chId = (data->buffer[i * 2 + j] & _ADC_SCANDATAX_SCANINPUTID_MASK) >> _ADC_SCANDATAX_SCANINPUTID_SHIFT;
      uint32_t chSample = data->buffer[i * 2 + j] & _ADC_SCANDATAX_DATA_MASK;

      if (chId == 12)
      {
        CHxN_counts = chSample;
      }
      else if (chId == 2)
      {
        CHxP_counts = chSample;
      }
      else
      {
        return; // no more data to process
      }
    }

    // sum the counts for the averages
    chxp_counts_sum += CHxP_counts;
    chxn_counts_sum += CHxN_counts;

    // convert the counts to volts
    double CHxP_v = CHxP_counts * (double)ADC_SE_VFS / (double)ADC_16BIT_MAX;
    double CHxN_v = CHxN_counts * (double)ADC_SE_VFS / (double)ADC_16BIT_MAX;

    // calculate the oxygenation current
    double current = (double)(CHxP_v - CHxN_v) / FEEDBACK_RESISTOR;
    int32_t current_na = (int32_t)(current * (double)1e9);

    // keep processing individual readings if we're not in averaging mode
    if (get_average_samples_enabled() == false)
    {
      // calculate timestamp
      double reading_timestamp_ms = data->first_timestamp + (double)i * (double)1000 / (double)get_potentiostat_measurement_hz();
      uint64_t reading_timestamp_ms_u64 = (uint64_t)reading_timestamp_ms;

      // put the reading into the local buffer. calculate next write index.
      channel_measurement_t reading = { .ch = data->channel, .CHxP_counts = CHxP_counts, .CHxN_counts = CHxN_counts, .i_adc = current_na, .timestamp = reading_timestamp_ms_u64 };
      
      CORE_DECLARE_IRQ_STATE;
      CORE_ENTER_ATOMIC();
      {
        uint16_t next_write_cursor = (measurement_buffer_write_cursor + 1) % MEASUREMENT_BUFFER_SIZE;

        if (next_write_cursor == measurement_buffer_read_cursor)
        {
          app_log_error("Measurement buffer full, overwriting old data\r\n");
        }

        measurement_buffer[measurement_buffer_write_cursor] = reading;
        measurement_buffer_write_cursor = next_write_cursor;
      }
      CORE_EXIT_ATOMIC();

      // log data
    
      // if we're on the last sample, erase the buffer
      if (i == num_samples - 1)
      {
        app_log_info("LAST RAW // index: %u, channel: %u, CHxP: %u counts, CHxN: %u counts, I: %ld nA, timestamp: %lu ms\r\n", i, reading.ch, reading.CHxP_counts, reading.CHxN_counts, reading.i_adc, (uint32_t)reading.timestamp);
        memset((void*)data->buffer, 0, data->size * sizeof(uint32_t));
        app_log_debug("DMA buffer cleared\r\n");
      }
    }
  }

  // if we're in averaging mode, calculate the averages and send them to the BLE service
  if (get_average_samples_enabled() == true)
  {

    // calculate the average voltages
    double average_chxp_counts = chxp_counts_sum / data->size;
    double average_chxn_counts = chxn_counts_sum / data->size;

    // calculate the average voltages
    double average_chxp_v = average_chxp_counts * (double)ADC_SE_VFS / (double)ADC_16BIT_MAX;
    double average_chxn_v = average_chxn_counts * (double)ADC_SE_VFS / (double)ADC_16BIT_MAX;

    // calculate the average oxygenation current
    double average_current_na = ( (average_chxp_v - average_chxn_v) / FEEDBACK_RESISTOR ) * (double)1e9;
    //convert to int32_t
    int32_t average_current_na_u32 = (int32_t)average_current_na;

      // put the reading into the local buffer. calculate next write index.
      channel_measurement_t reading = { .ch = data->channel, .CHxP_counts = average_chxp_counts, .CHxN_counts = average_chxn_counts, .i_adc = average_current_na_u32, .timestamp = data->first_timestamp };

      CORE_DECLARE_IRQ_STATE;
      CORE_ENTER_ATOMIC();
      {
        uint16_t next_write_cursor = (measurement_buffer_write_cursor + 1) % MEASUREMENT_BUFFER_SIZE;

        if (next_write_cursor == measurement_buffer_read_cursor)
        {
          app_log_error("Measurement buffer full, overwriting old data\r\n");
        }

        measurement_buffer[measurement_buffer_write_cursor] = reading;
        measurement_buffer_write_cursor = next_write_cursor;
      }
      CORE_EXIT_ATOMIC();

      app_log_info("AVERAGED // channel: %u, CHxP: %u counts, CHxN: %u counts, I: %ld nA, timestamp: %lu ms\r\n", reading.ch, reading.CHxP_counts, reading.CHxN_counts, reading.i_adc, (uint32_t)reading.timestamp);
  }
}

/**
 * @brief callback for DMA transfer completion
 */
static bool adc_dma_cb_(unsigned int channel, unsigned int sequenceNo, void* userParam)
{
  (void)userParam;

  app_log_debug("adc_dma_cb_()\r\n");
  app_log_debug("dma channel: %u, sequenceNo: %u\r\n", channel, sequenceNo);
  app_log_debug("oxy channel: %u, size: %u, first timestamp: %lu\r\n", ldma_callback_data.channel, ldma_callback_data.size, (uint32_t)ldma_callback_data.first_timestamp);

  // stop the ADC timer
  enable_adc_timer_(false);

  // stop the dma transfer
  DMADRV_StopTransfer(dma_channel);

  process_dma_buffer_((ldma_callback_data_t*)userParam);

  // increment the channel
  oxygenation_channels_t next_channel = (oxygenation_channels_t)((current_channel + 1) % NUM_CHANNELS);
  mux_select_channel_(next_channel);

  return true;
}

/**************************************************************************//**
 * @brief Initialize LETIMER0 to drive the PRS
 *****************************************************************************/
static void letimer_init_prs_()
{
  // Start LFRCO and wait until it is stable
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

  // Enable clock to the interface of the low energy modules
  CMU_ClockEnable(cmuClock_HFLE, true);

  // Route the LFRCO clock to LFA (TIMER0)
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);

  // Enable clock for LETIMER0
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  // Configure LETIMER0
  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

  // Reload COMP0 on underflow, pulse output, and run in repeat mode
  letimerInit.enable    = false;
  letimerInit.comp0Top  = true;
  letimerInit.ufoa0     = letimerUFOAPulse;
  letimerInit.repMode   = letimerRepeatFree;

  LETIMER_Init(LETIMER0, &letimerInit);

  // Need REP0 != 0 to pulse on underflow
  LETIMER_RepeatSet(LETIMER0, 0, 1);

  // calculate the topValue
  uint32_t topValue = CMU_ClockFreqGet(cmuClock_LETIMER0) / get_potentiostat_measurement_hz();

  LETIMER_CompareSet(LETIMER0, 0, topValue);

  // Configure PRS to route LETIMER0 output signal to the ADC (if using PRS)
  PRS_SourceAsyncSignalSet(ADC_PRS_CH_SELECT, PRS_CH_CTRL_SOURCESEL_LETIMER0, PRS_CH_CTRL_SIGSEL_LETIMER0CH0);
}

/**************************************************************************//**
 * @brief Initialize ADC for single and scan conversion
 *****************************************************************************/
void adc_init_(void)
{
  /* Enable ADC clock */
  CMU_ClockEnable(cmuClock_ADC0, true);
  // CMU_ClockEnable(cmuClock_HFPER, true);

  /* Select AUXHFRCO for ADC ASYNC mode so that ADC can run on EM2 */
  CMU->ADCCTRL = CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO;

  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

  // Set AUXHFRCO frequency and use it to setup the ADC
  CMU_AUXHFRCOBandSet(ADC_ASYNC_CLOCK);
  init.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, CMU_AUXHFRCOBandGet());
  init.em2ClockConfig = adcEm2ClockOnDemand;
  init.ovsRateSel = adcOvsRateSel64;
  init.warmUpMode = adcWarmupNormal;

  /* Set up scan channels, define DEBUG_EFM in debug build to identify invalid channel range */
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_CHxP);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_CHxN);

  scanInit.diff = false; // Single-ended input
  scanInit.reference = adcRef2V5; // 2.5V reference
  scanInit.resolution = adcResOVS; // 12-bit resolution
  scanInit.acqTime = adcAcqTime16; // Set acquisition time based on impedance/capacitance/frequency of the signal

  /* DMA is available in EM2 for processing SCANFIFO DVL request */
  scanInit.scanDmaEm2Wu = 1;

  /* Enable PRS trigger and select the same channel as LETIMER0 */
  scanInit.prsEnable = true;
  scanInit.prsSel = ADC_PRS_CH_SELECT;


  ADC_Init(ADC0, &init);
  ADC_InitScan(ADC0, &scanInit);

  /* Set scan data valid level (DVL) to trigger DMA*/
  ADC0->SCANCTRLX |= (ADC_SCAN_DVL - 1) << _ADC_SCANCTRLX_DVL_SHIFT;

  /* Clear the SCAN FIFO */
  ADC0->SCANFIFOCLEAR = ADC_SCANFIFOCLEAR_SCANFIFOCLEAR;

  // /* Use LOWACC if not using bandgap reference to reduce current consumption */ 
  ADC0->BIASPROG = ADC_BIASPROG_GPBIASACC;
}

static void ldma_init_(ldma_callback_data_t *ldma_data)
{
  uint16_t real_size = ldma_data->size * 2 + 2; // each measurement is 2 samples (CHxP and CHxN), plus add 1 set of samples for warmup/dummy data

  app_log_debug("ldma_init_() // channel: %u, size: %u\r\n", ldma_data->channel, real_size);

  if (real_size > ADC_BUFFER_SIZE)
  {
    app_log_error("DMA buffer size too large\r\n");
    real_size = ADC_BUFFER_SIZE;
  }

  Ecode_t err;

  LDMA_TransferCfg_t transfer_config = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_ADC0_SCAN);

  ldma_descriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&ADC0->SCANDATAX, ldma_data->buffer, real_size, 0);
  ldma_descriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit2;
  ldma_descriptor.xfer.ignoreSrec = 1;

  err = DMADRV_LdmaStartTransfer(dma_channel, &transfer_config, &ldma_descriptor, adc_dma_cb_, ldma_data);
  dma_err_parser_(err);

  // Clear pending and enable interrupts for channel
  NVIC_ClearPendingIRQ(LDMA_IRQn);
  NVIC_EnableIRQ(LDMA_IRQn);
}

static void mux_init_()
{
  GPIO_PinModeSet(CHMUX_EN_PORT, CHMUX_EN_PIN, gpioModePushPull, 1);
  GPIO_PinModeSet(CHMUX_A0_PORT, CHMUX_A0_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CHMUX_A1_PORT, CHMUX_A1_PIN, gpioModePushPull, 0);

  mux_select_channel_(CH_1);

  app_log_info("MUX initialized\r\n");
}

void mux_select_channel_(oxygenation_channels_t channel)
{
    switch(channel)
    {
        case CH_1: // S4
            GPIO_PinOutSet(CHMUX_A0_PORT, CHMUX_A0_PIN);
            GPIO_PinOutSet(CHMUX_A1_PORT, CHMUX_A1_PIN);
            break;
        case CH_2: // S3
            GPIO_PinOutSet(CHMUX_A0_PORT, CHMUX_A0_PIN);
            GPIO_PinOutClear(CHMUX_A1_PORT, CHMUX_A1_PIN);
            break;
        case CH_3: // S2
            GPIO_PinOutClear(CHMUX_A0_PORT, CHMUX_A0_PIN);
            GPIO_PinOutSet(CHMUX_A1_PORT, CHMUX_A1_PIN);
            break;
        case CH_4: // S1
            GPIO_PinOutClear(CHMUX_A0_PORT, CHMUX_A0_PIN);
            GPIO_PinOutClear(CHMUX_A1_PORT, CHMUX_A1_PIN);
            break;
        default:
            break;
    }

    GPIO_PinOutSet(CHMUX_EN_PORT, CHMUX_EN_PIN);

    app_log_debug("MUX channel selected: %u\r\n", (uint8_t)channel + 1);

    // log which pins are set
    app_log_debug("CHMUX_EN: %u, CHMUX_A0: %u, CHMUX_A1: %u\r\n", GPIO_PinOutGet(CHMUX_EN_PORT, CHMUX_EN_PIN), GPIO_PinOutGet(CHMUX_A0_PORT, CHMUX_A0_PIN), GPIO_PinOutGet(CHMUX_A1_PORT, CHMUX_A1_PIN));

    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_ATOMIC();
      current_channel = channel;
    CORE_EXIT_ATOMIC();
}

static void start_measurement_timer_cb(sl_sleeptimer_timer_handle_t* handle, void* data)
{
  (void)handle;
  (void)data;

  app_log_debug("Measurement timer fired\r\n");

  // configure the DMA for the correct number of samples
  ldma_callback_data.buffer = adc_buffer[current_channel];
  ldma_callback_data.size = get_potentiostat_measurement_samples();
  ldma_callback_data.channel = current_channel;
  ldma_callback_data.first_timestamp = get_uptime_ms();
  ldma_init_(&ldma_callback_data);

  // start the LETIMER to trigger the ADC at the configured frequency
  enable_adc_timer_(true);

  // start the measurement timer to start the next measurement
  uint32_t period_ms = (get_potentiostat_measurement_period_s() * 1000) / NUM_CHANNELS; // convert to ms, account for 4 channels
  app_log_debug("Starting measurement timer with period %lu\r\n", period_ms);
  sl_sleeptimer_start_timer_ms(&start_measurement_timer, period_ms, start_measurement_timer_cb, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

static void enable_measurement_(bool enable)
{
  if (enable)
  {
    // if the timer is already running, don't start it again
    bool timer_running;
    sl_sleeptimer_is_timer_running(&start_measurement_timer, &timer_running);
    if (timer_running)
    {
      // how much time is left on the timer?
      uint32_t time_left_ms;
      sl_sleeptimer_get_timer_time_remaining(&start_measurement_timer, &time_left_ms);
      app_log_info("Measurement already enabled: %lu ms left on timer\r\n", time_left_ms);
      return;
    }

    // if the timer isn't running, start it
    app_log_debug("Starting measurement timer\r\n");
    uint32_t period_ms = (get_potentiostat_measurement_period_s() * 1000) / NUM_CHANNELS; // convert to ms, account for 4 channels
    sl_sleeptimer_start_timer_ms(&start_measurement_timer, period_ms, start_measurement_timer_cb, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG); // we'll kick it off with an initial delay of 1s before using the configured period
  }
  else
  {
    sl_sleeptimer_stop_timer(&start_measurement_timer);
  }
}

static void enable_adc_timer_(bool enable)
{
  if (enable)
  {

    CMU_ClockEnable(cmuClock_LETIMER0, true);
    app_log_debug("ADC timer enable\r\n");
    LETIMER_Enable(LETIMER0, true);
  }
  else
  {
    app_log_debug("ADC timer disable\r\n");
    LETIMER_Enable(LETIMER0, false);
    
    // Disable clock for LETIMER0
    CMU_ClockEnable(cmuClock_LETIMER0, false);
  }
}

static void set_measurement_hz_(uint16_t hz)
{
  // calculate new topvalue
  uint32_t topValue = CMU_ClockFreqGet(cmuClock_LETIMER0) / hz;

  LETIMER_CompareSet(LETIMER0, 0, topValue);
}

static void set_measurement_period_(uint16_t period)
{
  // if the measurement is currently running, stop it and update the timer
  bool timer_running;
  sl_sleeptimer_is_timer_running(&start_measurement_timer, &timer_running);
  if (timer_running)
  {
    sl_sleeptimer_stop_timer(&start_measurement_timer);
    uint32_t period_ms = (period * 1000) / NUM_CHANNELS; // convert to ms, account for 4 channels
    sl_sleeptimer_start_timer_ms(&start_measurement_timer, period_ms, start_measurement_timer_cb, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
  }
  // otherwise, don't worry about it because the timer will be started with the new period when the measurement is enabled
}

static void set_oxygenation_period_(uint16_t period)
{
  // stop the period timer if it's running
  sl_sleeptimer_stop_timer(&oxygenation_period_timer);

  // restart with the new period
  sl_sleeptimer_start_timer_ms(&oxygenation_period_timer, period, oxygenation_period_timer_cb, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

static void oxygenation_period_timer_cb(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

  app_log_debug("Oxygenation period timer fired\r\n");

  // turn oxygenation on
  dac12.dutycycle_off = false;
  dac34.dutycycle_off = false;
  refresh_dac_channel_states_();

  // restart the period timer
  sl_sleeptimer_start_timer_ms(&oxygenation_period_timer, get_oxygenation_period_ms(), oxygenation_period_timer_cb, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
  
  if (get_oxygenation_duty_cycle() == 100)
  {
    return; // call it a day here
  }

  // calculate duty cycle timer period
  uint8_t duty_cycle_percent = get_oxygenation_duty_cycle();
  uint32_t duty_cycle_period_ms = (get_oxygenation_period_ms() * duty_cycle_percent) / 100;
  app_log_debug("Starting oxygenation duty cycle timer with period %lu\r\n", duty_cycle_period_ms);
  
  // start the duty cycle timer
  sl_sleeptimer_start_timer_ms(&oxygenation_duty_cycle_timer, duty_cycle_period_ms, oxygenation_duty_cycle_timer_cb, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

static void oxygenation_duty_cycle_timer_cb(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

  app_log_debug("Oxygenation duty cycle timer fired\r\n");

  // turn oxygenation off
  dac12.dutycycle_off = true;
  dac34.dutycycle_off = true;
  refresh_dac_channel_states_();
}

static void refresh_dac_channel_states_()
{
  // we disable the channel by disabling the DAC that controls it. If either channel is disabled, we disable the DAC.
  // CH1 and CH2 are controlled by DAC12, CH3 and CH4 are controlled by DAC34
  max5535_enable(&dac12, (get_channel_state(CH_1) && get_channel_state(CH_2)) && dac12.dutycycle_off == false); // only enable if both channels are enabled
  max5535_enable(&dac34, (get_channel_state(CH_3) && get_channel_state(CH_4)) && dac34.dutycycle_off == false); // only enable if both channels are enabled

  // set the voltages for the enabled channels
  set_oxy_channel_voltage_(CH_1, get_channel_mv(CH_1));
  set_oxy_channel_voltage_(CH_2, get_channel_mv(CH_2));
  set_oxy_channel_voltage_(CH_3, get_channel_mv(CH_3));
  set_oxy_channel_voltage_(CH_4, get_channel_mv(CH_4));
  
  app_log_info("DAC12 state: %u\r\n", dac12.enabled);
  app_log_info("DAC34 state: %u\r\n", dac34.enabled);
}

// set oxygenator voltage
static void set_oxy_channel_voltage_(oxygenation_channels_t channel, uint16_t millivolts)
{
    DAC_t* parent_dac;
    max5535_ch_t dac_channel;

    switch (channel)
    {
      case CH_1:
        parent_dac = &dac12;
        dac_channel = MAX5535_CH_A;
        if (dac12.ch_a_mv == millivolts)
        {
          return;
        }
        break;
      case CH_2:
        parent_dac = &dac12;
        dac_channel = MAX5535_CH_B;
        if (dac12.ch_b_mv == millivolts)
        {
          return;
        }
        break;
      case CH_3:
        parent_dac = &dac34;
        dac_channel = MAX5535_CH_A;
        if (dac34.ch_a_mv == millivolts)
        {
          return;
        }
        break;
      case CH_4:
        parent_dac = &dac34;
        dac_channel = MAX5535_CH_B;
        if (dac34.ch_b_mv == millivolts)
        {
          return;
        }
        break;
      default:
        return;
    }

    if (!parent_dac->enabled)
    {
      app_log_error("DAC not enabled for channel %u\r\n", channel);
      return;
    }
    else if (parent_dac->dutycycle_off)
    {
      app_log_error("DAC duty cycle off for channel %u\r\n", channel);
      return;
    }

    // set voltage
    set_channel_voltage(parent_dac, dac_channel, millivolts);
}

/***************************************************************************//**
 * @brief
 *   Calibrate offset for the specified reference.
 *
 * @details
 *   The offset calibration routine measures 0 V with the ADC, and adjust
 *   the calibration register until the converted value equals 0.
 *
 *
 * @return
 *   The final value of the calibration register, note that the calibration
 *   register gets updated with this value during the calibration.
 *   No need to load the calibration values after the function returns.
 ******************************************************************************/
static uint32_t adc_offset_calibration_(void)
{
  int32_t  sample;
  uint32_t cal;

  /* Binary search variables */
  uint8_t high;
  uint8_t mid;
  uint8_t low;

  /* Reset ADC to be sure we have default settings and wait for ongoing */
  /* conversions to be complete. */
  ADC_Reset(ADC0);

  ADC_Init_TypeDef ADC0_init = ADC_INIT_DEFAULT;
  /* Init common settings for both single conversion and scan mode */
  ADC0_init.timebase = ADC_TimebaseCalc(0);
  ADC0_init.prescale = ADC_PrescaleCalc(11000000, 0);

  /* Set an oversampling rate for more accuracy */
  ADC0_init.ovsRateSel = adcOvsRateSel4096;
  ADC_Init(ADC0, &ADC0_init);

  ADC_InitSingle_TypeDef ADC0_init_single = ADC_INITSINGLE_DEFAULT;
  /* Init for single conversion use, measure DIFF0 with selected reference. */
  ADC0_init_single.reference = adcRef2V5;
  ADC0_init_single.posSel = adcPosSelVSS;
  ADC0_init_single.negSel = adcNegSelVSS;
  ADC0_init_single.acqTime   = adcAcqTime16;
  ADC0_init_single.diff      = false;
  /* Enable oversampling rate */
  ADC0_init_single.resolution = adcResOVS;

  ADC_InitSingle(ADC0, &ADC0_init_single);


   /* ADC is now set up for offset calibration */
  /* Offset calibration register is a 7 bit signed 2's complement value. */
  /* Use unsigned indexes for binary search, and convert when calibration */
  /* register is written to. */
  high = 128;
  low  = 0;

  /* Do binary search for offset calibration*/
  while (low < high)
  {
    /* Calculate midpoint */
    mid = low + (high - low) / 2;

    /* Midpoint is converted to 2's complement and written to both scan and */
    /* single calibration registers */
    cal      = ADC0->CAL & ~(_ADC_CAL_SINGLEOFFSET_MASK | _ADC_CAL_SCANOFFSET_MASK);
    cal     |= (uint8_t)(mid - 63) << _ADC_CAL_SINGLEOFFSET_SHIFT;
    cal     |= (uint8_t)(mid - 63) << _ADC_CAL_SCANOFFSET_SHIFT;
    ADC0->CAL = cal;

    /* Do a conversion */
    ADC_Start(ADC0, adcStartSingle);
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT)
      ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(ADC0);

    /* Check result and decide in which part to repeat search */
    /* Calibration register has negative effect on result */
    if (sample < 0)
    {
      /* Repeat search in bottom half. */
      high = mid;
    }
    else if (sample > 0)
    {
      /* Repeat search in top half. */
      low = mid + 1;
    }
    else
    {
      /* Found it, exit while loop */
      break;
    }
  }

  app_log_info("ADC offset calibration complete\r\n");

  return ADC0->CAL;
}

static void dma_err_parser_(Ecode_t err_code)
{
  switch(err_code)
  {
    case ECODE_EMDRV_DMADRV_OK:
      app_log_debug("DMA operation successful\r\n");
      break;
    case ECODE_EMDRV_DMADRV_PARAM_ERROR:
      app_log_error("DMA operation failed: illegal input parameter\r\n");
      break;
    case ECODE_EMDRV_DMADRV_NOT_INITIALIZED:
      app_log_error("DMA operation failed: DMA not initialized\r\n");
      break;
    case ECODE_EMDRV_DMADRV_ALREADY_INITIALIZED:
      app_log_error("DMA operation failed: DMA already initialized\r\n");
      break;
    case ECODE_EMDRV_DMADRV_CHANNELS_EXHAUSTED:
      app_log_error("DMA operation failed: no DMA channels available\r\n");
      break;
    case ECODE_EMDRV_DMADRV_IN_USE:
      app_log_error("DMA operation failed: DMA in use\r\n");
      break;
    case ECODE_EMDRV_DMADRV_ALREADY_FREED:
      app_log_error("DMA operation failed: DMA channel already freed\r\n");
      break;
    case ECODE_EMDRV_DMADRV_CH_NOT_ALLOCATED:
      app_log_error("DMA operation failed: channel not reserved\r\n");
      break;
    default:
      app_log_error("DMA operation failed: unknown error code\r\n");
      break;
  }
}
