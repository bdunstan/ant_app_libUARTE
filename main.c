/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include "ant_bpwr.h"
#include "app_timer.h"
#include "app_util_platform.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_libuarte_async.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_sdh_soc.h"
#include <stdint.h>
#include <string.h>

#include "ant_interface.h"
#include "ant_key_manager.h"
#include "ant_state_indicator.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_error.h"
#include "bsp.h"
#include "bsp_btn_ant.h"
#include "hardfault.h"
#include "nrf_drv_clock.h"

#define DEAD_BEEF 0xDEADBEEF

NRF_LIBUARTE_ASYNC_DEFINE(libuarte, 0, 1, 2, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 255, 4);

/** @snippet [ANT BPWR RX Instance] */
void ant_bpwr_evt_handler(ant_bpwr_profile_t *p_profile, ant_bpwr_evt_t event);

BPWR_DISP_CHANNEL_CONFIG_DEF(m_ant_bpwr,
    BPWR_CHANNEL_NUM,
    CHAN_ID_TRANS_TYPE,
    CHAN_ID_DEV_NUM,
    ANTPLUS_NETWORK_NUM);
BPWR_DISP_PROFILE_CONFIG_DEF(m_ant_bpwr,
    ant_bpwr_evt_handler);

static ant_bpwr_profile_t m_ant_bpwr;
/** @snippet [ANT BPWR RX Instance] */

NRF_SDH_ANT_OBSERVER(m_ant_observer, ANT_BPWR_ANT_OBSERVER_PRIO,
    ant_bpwr_disp_evt_handler, &m_ant_bpwr);

uint8_t event_sent;

/**@brief Function for handling Bicycle Power profile's events
 *
 

typedef struct
{
  uint8_t page_number;
  uint8_t page_payload[7];
} ant_bpwr_message_layout_t;

typedef struct
{
  uint8_t update_event_count;
  uint8_t pedal_power;
  uint8_t reserved;
  uint8_t accumulated_power[2];
  uint8_t instantaneous_power[2];
} ant_bpwr_page16_data_layout_t;
*/

void ant_bpwr_evt_handler(ant_bpwr_profile_t *p_profile, ant_bpwr_evt_t event) {
  uint32_t err_code;

  nrf_pwr_mgmt_feed();

  ant_bpwr_page16_data_t *p_page_buffer;

  switch (event) {
  case ANT_BPWR_PAGE_1_UPDATED:
    // calibration data received from sensor
    NRF_LOG_INFO("ANT_BPWR_PAGE_1_UPDATED");
    break;

  case ANT_BPWR_PAGE_16_UPDATED:

    p_page_buffer = &(p_profile->page_16);
   
    if (event_sent != p_page_buffer->update_event_count) {

    NRF_LOG_INFO("event:                   %d", p_page_buffer->update_event_count);

    //NRF_LOG_INFO("accumulated power:       %u W", p_page_buffer->accumulated_power);
    NRF_LOG_INFO("instantaneous power:     %u W", p_page_buffer->instantaneous_power);
    NRF_LOG_HEXDUMP_INFO(p_page_buffer, sizeof(ant_bpwr_page16_data_t));

      err_code = nrf_libuarte_async_tx(&libuarte, (uint8_t *)p_page_buffer, sizeof(ant_bpwr_page16_data_t));
      APP_ERROR_CHECK(err_code);
    }
    event_sent = p_page_buffer->update_event_count;
    break;

  default:
    // never occurred
    break;
  }
}

/* BPWR: local instance */

void ant_disp_evt_handler(ant_evt_t *p_ant_event, void *p_context) {
  ant_bpwr_profile_t *p_profile = (ant_bpwr_profile_t *)p_context;

  if (p_ant_event->channel == p_profile->channel_number) {
    switch (p_ant_event->event) {
    case EVENT_RX:

      if (p_ant_event->message.ANT_MESSAGE_ucMesgID == MESG_BROADCAST_DATA_ID || p_ant_event->message.ANT_MESSAGE_ucMesgID == MESG_ACKNOWLEDGED_DATA_ID || p_ant_event->message.ANT_MESSAGE_ucMesgID == MESG_BURST_DATA_ID) {
        NRF_LOG_INFO("ant_disp_evt_handler:");
        //disp_message_decode(p_profile, p_ant_event->message.ANT_MESSAGE_aucPayload);
      }
      break;

    default:
      break;
    }
    //service_calib(p_profile, p_ant_event->event);
  }
}

/**
 * @brief Function for Bicycle Power profile initialization.
 *
 * @details Initializes the Bicycle Power profile and open ANT channel.
 */
static void profile_setup(void) {

  /** @snippet [ANT BPWR RX Profile Setup] */
  ret_code_t err_code = ant_bpwr_disp_init(&m_ant_bpwr,
      BPWR_DISP_CHANNEL_CONFIG(m_ant_bpwr),
      BPWR_DISP_PROFILE_CONFIG(m_ant_bpwr));
  APP_ERROR_CHECK(err_code);

  err_code = ant_bpwr_disp_open(&m_ant_bpwr);
  APP_ERROR_CHECK(err_code);

  err_code = ant_state_indicator_channel_opened();
  APP_ERROR_CHECK(err_code);

  /** @snippet [ANT BPWR RX Profile Setup] */
}

/* Random Counters for debug */
uint8_t sc = 0;
/** @snippet [ANT BPWR RX Instance] */

void ant_evt_handler(ant_evt_t *p_ant_evt, void *p_context);

// NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, NULL);
/*
typedef struct
{
  uint8_t update_event_count;
  uint8_t pedal_power;
  uint8_t reserved;
  uint8_t accumulated_power[2];
  uint8_t instantaneous_power[2];
} ant_bpwr_page16_data_layout_t;
*/

uint8_t uart_sent[64];

void uart_tx() {
  ret_code_t err_code;

  static uint8_t text[] = {0xac, 0xc8, 0x28, 0x57, 0x12, 0x4a, 0x00};
  static uint8_t text_size = sizeof(text);

  static uint8_t text_string[32];
  sprintf(text_string, "%02X ", *(uint32_t *)text);
  NRF_LOG_INFO("send: %s", text_string);

  for (int i = 0; i < text_size; i++) {
    NRF_LOG_INFO("%02X ", text[i]);
  }
  NRF_LOG_INFO(" (%d)\n", sc);
  NRF_LOG_INFO("Send: %d\n", text_size);
  /*

  ant_bpwr_page16_data_layout_t const *p_incoming_data =
      (ant_bpwr_page16_data_layout_t *)text;

  NRF_LOG_INFO("update_event_count:  %u\n", p_incoming_data->update_event_count);
  NRF_LOG_INFO("pedal_power:         %u\n", p_incoming_data->pedal_power);
  NRF_LOG_INFO("reserved:            %u\n", p_incoming_data->reserved);
  NRF_LOG_INFO("accumulated_power:   %u\n", uint16_decode(p_incoming_data->accumulated_power));
  NRF_LOG_INFO("instantaneous_power: %u\n", uint16_decode(p_incoming_data->instantaneous_power));
  */

  err_code = nrf_libuarte_async_tx(&libuarte, text, text_size);

  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling a ANT stack event.
 *
 * @param[in] p_ant_evt  ANT stack event.
 * @param[in] p_context  Context.
 */
void ant_evt_handler(ant_evt_t *p_ant_evt, void *p_context) {

  ret_code_t err_code;

  if (p_ant_evt->channel == ANT_CHANNEL_NUM) {
    switch (p_ant_evt->event) {
    case EVENT_RX:

        // Really strange, need this (;) or error: see:
        // https://stackoverflow.com/questions/18496282/why-do-i-get-a-label-can-only-be-part-of-a-statement-and-a-declaration-is-not-a
        ;
      //unsigned char *charPtr = (unsigned char *)p_ant_evt;
      ANT_MESSAGE *p_ant_message = (ANT_MESSAGE *)&p_ant_evt->message;

      //NRF_LOG_HEXDUMP_INFO(p_ant_message->ANT_MESSAGE_aucPayload, ANT_STANDARD_DATA_PAYLOAD_SIZE);

      //NRF_LOG_INFO("EVENT_RX : %02X %02X",p_ant_message->ANT_MESSAGE_aucPayload[0], p_ant_message->ANT_MESSAGE_aucPayload[1]);

      // Could use ANT_STANDARD_DATA_PAYLOAD_SIZE for the size ?

      unsigned char *charPtr = (unsigned char *)p_ant_message->ANT_MESSAGE_aucPayload;
      if (p_ant_message->ANT_MESSAGE_aucPayload[0] != 0x10) {
        break;
      }
      /*
      if (uart_sent[(uint8_t)p_ant_message->ANT_MESSAGE_aucPayload[0]] == p_ant_message->ANT_MESSAGE_aucPayload[1]) {
        NRF_LOG_ERROR("Already Sent: %u", p_ant_message->ANT_MESSAGE_aucPayload[1])
        break;
      }
 */

      //uart_sent[(uint8_t)p_ant_message->ANT_MESSAGE_aucPayload[0]] = p_ant_message->ANT_MESSAGE_aucPayload[1];
      //NRF_LOG_ERROR("Size of payload: %u %u", sizeof(p_ant_message->ANT_MESSAGE_aucPayload), p_ant_message->ANT_MESSAGE_ucSize);

      //static uint8_t text[] = { 0x10, 0xac, 0xc8, 0x28, 0x57, 0x12, 0x4a, 0x00 };
      static uint8_t text[ANT_STANDARD_DATA_PAYLOAD_SIZE];
      static uint8_t text_size = sizeof(text);
      static uint8_t text_string[32];

      NRF_LOG_HEXDUMP_INFO(p_ant_message->ANT_MESSAGE_aucPayload, ANT_STANDARD_DATA_PAYLOAD_SIZE);
      NRF_LOG_INFO("%d", p_ant_message->ANT_MESSAGE_aucPayload[1]);
      //printf("Payload 1 (%d)\n", p_ant_message->ANT_MESSAGE_aucPayload[1]);

      for (int i = 0; i < ANT_STANDARD_DATA_PAYLOAD_SIZE; i++) {
        //NRF_LOG_INFO("%02X ", p_ant_message->ANT_MESSAGE_aucPayload[i]);
        text[i] = p_ant_message->ANT_MESSAGE_aucPayload[i];
      }
      //NRF_LOG_INFO("Sent: %d %d (%d)", p_ant_message->ANT_MESSAGE_aucPayload[1], ANT_STANDARD_DATA_PAYLOAD_SIZE, sc);

      sc++;

      //err_code = nrf_libuarte_async_tx(&libuarte, (uint8_t *)&p_ant_message->ANT_MESSAGE_aucPayload, ANT_STANDARD_DATA_PAYLOAD_SIZE);

      err_code = nrf_libuarte_async_tx(&libuarte, text, text_size);
      APP_ERROR_CHECK(err_code);

      if (p_ant_evt->message.ANT_MESSAGE_ucMesgID == MESG_BROADCAST_DATA_ID || p_ant_evt->message.ANT_MESSAGE_ucMesgID == MESG_ACKNOWLEDGED_DATA_ID || p_ant_evt->message.ANT_MESSAGE_ucMesgID == MESG_BURST_DATA_ID) {
        // LED 2 is inverted for 100 msec
        err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
        APP_ERROR_CHECK(err_code);
      }

      break;

    default:
      break;
    }
  }
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void) {
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for ANT stack initialization.
 */
void ant_stack_init(void) {

  ret_code_t err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  ASSERT(nrf_sdh_is_enabled());

  err_code = nrf_sdh_ant_enable();
  APP_ERROR_CHECK(err_code);

  err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUM);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for setting up the ANT module to be ready for RX broadcast.
 */
static void ant_channel_rx_broadcast_setup(void) {
  NRF_LOG_INFO("ant_channel_rx_broadcast_setup.");
  ant_channel_config_t broadcast_channel_config =
      {
          //.channel_number    = BROADCAST_CHANNEL_NUMBER,
          .channel_number = ANT_CHANNEL_NUM,
          .channel_type = CHANNEL_TYPE_SLAVE,
          .ext_assign = 0x00,
          .rf_freq = RF_FREQ,
          //.rf_freq           = 0x39,
          .transmission_type = CHAN_ID_TRANS_TYPE,
          .device_type = CHAN_ID_DEV_TYPE,
          .device_number = CHAN_ID_DEV_NUM,
          .channel_period = CHAN_PERIOD,
          .network_number = ANTPLUS_NETWORK_NUM};

  ret_code_t err_code = ant_channel_init(&broadcast_channel_config);
  APP_ERROR_CHECK(err_code);

  // Open channel.
  err_code = sd_ant_channel_open(ANT_CHANNEL_NUM);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event) {
  uint32_t err_code;
  switch (event) {
  case BSP_EVENT_SLEEP:

    break;

  case BSP_EVENT_DISCONNECT:

    break;

  case BSP_EVENT_WHITELIST_OFF:

    break;

  default:
    break;
  }
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handler(void *context, nrf_libuarte_async_evt_t *p_evt) {
  nrf_libuarte_async_t *p_libuarte = (nrf_libuarte_async_t *)context;
  ret_code_t ret;
  uint16_t index = 0;

  switch (p_evt->type) {
  case NRF_LIBUARTE_ASYNC_EVT_ERROR:
    break;
  case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
    while (index < p_evt->data.rxtx.length) {
    }
    nrf_libuarte_async_rx_free(p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
    break;

  case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:

    break;
  default:
    break;
  }
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void) {
  uint32_t err_code;

  nrf_libuarte_async_config_t nrf_libuarte_async_config = {
   //NRF9160DK
      .tx_pin = 17,
      .rx_pin = 20,
  // Thingy 91   
      //.tx_pin = 25,
      //.rx_pin = 32,
      .baudrate = NRF_UARTE_BAUDRATE_115200,
      .parity = NRF_UARTE_PARITY_EXCLUDED,
      .hwfc = NRF_UARTE_HWFC_DISABLED,
      .timeout_us = 100,
      .int_prio = APP_IRQ_PRIORITY_LOW};

  err_code = nrf_libuarte_async_init(&libuarte, &nrf_libuarte_async_config, uart_event_handler, (void *)&libuarte);

  APP_ERROR_CHECK(err_code);

  nrf_libuarte_async_enable(&libuarte);
}
/**@snippet [UART Initialization] */

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool *p_erase_bonds) {
  bsp_event_t startup_event;

  uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

/**@brief Application main function.
 */
int main(void) {
  bool erase_bonds;

  // Initialize.
  uart_init();
  log_init();
  timers_init();
  buttons_leds_init(&erase_bonds);
  power_management_init();
  ant_stack_init();
  profile_setup();
  //ant_channel_rx_broadcast_setup();

  sd_clock_hfclk_request();

  // Start execution.
  NRF_LOG_INFO("Start bpwr over uart:");
  printf("Test Messgage\n");

  for (;;) {
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
  }
}

/**
 * @}
 */