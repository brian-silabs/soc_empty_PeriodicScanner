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
#include "em_gpio.h"

#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"

#include "sl_sleeptimer.h"


#define FORCE_HALT_BETWEEN_SYNC_DATA  (1)

#define RADIO_WAKE_UP_GUARD_TIME_US   (3000)

#define WITH_RADIO_RETRY              (1)
#if(WITH_RADIO_RETRY)
  #define PERIODIC_ADV_PERIOD_US      (7500)
#else
  #define PERIODIC_ADV_PERIOD_US      (15000)
#endif

#define GPIO_TIMING_TRACES_ENABLE     (0)

// This constant is UUID of periodic synchronous service
const uint8_t periodicSyncService[16] = {0x81,0xc2,0x00,0x2d,0x31,0xf4,0xb0,0xbf,0x2b,0x42,0x49,0x68,0xc7,0x25,0x71,0x41};

static uint16_t sync;
bool haltStack = false;
sl_status_t sc;

#if(FORCE_HALT_BETWEEN_SYNC_DATA)
sl_status_t status;
sl_sleeptimer_timer_handle_t low_power_scan_timer;
uint32_t low_power_scan_timer_timeout_ticks = (((((PERIODIC_ADV_PERIOD_US * 1) - RADIO_WAKE_UP_GUARD_TIME_US)) * 32768) / 1000000);

void low_power_scan_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
#endif


// Parse advertisements looking for advertised periodicSync Service.
static uint8_t findServiceInAdvertisement(uint8_t *data, uint8_t len)
{
  uint8_t adFieldLength;
  uint8_t adFieldType;
  uint8_t i = 0;
  // Parse advertisement packet
  while (i < len) {
    adFieldLength = data[i];
    adFieldType = data[i + 1];
    // Partial ($02) or complete ($03) list of 128-bit UUIDs
    if (adFieldType == 0x06 || adFieldType == 0x07) {
      // compare UUID to service UUID
      if (memcmp(&data[i + 2], periodicSyncService, 16) == 0) {
        return 1;
      }
    }
    // advance to the next AD struct
    i = i + adFieldLength + 1;
  }
  return 0;
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

#if(GPIO_TIMING_TRACES_ENABLE)
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);//EXP #11 on MG22 radio board BRD4182
#endif
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

  if(haltStack)
  {
    status = sl_sleeptimer_start_timer(&low_power_scan_timer,
                                       low_power_scan_timer_timeout_ticks,
                                       low_power_scan_timer_callback,
                                       (void *)NULL,
                                       0,
                                       0);

    haltStack = false;
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
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

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

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Regular ADV ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

      sc = sl_bt_sync_set_parameters ( 1,//May skip 1
                                       0x64,//1000ms
                                       0);
      app_assert_status(sc);

      // periodic scanner setting
      sc = sl_bt_scanner_set_timing(gap_1m_phy, 200,200);
      app_assert_status(sc);

      sl_bt_scanner_set_mode(gap_1m_phy,0);
      app_assert_status(sc);

      sl_bt_scanner_start(gap_1m_phy,
                          scanner_discover_observation);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:

      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // scan response
    case sl_bt_evt_scanner_scan_report_id:
      /* only look at extended advertisements */
      if(evt->data.evt_scanner_scan_report.packet_type & 0x80){

        if (findServiceInAdvertisement(&(evt->data.evt_scanner_scan_report.data.data[0]), evt->data.evt_scanner_scan_report.data.len) != 0) {
          sc = sl_bt_sync_open(evt->data.evt_scanner_scan_report.address,
                               evt->data.evt_scanner_scan_report.address_type,
                               evt->data.evt_scanner_scan_report.adv_sid,
                               &sync);
        }
      }
      break;

    case sl_bt_evt_sync_opened_id:
      /* now that sync is open, we can stop scanning*/
      sl_bt_scanner_stop();
      break;

    case sl_bt_evt_sync_closed_id:
       /* restart discovery */
       sl_bt_scanner_start(gap_1m_phy,
                           scanner_discover_observation);
       break;

     case sl_bt_evt_sync_data_id:
       //Packets are received here
#if (FORCE_HALT_BETWEEN_SYNC_DATA)
//       haltStack = true;
       sc = sl_bt_system_halt(1);//TDOD should this be called outside evt handler ?
       app_assert_status(sc);
       status = sl_sleeptimer_start_timer(&low_power_scan_timer,
                                          low_power_scan_timer_timeout_ticks,
                                          low_power_scan_timer_callback,
                                          (void *)NULL,
                                          0,
                                          0);
#if(GPIO_TIMING_TRACES_ENABLE)
       GPIO_PinOutToggle(gpioPortD, 2);
#endif
       if(status != SL_STATUS_OK) {
           app_assert_status(status);
       }
#endif
       break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

#if (FORCE_HALT_BETWEEN_SYNC_DATA)
void low_power_scan_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  sc = sl_bt_system_halt(0);//Resume stack
  app_assert_status(sc);
#if(GPIO_TIMING_TRACES_ENABLE)
  GPIO_PinOutToggle(gpioPortD, 2);
#endif
}
#endif


