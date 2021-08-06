/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*
* RTC Sample
*
* This application provides the sample code for interfacing with
* the on-chip RTC clock.
*
*
* Features demonstrated
*  - Use of the on-chip RTC interface.
*
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Application initializes the RTC and prints time every second.
*
*/

#include "spar_utils.h"
#include "bleprofile.h"
#include "bleapp.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "bleappconfig.h"
#include "cfa.h"
#include "rtc.h"
#include "bleapputils.h"
#include "bleapp.h"
#include "devicelpm.h"
#include "miadriver.h"
#include "sparcommon.h"
#include "hidddriversconfig.h"

extern void rtc_setReferenceTime(RtcTime* ref_time);
/******************************************************
 *                      Constants
 ******************************************************/

/******************************************************
 *               Function Prototypes
 ******************************************************/
static void rtc_sample_create(void);
static void rtc_sample_timeout(UINT32 arg);
static void rtc_sample_fine_timeout(UINT32 arg);

/******************************************************
 *               Variables Definitions
 ******************************************************/

// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG rtc_sample_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ PUARTENABLE | GPIO_PIN_UART_TX,
    /*.rxpin      =*/ PUARTENABLE | GPIO_PIN_UART_RX,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

// Application initialization
APPLICATION_INIT()
{
    bleapp_set_cfg(NULL, 0, NULL, (void *)&rtc_sample_puart_cfg, NULL, rtc_sample_create);

    // BLE_APP_DISABLE_TRACING();     ////// Uncomment to disable all tracing
    BLE_APP_ENABLE_TRACING_ON_PUART();
}

// Create the RTC sample.
void rtc_sample_create(void)
{
    RtcTime current_time;
	char buffer[64];

    ble_trace0("rtc_sample_create()\n");

    if (!mia_isResetReasonPor())
    {
		ble_trace0("Waking from deep sleep because the timer went off or a GPIO triggered while waiting for timer to expire.");
    }
    else
    {
		ble_trace0("Not a timed wake.");
    }

    blecm_configFlag |= BLECM_DBGUART_LOG;

    bleprofile_Init(bleprofile_p_cfg);

    // Initialize the RTC.
    rtc_init();

    memset(buffer, 0x00, sizeof(buffer));
	ble_trace0("Time base is:");

	// RtcTime of 0x00 is the start of RTC time.
    memset(&current_time, 0x00, sizeof(current_time));
	rtc_ctime(&current_time, buffer);
	ble_trace0(buffer);

	// Let year = 2021.
	current_time.year = 2021;
	// Let month = july = 6 (jan = 0)
	current_time.month = 6;
	// Let day = 21st.
	current_time.day = 21;
	// Let current time be 12:00:00 Noon.
	current_time.hour = 12;
	current_time.minute = 00;
	current_time.second = 0x00;

	// If this is a power-on reset, we need to set up the reference time.
	if (mia_isResetReasonPor())
	{
		// Now set the on-chip RTC.
		if(rtc_setRTCTime(&current_time))
		{
			memset(buffer, 0x00, sizeof(buffer));

			ble_trace0("Power-on reset, set current time to:");
			rtc_ctime(&current_time, buffer);
			ble_trace0(buffer);
		}
		else
		{
			ble_trace0("Unable to set time.");
		}
	}
	else
	{
		// Set up the original reference time instead of using 01/01/2010, 00:00:00 as the reference
		// because this is a wake from deep sleep. The HW clock keeps running in deep sleep so when
		// we wake up, the FW needs to know what was used as the original reference time.
		rtc_setReferenceTime(&current_time);
	}

    bleprofile_regTimerCb(rtc_sample_fine_timeout, rtc_sample_timeout);
    bleprofile_StartTimer();

    // Trace out number of bytes free.
    ble_trace1("Number of free bytes in RAM: %d",  cfa_mm_MemFreeBytes());
}

// One second timer expired. Read the time from RTC and print it.
void rtc_sample_timeout(UINT32 arg)
{
	ble_trace0("rtc sample timeout\n");
	RtcTime current_time;
	UINT32 seconds_since_time_base;
	char buffer[64];
	tRTC_REAL_TIME_CLOCK raw_clock;

	memset(buffer, 0x00, sizeof(buffer));

	// Get and print current time.
	rtc_getRTCTime(&current_time);
	rtc_ctime(&current_time, buffer);
	ble_trace0("Current time is:\n");
	ble_trace0(buffer);
	ble_trace0("\n");

	// Get and print time since time base in seconds.
	rtc_RtcTime2Sec(&current_time, &seconds_since_time_base);
	ble_trace1("Its been %d seconds since bigbang.\n", seconds_since_time_base);

	// Get and print the raw 48 bit clock value.
	rtc_getRTCRawClock(&raw_clock);

	ble_trace2("Upper and lower 32 bit values: 0x%08X, 0x%08X\n", raw_clock.reg32map.rtc32[1], raw_clock.reg32map.rtc32[0]);

}

void rtc_sample_fine_timeout(UINT32 arg)
{
	ble_trace0("rtc sample fine timeout\n");
}
