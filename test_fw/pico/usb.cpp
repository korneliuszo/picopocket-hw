/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#include "tusb.h"
#include "pico/bootrom.h"
#include "device/usbd_pvt.h"
#include "hardware/gpio.h"

#include "testd.hpp"

static uint8_t itf_num;

static void testd_init(void) {
}

static void testd_reset(uint8_t rhport) {
	itf_num = 0;
}

static size_t monitor_data_ptr;

static uint16_t testd_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
	TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass &&
			0 == itf_desc->bInterfaceSubClass &&
			0 == itf_desc->bInterfaceProtocol, 0);

	uint16_t drv_len = sizeof(tusb_desc_interface_t);
	TU_VERIFY(max_len >= drv_len, 0);

	itf_num = itf_desc->bInterfaceNumber;

	return drv_len;
}

// Support for parameterized reset via vendor interface control request
static bool testd_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
	// nothing to do with DATA & ACK stage
	if (stage != CONTROL_STAGE_SETUP) return true;

	if (request->wIndex == itf_num) {
		switch(request->bRequest)
		{
		case 0:
			reset_usb_boot(0, 1);
			break;
			// does not return, otherwise we'd return true
		case 1:
		{
			uint8_t phase = testd_get_phase();
			return tud_control_xfer(rhport, request, &phase,1);
		}
		default:
			return false;
		}
	}
	return false;
}

[[gnu::section(".tusb_driver_class"), gnu::used]]
static usbd_class_driver_t const _testd_driver =
{
#if CFG_TUSB_DEBUG >= 2
		.name = "MONITOR",
#endif
		.init             = testd_init,
		.reset            = testd_reset,
		.open             = testd_open,
		.control_xfer_cb  = testd_control_xfer_cb,
		.xfer_cb          = NULL,
		.sof              = NULL
};
