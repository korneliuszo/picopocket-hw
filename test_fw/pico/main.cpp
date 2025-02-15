/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#ifndef PICOPOCKET_SIM
#include "pico/stdlib.h"
#include "hardware/vreg.h"
#include "hardware/sync.h"
#include "hardware/regs/vreg_and_chip_reset.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/regs/busctrl.h"              /* Bus Priority defines */

#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#endif

#include "isa_worker.hpp"
#include "jmpcoro.hpp"
#include "testd.hpp"

constexpr uint32_t PICO_Freq=250; //PM_SYS_CLK;


int main(void)
{
#ifndef PICOPOCKET_SIM
	// Overclock!
	vreg_set_voltage(VREG_VOLTAGE_1_25);
	sleep_ms(100);
	set_sys_clock_khz(PICO_Freq*1000, true);
#endif
	ISA_Pre_Init();

	Thread main_thread;

	testd_install(&main_thread);

#ifndef PICOPOCKET_SIM
	tusb_init();
#endif
	ISA_TC_Init();
	ISA_Init();

#ifndef PICOPOCKET_SIM
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;
#endif

    ISA_Start();

    while(1)
    {
#ifndef PICOPOCKET_SIM
		tud_task(); // tinyusb device task
#endif
		main_thread.yield();
    }
}

#ifndef PICOPOCKET_SIM
// Implement callback to add our custom driver
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {

    extern usbd_class_driver_t __tusb_driver_class_start;
    extern usbd_class_driver_t __tusb_driver_class_end;

	*driver_count = &__tusb_driver_class_end - &__tusb_driver_class_start;
	return &__tusb_driver_class_start;
}
#endif
