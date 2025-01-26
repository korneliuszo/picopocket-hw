/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
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

#include "isa_worker.hpp"
#include "fifo.hpp"
#include "class/vendor/vendor_device.h"

constexpr uint32_t PICO_Freq=250; //PM_SYS_CLK;

Fifo<uint8_t,4096> datastream;

static uint32_t mem_rdfn(void* obj, uint32_t faddr)
{
	while(datastream.fifo_free() < 4);
	uint8_t data = gpio_get_all()>>0;
	auto buff = datastream.get_wrbuff();
	buff[0] = 0x10 | (faddr >> 16);
	buff[1] = faddr >> 8;
	buff[2] = faddr >> 0;
	buff[3] = data;
	datastream.commit_wrbuff(4);
	return ISA_FAKE_READ;

}
static void mem_wrfn(void* obj, uint32_t faddr, uint8_t data)
{
	while(datastream.fifo_free() < 4);
	auto buff = datastream.get_wrbuff();
	buff[0] = 0x20 | (faddr >> 16);
	buff[1] = faddr >> 8;
	buff[2] = faddr >> 0;
	buff[3] = data;
	datastream.commit_wrbuff(4);
}

static uint32_t io_rdfn(void* obj, uint32_t faddr)
{
	while(datastream.fifo_free() < 4);
	uint8_t data = gpio_get_all()>>0;
	auto buff = datastream.get_wrbuff();
	buff[0] = 0x40 | (faddr >> 16);
	buff[1] = faddr >> 8;
	buff[2] = faddr >> 0;
	buff[3] = data;
	datastream.commit_wrbuff(4);
	return ISA_FAKE_READ;
}
static void io_wrfn(void* obj, uint32_t faddr, uint8_t data)
{
	while(datastream.fifo_free() < 4);
	auto buff = datastream.get_wrbuff();
	buff[0] = 0x80 | (faddr >> 16);
	buff[1] = faddr >> 8;
	buff[2] = faddr >> 0;
	buff[3] = data;
	datastream.commit_wrbuff(4);
}

int main(void)
{
#ifndef PICOPOCKET_SIM
	// Overclock!
	vreg_set_voltage(VREG_VOLTAGE_1_25);
	sleep_ms(100);
	set_sys_clock_khz(PICO_Freq*1000, true);
#endif
	ISA_Pre_Init();

	add_device({
					.start = 0x00000,
					.size = (uint32_t)(size_t)0x100000,
					.type = Device::Type::MEM,
					.rdfn = mem_rdfn,
					.wrfn = mem_wrfn,
					.obj = nullptr,
	});
	add_device({
					.start = 0x00000,
					.size = (uint32_t)0x10000,
					.type = Device::Type::IO,
					.rdfn = io_rdfn,
					.wrfn = io_wrfn,
					.obj = nullptr,
	});

	tusb_init();

	ISA_Init();

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;

    ISA_Start();

    while(1)
    {
		tud_task(); // tinyusb device task

		uint len;
		uint ulen;
		while(tud_vendor_n_mounted(0)
				&& (len=datastream.fifo_check())
				&& (ulen=tud_vendor_n_write_available(0))>4)
		{
			len = MIN(len,ulen);
			len = len &~0x3; //full frames
			auto buff = datastream.get_rdbuff();
			buff.set_len(len);
			tud_vendor_n_write(0
					, const_cast<uint8_t*>(buff.start)
					, buff.stop-buff.start);
			if(buff.stop2-buff.start2)
				tud_vendor_n_write(0
						, const_cast<uint8_t*>(buff.start2)
						, buff.stop2-buff.start2);
			datastream.commit_rdbuff(len);
		}

    }
}

