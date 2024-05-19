#include "isa_worker.hpp"
#include "hardware/dma.h"
#include <string.h>
#include "testd.hpp"

[[gnu::aligned(4)]] volatile uint8_t monitor_data_window[2048];

extern "C" const uint8_t _binary_optionrom_bin_start[];
extern "C" const uint8_t _binary_optionrom_bin_size[];

extern "C" const uint8_t _binary_banner_bin_start[];
extern "C" const uint8_t _binary_banner_bin_size[];


static uint32_t read_fn(void* obj, uint32_t faddr)
{
	volatile uint8_t * arr = static_cast<volatile uint8_t*>(obj);
	uint8_t ret = arr[faddr];
	return ret;
}

static void nop_wrfn(void* obj, uint32_t faddr, uint8_t data) {}


volatile uint8_t phase = 0x00;
volatile uint8_t io_reg = 0x00;


static uint32_t io_rdfn(void* obj, uint32_t faddr)
{
	switch(faddr)
	{
	case 0:
	{
		return phase;
	}
	case 1:
	{
		return io_reg;
	}
	default:
		return 0xff;
	}
}
static void io_wrfn(void* obj, uint32_t faddr, uint8_t data)
{
	switch(faddr)
	{
	case 0:
	{
		phase = data;
		return;
	}
	case 1:
	{
		io_reg = data;
		return;
	}
	default:
		return;
	}
}

static void wait_for_phase_passed(Thread * thread, uint8_t * phase_cntr)
{
	while (phase <= *phase_cntr)
		thread->yield();
	*phase_cntr = phase; //nonatomic!!!
}
static void start_new_phase(uint8_t * phase_cntr)
{
	*phase_cntr+=1;
	phase = *phase_cntr;
}

volatile uint8_t rx_buff[7*80*2];

static void testd_task(Thread * thread)
{
	add_device({
					.start = 0xDC000,
					.size = (uint32_t)_binary_optionrom_bin_size,
					.type = Device::Type::MEM,
					.rdfn = read_fn,
					.wrfn = nop_wrfn,
					.obj = (void*)_binary_optionrom_bin_start,
	});
	add_device({
					.start = 0x78,
					.size = (uint32_t)8,
					.type = Device::Type::IO,
					.rdfn = io_rdfn,
					.wrfn = io_wrfn,
					.obj = nullptr,
	});

	uint8_t phase_cntr = 0;

	wait_for_phase_passed(thread,&phase_cntr); // 01 boot passed
	start_new_phase(&phase_cntr); // continue work on 386
	wait_for_phase_passed(thread,&phase_cntr); // DMA TEST

	//prepare write DMA
    uint dma_chan = 0;
    SetupSingleTransferTXDMA(0,(uint8_t*)_binary_banner_bin_start,(size_t)_binary_banner_bin_size);
	start_new_phase(&phase_cntr); // continue work on 386
	// DMA Transfer
	wait_for_phase_passed(thread,&phase_cntr); // DMA done
	io_reg = TC_Triggered();
	start_new_phase(&phase_cntr); // continue work on 386
	wait_for_phase_passed(thread,&phase_cntr); // TC info displayed
    SetupSingleTransferRXDMA(0,rx_buff,sizeof(rx_buff));
	start_new_phase(&phase_cntr); // continue work on 386
	wait_for_phase_passed(thread,&phase_cntr); // DMA started
	while(!TC_Triggered())
	{
		thread->yield();
	}
	while(dma_channel_is_busy(0))
	{
		thread->yield();
	}
	io_reg = !!(memcmp((uint8_t*)_binary_banner_bin_start,(uint8_t*)rx_buff,sizeof(rx_buff)));
	start_new_phase(&phase_cntr); // continue work on 386


	while(1)
		thread->yield();

}

static StaticThread<1024> testd;

void testd_install(Thread * main)
{
	testd.run(main,testd_task);
}

uint8_t testd_get_phase()
{
	return phase;
}
