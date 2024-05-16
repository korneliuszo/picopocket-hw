#include "isa_worker.hpp"
#include <string.h>
#include "testd.hpp"

[[gnu::aligned(4)]] volatile uint8_t monitor_data_window[2048];

extern "C" const uint8_t _binary_optionrom_bin_start[];
extern "C" const uint8_t _binary_optionrom_bin_size[];

static uint32_t read_fn(void* obj, uint32_t faddr)
{
	volatile uint8_t * arr = static_cast<volatile uint8_t*>(obj);
	uint8_t ret = arr[faddr];
	return ret;
}

static void nop_wrfn(void* obj, uint32_t faddr, uint8_t data) {}


volatile uint8_t phase = 0x00;

static uint32_t io_rdfn(void* obj, uint32_t faddr)
{
	switch(faddr)
	{
	case 0:
	{
		return phase;
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

	wait_for_phase_passed(thread,&phase_cntr);
	start_new_phase(&phase_cntr);

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
