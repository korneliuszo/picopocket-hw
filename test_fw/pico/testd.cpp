#include "isa_worker.hpp"
#include "hardware/dma.h"
#include "pico/time.h"
#include <string.h>
#include "testd.hpp"
#include "psram_pio.hpp"
#include "ar1021.hpp"
#include "audio_dma.hpp"

[[gnu::aligned(4)]] volatile uint8_t monitor_data_window[2048];

extern "C" const uint8_t _binary_optionrom_bin_start[];
extern "C" const uint8_t _binary_optionrom_bin_size[];

extern "C" const uint8_t _binary_banner_bin_start[];
extern "C" const uint8_t _binary_banner_bin_end[];
extern "C" const uint8_t _binary_banner_bin_size[];

#include "S16LE2_44100.hpp"


static uint32_t read_fn(void* obj, uint32_t faddr)
{
	volatile uint8_t * arr = static_cast<volatile uint8_t*>(obj);
	uint8_t ret = arr[faddr];
	return ret;
}

static void nop_wrfn(void* obj, uint32_t faddr, uint8_t data) {}


union Regs {
struct [[gnu::packed]]  {
	uint8_t phase;
	uint8_t io_reg;
	uint16_t irr;
	uint16_t isr;
	uint16_t padto8;
} s;
struct [[gnu::packed]]  {
	uint8_t phase;
	uint8_t type;
	uint16_t ver;
} s2;
struct [[gnu::packed]]  {
	uint8_t phase;
	uint8_t X;
	uint8_t Y;
	uint8_t pen;
} s3;
uint8_t reg[8];
};

volatile Regs regs;

static uint32_t io_rdfn(void* obj, uint32_t faddr)
{
	return regs.reg[faddr];
}
static void io_wrfn(void* obj, uint32_t faddr, uint8_t data)
{
	regs.reg[faddr] = data;
}

static void wait_for_phase_passed(Thread * thread, uint8_t * phase_cntr)
{
	while (regs.s.phase <= *phase_cntr)
		thread->yield();
	*phase_cntr = regs.s.phase; //nonatomic!!!
}
static void start_new_phase(uint8_t * phase_cntr)
{
	*phase_cntr+=1;
	regs.s.phase = *phase_cntr;
}

volatile uint8_t rx_buff[25*80*2];
constexpr size_t rx_banner_size = 7*80*2;

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
    uint dma_chan = dma_claim_unused_channel(true);
    SetupSingleTransferTXDMA(dma_chan,(uint8_t*)_binary_banner_bin_start,(size_t)_binary_banner_bin_size);
	start_new_phase(&phase_cntr); // continue work on 386
	// DMA Transfer
	wait_for_phase_passed(thread,&phase_cntr); // DMA done
	regs.s.io_reg = TC_Triggered();
	start_new_phase(&phase_cntr); // continue work on 386
	wait_for_phase_passed(thread,&phase_cntr); // TC info displayed
    SetupSingleTransferRXDMA(dma_chan,rx_buff,rx_banner_size);
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
	regs.s.io_reg = !!(memcmp((uint8_t*)_binary_banner_bin_start,(uint8_t*)rx_buff,rx_banner_size));
	start_new_phase(&phase_cntr); // continue work on 386
	wait_for_phase_passed(thread,&phase_cntr);
	// IRQ configured

	uint8_t irq_handle = IRQ_Create_Handle(0);
	regs.s.io_reg = 0;

	uint8_t isrs[]={9,3,4,5,10,11,15};
	for(size_t i=0;i<(sizeof(isrs)/sizeof(*isrs));i++)
	{
		IRQ_Handle_Change_IRQ(irq_handle,isrs[i]);
		regs.s.irr = 0; //restore first arrived
		regs.s.isr = 0;
		IRQ_Set(irq_handle,true);
		uint32_t timeout = to_ms_since_boot(get_absolute_time()) + 500;
		while((to_ms_since_boot(get_absolute_time())<timeout) && !regs.s.io_reg)
			thread->yield();
		IRQ_Set(irq_handle,false);
		uint16_t irr = regs.s.irr; //latch first arrived
		uint16_t isr = regs.s.isr;
		regs.s.io_reg = 0;

		timeout = to_ms_since_boot(get_absolute_time()) + 500;
		while((to_ms_since_boot(get_absolute_time())<timeout))
		{ // eat up late
			thread->yield();
			regs.s.io_reg = 0;
		}

		regs.s.irr = irr; //restore first arrived
		regs.s.isr = isr;

		start_new_phase(&phase_cntr); // continue work on 386
		//READ IRQ;
		wait_for_phase_passed(thread,&phase_cntr);
	}

	AR1021::AR1021::init();

	PSRAM::PSRAM::init();
	volatile uint64_t psram_id = PSRAM::PSRAM::read_id_sync();

	regs.s.io_reg = psram_id>>(6*8);

	start_new_phase(&phase_cntr); // continue work on 386
	wait_for_phase_passed(thread,&phase_cntr); // PSRAM stat displayed

	{
    	const uint8_t * mem = (const uint8_t*)_binary_banner_bin_start;
		auto ptask = PSRAM::PSRAM::Write_Mem_Task(0,mem,sizeof(rx_buff));
		while(!ptask.complete_trigger())
			thread->yield();
	}
    memset((uint8_t*)rx_buff,1,sizeof(rx_buff));
    {
    	auto ptask = PSRAM::PSRAM::Read_Mem_Task(0,rx_buff,sizeof(rx_buff));
		while(!ptask.complete_trigger())
			thread->yield();
    }
	regs.s.io_reg = !(memcmp((uint8_t*)_binary_banner_bin_start,(uint8_t*)rx_buff,sizeof(rx_buff)));

	start_new_phase(&phase_cntr); // continue work on 386
	wait_for_phase_passed(thread,&phase_cntr); // PSRAM DATA OK
	AR1021::AR1021::disable_touch(thread);

	uint16_t version=0;
	uint8_t type=0;
	AR1021::AR1021::get_version(version,type,thread);
	regs.s2.ver = version;
	regs.s2.type = type;
	start_new_phase(&phase_cntr); // continue work on 386
	wait_for_phase_passed(thread,&phase_cntr); // TOUCH DISP

	AudioDMA::AudioDMA::init();
#if 0
	AudioDMA::AudioDMA::Single_playback::init_playback(44100,Jingle_ptr,Jingle_size);

	while(!AudioDMA::AudioDMA::Single_playback::is_complete())
		thread->yield();
#endif

	start_new_phase(&phase_cntr); // continue work on 386
	wait_for_phase_passed(thread,&phase_cntr); // AUDIO played

	regs.s3.X = 80/2;
	regs.s3.Y = 25/2;
	regs.s3.pen = 0;
	AR1021::AR1021::enable_touch(thread);

	start_new_phase(&phase_cntr); // prepared touchscreen

	while(1)
	{
		auto report = AR1021::AR1021::get_pos(thread);
		if(report.valid)
		{
			regs.s3.X = report.x*80/(1<<12);
			regs.s3.Y = report.y*25/(1<<12);
			regs.s3.pen = report.pen;
		}
	}
}

static StaticThread<1024> testd;

void testd_install(Thread * main)
{
	testd.run(main,testd_task);
}

uint8_t testd_get_phase()
{
	return regs.s.phase;
}
