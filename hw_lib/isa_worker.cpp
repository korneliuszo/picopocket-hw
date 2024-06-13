/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "hardware/structs/sio.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include <array>
#include <cstring>
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"

#include "isa_dma.pio.h"

#include <isa_worker.hpp>

constexpr uint isa_bus_sm = 0;            // Warning : HardCoded ISA State Machine number for speed

constexpr uint TC_PIN = 10;
constexpr uint DRQ_PIN = 19;
constexpr uint IA0_PIN = 26;

[[gnu::section(".core1")]] volatile uint8_t mem_map[1024*1024/2048]; //2kB resolution - bitfield
[[gnu::section(".core1")]] volatile uint8_t io_map[64*1024/8]; //8 IO regs resolution

volatile bool dma_single_transfer;

void ISA_Pre_Init()
{
	// ************ Pico I/O Pin Initialisation ****************
	// * Put the I/O in a correct state to avoid PC Crash      *

	gpio_init(DRQ_PIN);
	gpio_put(DRQ_PIN, 0);
	gpio_set_dir(DRQ_PIN,true);

	//IRQ
	for(uint pin=IA0_PIN;pin<IA0_PIN+3;pin++)
	{
		gpio_init(pin);
		gpio_put(pin, 0);
		gpio_set_dir(pin,true);
	}

	memset((void*)mem_map,0,sizeof(mem_map));
	memset((void*)io_map,0,sizeof(io_map));
}

volatile bool TC_triggered_val;

static void TC_trigger_IRQ(void)
{
    if (gpio_get_irq_event_mask(TC_PIN)) {
       gpio_acknowledge_irq(TC_PIN, GPIO_IRQ_EDGE_RISE);
       if(dma_single_transfer)
    	   gpio_put(DRQ_PIN, 0); // we assume transfer only ends from PC side
       TC_triggered_val = true;
    }
}

bool TC_Triggered()
{
	if(TC_triggered_val)
	{
		TC_triggered_val = false;
		return true;
	}
	return false;
}

void SetupSingleTransferTXDMA(uint dma_chan, const volatile uint8_t * buff, size_t len)
{
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, 1, true));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);

    dma_channel_configure(dma_chan, &c,
    	&pio0->txf[1],      // Destination pointer
		buff,               // Source pointer
        len,                // Number of transfers
        true                // Start immediately
    );

    dma_single_transfer = true;
	TC_triggered_val = false;
    gpio_put(DRQ_PIN, 1);
}

void SetupSingleTransferRXDMA(uint dma_chan, volatile uint8_t * buff, size_t len)
{
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, 1, false));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);

    dma_channel_configure(dma_chan, &c,
    	buff,               // Destination pointer
		&pio0->rxf[1],      // Source pointer
        len,                // Number of transfers
        true                // Start immediately
    );

    dma_single_transfer = true;
	TC_triggered_val = false;
    gpio_put(DRQ_PIN, 1);
}

void ISA_Init()
{
	//TC line
	gpio_init(TC_PIN);
	gpio_set_irq_enabled(TC_PIN, GPIO_IRQ_EDGE_RISE, true);
	gpio_add_raw_irq_handler_masked((1<<TC_PIN), TC_trigger_IRQ);
	irq_set_enabled(IO_IRQ_BANK0, true);

	initialize_isa_pio(pio0);
}

struct Device_int
{
	uint32_t mask = 0;
	uint32_t (*rdfn)(void*, uint32_t);
	void (*wrfn)(void*, uint32_t,uint8_t);
	void * obj;
};


constexpr uint32_t MEM_RD = (1<<(20+8+3-8));
constexpr uint32_t MEM_WR = (1<<(20+8+2-8));
constexpr uint32_t IO_RD = (1<<(20+8+1-8));
constexpr uint32_t IO_WR = (1<<(20+8+0-8));


std::array<Device_int,4> devices_mem = {};
std::array<Device_int,4> devices_io = {};


size_t used_devices_mem = 0;
size_t used_devices_io = 0;

bool add_device(const Device & device)
{
	switch(device.type)
	{
	case Device::Type::MEM:
		if(used_devices_mem == devices_mem.size())
			return false;
		{
			devices_mem[used_devices_mem] =
			{
					.mask = (device.size-1),
					.rdfn = device.rdfn,
					.wrfn = device.wrfn,
					.obj = device.obj,
			};
			for(uint32_t page=device.start;page<device.start+device.size;page+=2048)
			{
				uint32_t idx = page>>11;
				mem_map[idx]= 1+used_devices_mem;
			}
		}
		used_devices_mem++;
		break;
	case Device::Type::IO:
		if(used_devices_io == devices_io.size())
			return false;
		{
			devices_io[used_devices_io] =
			{
					.mask = (device.size-1),
					.rdfn = device.rdfn,
					.wrfn = device.wrfn,
					.obj = device.obj,
			};
			for(uint32_t page=device.start;page<device.start+device.size;page+=8)
			{
				uint32_t idx = page>>3;
				io_map[idx] = 1+used_devices_io;
			}
		}
		used_devices_io++;

		break;
	default:
		return false;
	}
	return true;
}


template <uint32_t OP, auto & device_tbl>
[[gnu::always_inline]] static inline void do_transfer(const uint32_t idx, const uint32_t &ISA_TRANS)
{
	PIO isa_pio = pio0;
	uint32_t FADDR = ISA_TRANS;
	auto && dev = device_tbl[idx-1];

	if(ISA_TRANS&OP)
	{ // READ
		uint32_t datard;
		datard = dev.rdfn(dev.obj,FADDR & (dev.mask));
		pio_sm_put(isa_pio,0,datard);
		return;
	}
	else
	{
		uint8_t data = gpio_get_all()>>PIN_AD0;
		dev.wrfn(dev.obj,FADDR & (dev.mask),data);
		return;
	}

}


static void __scratch_x("core1_code") [[gnu::noreturn]] main_core1(void)
{
	constexpr uint32_t Yreg = 1; //wr
	uint32_t TMP, TMP2, TMP3;
	uint32_t ISA_TRANS, ISA_IDX;
	uint32_t IORDY = (1u << (PIO_FSTAT_RXEMPTY_LSB + 0));
	PIO isa_pio = pio0;

	for (;;)
	{

		// ** Wait until a Control signal is detected **
		asm volatile goto (
				"ldr %[TMP2], =%[Yreg] \n\t"
				"strb    %[TMP2],[%[PIOB],%[PIOT]] \n\t" // pio->txf[0] = Yreg - turn on RDY
				"not_us_retry:\n\t"
				"1:\n\t"
				"ldr %[TMP], [%[PIOB],%[PIOF]]\n\t" // read fstat
				"tst %[TMP], %[IORDY]\n\t"			// test flag
				"bne 1b\n\t"						// loop if empty

				"ldr  %[ISA_TRANS],[%[PIOB],%[PIOR]] \n\t" // read ISA_TRANSACTION from PIO

				"lsr    %[TMP3], %[ISA_TRANS], #(32-8-2)\n\t" // TMP = ISA_TRANS >> (32-2)
				"cmp    %[TMP3],#0 \n\t"					 // TMP == 0
				"beq io\n\t" 							// memory_access

				"lsl    %[TMP], %[ISA_TRANS], #(4+8)\n\t" // TMP = ISA_TRANS << 4 // eat up flags
				"lsr    %[TMP], %[TMP], #(32-20+11)\n\t" // TMP = TMP >> (32-20+11) // move so we have page address
				"ldr %[TMP2], =%[MEMT] \n\t"
				"ldrb %[ISA_IDX],[%[TMP2],%[TMP]]\n\t"	// tmp = io_map[tmp]
				"strb    %[ISA_IDX],[%[PIOB],%[PIOT]] \n\t" // pio->txf[0] = pin status;
				"cmp %[ISA_IDX], #0\n\t"
				"beq not_us_retry\n\t"
				"strb    %[TMP3],[%[PIOB],%[PIOT]] \n\t" // pio->txf[0] = pin status;
				"b %l[memaction]\n\t" // jump to memaction

				"io:\n\t"
				"lsl    %[TMP], %[ISA_TRANS], #(4+4+8)\n\t" // TMP = ISA_TRANS << 4 // eat up flags and high address bits
				"lsr    %[TMP], %[TMP], #(32-16+3)\n\t" // TMP = TMP >> (32-16+3) // move so we have page address
				"ldr %[TMP2], =%[IOT] \n\t"
				"ldrb %[ISA_IDX],[%[TMP2],%[TMP]]\n\t"	// tmp = io_map[tmp]
				"strb    %[ISA_IDX],[%[PIOB],%[PIOT]] \n\t" // pio->txf[0] = pin status;
				"cmp %[ISA_IDX], #0\n\t"
				"beq not_us_retry \n\t"
				"lsl    %[TMP3], %[ISA_TRANS], #(2+8)\n\t" // TMP3 = ISA_TRANS << (2)
				"lsr    %[TMP3], %[TMP3], #(32-2)\n\t" // TMP3 = TMP3 >> (32-2)
				"strb    %[TMP3],[%[PIOB],%[PIOT]] \n\t" // pio->txf[0] = pin status;
				"b %l[ioaction]\n\t"							  // jump to memaction

				".ltorg\n\t"
				:[TMP3]"=l" (TMP3),
				 [ISA_TRANS]"=l" (ISA_TRANS),
				 [ISA_IDX]"=l" (ISA_IDX),
				 [TMP]"=l" (TMP),
				 [TMP2]"=l" (TMP2),
				 [IORDY]"+l" (IORDY),
				 [PIOB]"+l"(isa_pio)
				:[PIOT]"i" (PIO_TXF0_OFFSET),
				 [PIOR]"i" (PIO_RXF0_OFFSET),
				 [MEMT]"i" (mem_map),
				 [IOT]"i" (io_map),
				 [Yreg]"i" (Yreg),
				 [PIOF]"i"(PIO_FSTAT_OFFSET)
				:"cc"
				:memaction,
				 ioaction
				  );            // Tell the compiler that flags are changed

		continue;

		memaction:
		do_transfer<MEM_RD,devices_mem>(ISA_IDX,ISA_TRANS);
		continue;

		ioaction:
		do_transfer<IO_RD,devices_io>(ISA_IDX,ISA_TRANS);
		continue;
	}
}

void ISA_Start()
{
	multicore_launch_core1(main_core1);
}

struct IRQh {
	bool lit;
	uint8_t address;
};

static constexpr size_t IRQh_len = 10;

static IRQh irqhandlers[IRQh_len];
static size_t IRQh_used;

void IRQ_Handle_Change_IRQ(uint8_t irqh,uint8_t irq)
{
	switch(irq)
	{
	case 9:
		irqhandlers[irqh].address=1;
		break;
	case 3:
		irqhandlers[irqh].address=2;
		break;
	case 4:
		irqhandlers[irqh].address=3;
		break;
	case 5:
		irqhandlers[irqh].address=4;
		break;
	case 10:
		irqhandlers[irqh].address=5;
		break;
	case 11:
		irqhandlers[irqh].address=6;
		break;
	case 15:
		irqhandlers[irqh].address=7;
		break;
	default:
		irqhandlers[irqh].address=0;
		break;
	}
}


uint8_t IRQ_Create_Handle(uint8_t irq)
{
	if(IRQh_used >= IRQh_len)
		return 0xff;
	IRQ_Handle_Change_IRQ(IRQh_used,irq);
	return IRQh_used++;
}

void IRQ_Set(uint8_t irqh, bool val)
{
	if(irqh >= IRQh_used)
		return;
	irqhandlers[irqh].lit = val;
	uint8_t orirq = 0;
	for(size_t i=0;i<IRQh_used;i++) // first handler prioritized
	{
		if(irqhandlers[i].lit)
		{
			orirq = irqhandlers[i].address;
			break;
		}
	}

	gpio_put_masked(7<<IA0_PIN,orirq<<IA0_PIN);
}

