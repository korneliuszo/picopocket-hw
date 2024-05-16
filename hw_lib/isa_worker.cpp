/*
 * isa_worker.cpp
 *
 *  Created on: Jan 6, 2024
 *      Author: kosa
 */

#include "hardware/structs/sio.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include <array>
#include <cstring>
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"

#include "isa_dma.pio.h"

#include <isa_worker.hpp>

constexpr uint isa_bus_sm = 0;            // Warning : HardCoded ISA State Machine number for speed

[[gnu::section(".core1")]] volatile uint8_t mem_map[1024*1024/2048]; //2kB resolution - bitfield
[[gnu::section(".core1")]] volatile uint8_t io_map[64*1024/8]; //8 IO regs resolution

void ISA_Pre_Init()
{
	// ************ Pico I/O Pin Initialisation ****************
	// * Put the I/O in a correct state to avoid PC Crash      *

	//TODO: IRQ

	memset((void*)mem_map,0,sizeof(mem_map));
	memset((void*)io_map,0,sizeof(io_map));
}

void ISA_Init()
{

	initialize_isa_pio(pio0);
}

struct Device_int
{
	uint32_t mask = 0;
	uint32_t (*rdfn)(void*, uint32_t);
	void (*wrfn)(void*, uint32_t,uint8_t);
	void * obj;
};


constexpr uint32_t MEM_RD = (1<<(20+8+3));
constexpr uint32_t MEM_WR = (1<<(20+8+2));
constexpr uint32_t IO_RD = (1<<(20+8+1));
constexpr uint32_t IO_WR = (1<<(20+8+0));


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
	uint32_t FADDR = ISA_TRANS>>8;
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
		dev.wrfn(dev.obj,FADDR & (dev.mask),ISA_TRANS);
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
				"str    %[TMP2],[%[PIOB],%[PIOT]] \n\t" // pio->txf[0] = Yreg - turn on RDY
				"not_us_retry:\n\t"
				"1:\n\t"
				"ldr %[TMP], [%[PIOB],%[PIOF]]\n\t" // read fstat
				"tst %[TMP], %[IORDY]\n\t"			// test flag
				"bne 1b\n\t"						// loop if empty

				"ldr  %[ISA_TRANS],[%[PIOB],%[PIOR]] \n\t" // read ISA_TRANSACTION from PIO

				"lsr    %[TMP3], %[ISA_TRANS], #(32-2)\n\t" // TMP = ISA_TRANS >> (32-2)
				"cmp    %[TMP3],#0 \n\t"					 // TMP == 0
				"beq io\n\t" 							// memory_access

				"lsl    %[TMP], %[ISA_TRANS], #4\n\t" // TMP = ISA_TRANS << 4 // eat up flags
				"lsr    %[TMP], %[TMP], #(32-20+11)\n\t" // TMP = TMP >> (32-20+11) // move so we have page address
				"ldr %[TMP2], =%[MEMT] \n\t"
				"ldrb %[ISA_IDX],[%[TMP2],%[TMP]]\n\t"	// tmp = io_map[tmp]
				"cmp %[ISA_IDX], #0\n\t"
				"beq not_us\n\t"
				"strb    %[TMP3],[%[PIOB],%[PIOT]] \n\t" // pio->txf[0] = pin status;
				"b %l[memaction]\n\t" // jump to memaction

				"io:\n\t"
				"lsl    %[TMP], %[ISA_TRANS], #(4+4)\n\t" // TMP = ISA_TRANS << 4 // eat up flags and high address bits
				"lsr    %[TMP], %[TMP], #(32-16+3)\n\t" // TMP = TMP >> (32-16+3) // move so we have page address
				"ldr %[TMP2], =%[IOT] \n\t"
				"ldrb %[ISA_IDX],[%[TMP2],%[TMP]]\n\t"	// tmp = io_map[tmp]
				"cmp %[ISA_IDX], #0\n\t"
				"beq not_us \n\t"
				"lsl    %[TMP3], %[ISA_TRANS], #2\n\t" // TMP3 = ISA_TRANS << (2)
				"lsr    %[TMP3], %[TMP3], #(32-2)\n\t" // TMP3 = TMP3 >> (32-2)
				"strb    %[TMP3],[%[PIOB],%[PIOT]] \n\t" // pio->txf[0] = pin status;
				"b %l[ioaction]\n\t"							  // jump to memaction

				"not_us:\n\t" //ISA_IDX is 0
				"strb    %[ISA_IDX],[%[PIOB],%[PIOT]] \n\t" // pio->txf[0] = pin status;
				"b not_us_retry\n\t"						// if so jump to noaction

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
};

static constexpr size_t IRQh_len = 2;

static IRQh irqhandlers[IRQh_len];
static size_t IRQh_used;

uint8_t IRQ_Create_Handle(uint8_t irq) //we have one irq on this board
{
	if(IRQh_used >= IRQh_len)
		return 0xff;
	return IRQh_used++;
}

void IRQ_Set(uint8_t irqh, bool val)
{
	if(irqh >IRQh_used)
		return;
	irqhandlers[irqh].lit = val;
	bool orirq = false;
	for(size_t i=0;i<IRQh_used;i++)
		if(irqhandlers[i].lit)
			orirq = true;

	//gpio_put(PIN_IRQ, orirq);  // Set IRQ Up (There is an inverter) for no IRQ

}
