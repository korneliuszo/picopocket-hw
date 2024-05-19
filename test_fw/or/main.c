#include <i86.h>
#include <stdint.h>
#include <string.h>
#include "biosint.h"
#include "inlines.h"

void wait(uint32_t ms);
#pragma aux wait = "mov ax, 8600h" "int 15h" parm [cx dx] modify exact [ax];


typedef struct {
	uint16_t es;
	uint16_t ds;
	uint16_t di;
	uint16_t si;
	uint16_t bp;
	uint16_t bx;
	uint16_t dx;
	uint16_t cx;
	uint16_t ax;
	uint16_t f;
	uint16_t ph1;
	uint16_t ph2;
	uint16_t ip;
	uint16_t cs;
	uint16_t rf;
} IRQ_DATA;

int __cdecl start(uint16_t irq, IRQ_DATA far *params);

static inline void phase_passed(uint8_t far * phase)
{
	*phase+=1;
	bios_printf(BIOS_PRINTF_ALL, "%02x",*phase);
	outb(0x78,*phase);
	uint8_t recv;
	while ((recv=inb(0x78)) <= *phase);
	*phase=recv;
	bios_printf(BIOS_PRINTF_ALL, "%02x",*phase);
	return;
}

int start(uint16_t irq, IRQ_DATA far *params) {
	if (irq == 0) {
		bios_printf(BIOS_PRINTF_ALL, "Picopocket tester\n");
		uint8_t phase = inb(0x78);
		if(phase != 0)
		{
			bios_printf(BIOS_PRINTF_ALL, "BUG: Received %02x\n",phase);
			goto error;
		}
		phase_passed(&phase); //BOOT passed
		//DMA TEST
		phase_passed(&phase); //wait for pico to prepare DMA

		outb(0x0a,0x05); // mask channels 1
		outb(0x0c, 0xFF); // reset the masked channel
		outb(0x0b, /*0b01000101*/ 0x45); // setup single transter, up, noauto, read, channel 1
		//0x0b8000
		outb(0x83,0x0b); //high address
		outb(0x02,0x00); // low address
		outb(0x02,0x80); // middle address
		// 0x0F9F - 4000 - 1
		outb(0x03,0x9F); // low count
		outb(0x03,0x0F); // high count

		outb(0x0A, 1); // enable channel 1

		wait(2000000); //2seconds

		phase_passed(&phase); //DMA done
		uint8_t io_reg = inb(0x79);
		printf("\nTC_triggered = %d\n",io_reg);
		phase_passed(&phase); // TC info displayed

		outb(0x0a,0x05); // mask channels 1
		outb(0x0c, 0xFF); // reset the masked channel
		outb(0x0b, /*0b01001001*/ 0x49); // setup single transter, up, noauto, write, channel 1
		//0x0b8000
		outb(0x83,0x0b); //high address
		outb(0x02,0x00); // low address
		outb(0x02,0x80); // middle address
		// 0x045f - 7*80*2-1
		outb(0x03,0x5F); // low count
		outb(0x03,0x04); // high count

		outb(0x0A, 1); // enable channel 1
		phase_passed(&phase); // DMA started
		io_reg = inb(0x79);
		printf("\nDMA Data OK = %d\n",io_reg);
		while(1)
			phase_passed(&phase); //pico phases
	}
	error:
	while(1);
}
