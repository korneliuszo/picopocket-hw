#include <i86.h>
#include <stdint.h>
#include <string.h>
#include "biosint.h"
#include "inlines.h"

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
		uint8_t phase = inb(0x78);
		bios_printf(BIOS_PRINTF_ALL, "Picopocket tester\n");
		phase_passed(&phase);

		phase_passed(&phase);
	}
	while(1);
}
