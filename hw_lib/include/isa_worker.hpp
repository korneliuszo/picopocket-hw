/*
 * isa_worker.hpp
 *
 *  Created on: Jan 6, 2024
 *      Author: kosa
 */
#pragma once

#include <stdint.h>
#include "pico/types.h"

void ISA_Pre_Init();
void ISA_Init();

constexpr uint32_t ISA_FAKE_READ = 1<<31;

struct Device{
	enum class Type
	{
		MEM,
		IO,
	};
	uint32_t start;
	uint32_t size; // must be power of 2
	Type type;
	uint32_t (*rdfn)(void*, uint32_t);
	void (*wrfn)(void*, uint32_t, uint8_t);
	void * obj;
};

bool add_device(const Device & device);
void ISA_Start();
uint8_t IRQ_Create_Handle(uint8_t irq);
void IRQ_Handle_Change_IRQ(uint8_t irqh,uint8_t irq);
void IRQ_Set(uint8_t irqh, bool val);

void SetupSingleTransferTXDMA(uint dma_chan, const volatile uint8_t * buff, size_t len);
void SetupSingleTransferRXDMA(uint dma_chan, volatile uint8_t * buff, size_t len);
//has been ..
bool TC_Triggered();

