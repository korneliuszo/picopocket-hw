/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <stdint.h>
#include "pico/types.h"
#include <atomic>

void ISA_Pre_Init();
void ISA_Init();
void ISA_TC_Init();

extern std::atomic<bool> wait_for_flash;

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
bool DMA_Complete(uint dma_chan);
//has been ..
bool TC_Triggered();


void DMA_RX_Setup();
void DMA_TX_Setup();
bool DMA_RX_is_ready();
uint8_t DMA_RX_get();
void DMA_TX_put(uint8_t val);

