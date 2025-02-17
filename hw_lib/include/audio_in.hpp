/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "pico.h"
#include "pico/types.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/structs/systick.h"
#include <atomic>
#include <stdint.h>
#include <algorithm>

namespace AudioIn {

#include "apdm.pio.h"

namespace PIO_Selector{

	template<uint>
	inline volatile PIO PIOPort();
	template<>
	inline volatile PIO PIOPort<0>() {
        return reinterpret_cast<PIO>(PIO0_BASE);
    }
	template<>
	inline volatile PIO PIOPort<1>() {
        return reinterpret_cast<PIO>(PIO1_BASE);
    }
};

template <uint PIN_PDM, uint PION, uint IRQN>
class AudioIn_Impl {

	static inline uint pio_sm;

	AudioIn_Impl() = delete;

	static inline int tick_dma_chan;
	static inline uint program_offset;

	static void init()
	{
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
		pio_sm = pio_claim_unused_sm(pio,true);
		program_offset = pio_add_program(pio, &apdm_input_program);
		apdm_input_program_init(
				pio,
				pio_sm,
				program_offset,
				PIN_PDM);
	    tick_dma_chan = dma_claim_unused_channel(true);
	    pio_sm_set_enabled(pio,pio_sm,1);
	    pio_gpio_init(pio, PIN_PDM);

	}

	static void deinit()
	{
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
		pio_sm_set_enabled(pio,pio_sm,0);
		pio_sm_unclaim(pio,pio_sm);
		dma_channel_unclaim(tick_dma_chan);
		pio_remove_program(pio,&apdm_input_program,program_offset);
	}
public:

	class Read_by_call{
		Read_by_call() = delete;
		static inline volatile uint32_t last_kicked;
	public:
		static void start()
		{
			volatile PIO pio = PIO_Selector::PIOPort<PION>();
			init();
			pio_sm_put(pio,pio_sm,0);
			last_kicked = time_us_32();
		}
		static void stop()
		{
			volatile PIO pio = PIO_Selector::PIOPort<PION>();
			pio_sm_set_enabled(pio,pio_sm,0);
			pio_sm_clear_fifos(pio,pio_sm);
			deinit();
		}
		static int16_t __not_in_flash_func(read)()
		{
			volatile PIO pio = PIO_Selector::PIOPort<PION>();
			if(pio_sm_is_rx_fifo_empty(pio,pio_sm))
				return 0x00; // probably nothing connected
			uint32_t duration = time_us_32() - last_kicked;
			pio_sm_put(pio,pio_sm,0);
			last_kicked = time_us_32();
			int32_t val = pio_sm_get(pio,pio_sm);
			//TODO: unhardcode speed
			return val * (250 / 2) / duration;
		}
	};

	template <void (*callback)(int16_t sample)>
	class Read_by_timer{
		Read_by_timer() = delete;
		static inline const uint32_t zeroval = 0;
		static inline int dma_timer;
		static inline uint samplerate;
		static inline uint pio_irq;
	public:
		static void start(uint sample_rate)
		{
			volatile PIO pio = PIO_Selector::PIOPort<PION>();
			init();
			samplerate = sample_rate;
			dma_channel_config cfg_dma_chan_config = dma_channel_get_default_config(tick_dma_chan);
		    channel_config_set_transfer_data_size(&cfg_dma_chan_config, DMA_SIZE_32);
		    channel_config_set_read_increment(&cfg_dma_chan_config, false);
		    channel_config_set_write_increment(&cfg_dma_chan_config, false);

			dma_timer = dma_claim_unused_timer(true /* required */);
			dma_timer_set_fraction(dma_timer, sample_rate/4000, clock_get_hz(clk_sys)/4000);  // divide system clock by num/denom
			int treq = dma_get_timer_dreq(dma_timer);
			channel_config_set_dreq(&cfg_dma_chan_config, treq);
			dma_channel_set_config(tick_dma_chan, &cfg_dma_chan_config, false);
		    dma_channel_set_write_addr(tick_dma_chan, &pio->txf[pio_sm], false);
		    dma_channel_set_read_addr(tick_dma_chan, &zeroval, false);
		    dma_channel_set_trans_count(tick_dma_chan, 0xFFFFFFFF, true); //TODO: reinit?

		    pio_irq = PIO0_IRQ_0 + 2*PION + IRQN; //hacky
		    irq_add_shared_handler(pio_irq, isr, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY); // Add a shared IRQ handler
		    irq_set_enabled(pio_irq, true); // Enable the IRQ
		    pio_set_irqn_source_enabled(pio, IRQN,
		    		(pio_interrupt_source)((uint)pis_sm0_rx_fifo_not_empty+pio_sm), true); // Set pio to tell us when the FIFO is NOT empty
		}
		static void stop()
		{
			volatile PIO pio = PIO_Selector::PIOPort<PION>();
			dma_channel_abort(tick_dma_chan);
			dma_timer_unclaim(dma_timer);
			pio_sm_set_enabled(pio,pio_sm,0);
			pio_sm_clear_fifos(pio,pio_sm);
		    irq_set_enabled(pio_irq, false);
		    irq_remove_handler(pio_irq,isr);
			deinit();
		}
		static void __not_in_flash_func(isr)()
		{
			volatile PIO pio = PIO_Selector::PIOPort<PION>();
			int32_t val = pio_sm_get(pio,pio_sm);
			//TODO: unhardcode speed
			callback(val * (samplerate/(25000000>>16)/4));
		}
	};
};

#if defined(AUDIO_PDM)
using AudioIn = AudioIn_Impl<AUDIO_PDM,AUDIO_PIO, AUDIO_PIO_IRQN>;
#endif

};
