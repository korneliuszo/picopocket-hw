/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "pico/types.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include <atomic>
#include <stdint.h>

namespace AudioDMA {

#include "audio_i2s.pio.h"

namespace PIO_Selector{

	template<uint>
	volatile PIO PIOPort();
	template<>
    volatile PIO PIOPort<0>() {
        return reinterpret_cast<PIO>(PIO0_BASE);
    }
	template<>
    volatile PIO PIOPort<1>() {
        return reinterpret_cast<PIO>(PIO1_BASE);
    }
};

template <uint PIN_DIN,uint PIN_BCK,uint PIN_LRCK, uint PION, uint IRQN>
class AudioDMA_Impl {
	static_assert(PIN_BCK + 1 == PIN_LRCK);

	static inline uint pio_sm;


	AudioDMA_Impl() = delete;
public:

	static constexpr size_t DMA_BYTE_LEN = 64;

	static inline int ping_dma_chan;
	static inline int pong_dma_chan;

	static void init()
	{
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
		pio_sm = pio_claim_unused_sm(pio,true);
		audio_i2s_program_init(
				pio,
				pio_sm,
				pio_add_program(pio, &audio_i2s_program),
				PIN_DIN,
				PIN_BCK);
	    ping_dma_chan = dma_claim_unused_channel(true);
	    pong_dma_chan = dma_claim_unused_channel(true);
	    pio_sm_set_enabled(pio,pio_sm,1);
	    pio_gpio_init(pio, PIN_DIN);
	    pio_gpio_init(pio, PIN_BCK);
	    pio_gpio_init(pio, PIN_LRCK);

	}

	static void update_pio_frequency(uint32_t sample_freq) {
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
	    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
	    assert(system_clock_frequency < 0x40000000);
	    //                                           frac lrclk bclk
	    uint32_t divider = system_clock_frequency * (256 / 32 / 2) / sample_freq;
	    assert(divider < 0x1000000);
	    pio_sm_set_clkdiv_int_frac(pio, pio_sm, divider >> 8u, divider & 0xffu);
	}

	static void setup_dma_cont(const int16_t* buff, int dma_chan, int next_dma_chan)
	{
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
		dma_channel_config cfg_dma_chan_config = dma_channel_get_default_config(dma_chan);
	    channel_config_set_transfer_data_size(&cfg_dma_chan_config, DMA_SIZE_32);
	    channel_config_set_read_increment(&cfg_dma_chan_config, true);
	    channel_config_set_write_increment(&cfg_dma_chan_config, false);
	    channel_config_set_dreq(&cfg_dma_chan_config, pio_get_dreq(pio, pio_sm, true));
	    channel_config_set_chain_to(&cfg_dma_chan_config, next_dma_chan);
	    dma_channel_set_write_addr(dma_chan, &pio->txf[pio_sm], false);
	    dma_channel_set_read_addr(dma_chan, buff, false);
	    dma_channel_set_trans_count(dma_chan, DMA_BYTE_LEN/4, false);
	    dma_channel_set_config(dma_chan, &cfg_dma_chan_config, false);
	    dma_irqn_set_channel_enabled(IRQN, dma_chan, 1);
	}

	static void setup_dma_const(int dma_chan, int next_dma_chan, uint32_t * val)
	{
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
		dma_channel_config cfg_dma_chan_config = dma_channel_get_default_config(dma_chan);
	    channel_config_set_transfer_data_size(&cfg_dma_chan_config, DMA_SIZE_32);
	    channel_config_set_read_increment(&cfg_dma_chan_config, false);
	    channel_config_set_write_increment(&cfg_dma_chan_config, false);
	    channel_config_set_dreq(&cfg_dma_chan_config, pio_get_dreq(pio, pio_sm, true));
	    channel_config_set_chain_to(&cfg_dma_chan_config, next_dma_chan);
	    dma_channel_set_config(dma_chan, &cfg_dma_chan_config, false);
	    channel_config_set_chain_to(&cfg_dma_chan_config, dma_chan);
	    dma_channel_set_config(next_dma_chan, &cfg_dma_chan_config, false);

	    dma_channel_set_write_addr(dma_chan, &pio->txf[pio_sm], false);
	    dma_channel_set_read_addr(dma_chan, val, false);
	    dma_channel_set_trans_count(dma_chan, DMA_BYTE_LEN/4, false);

	    dma_channel_set_write_addr(next_dma_chan, &pio->txf[pio_sm], false);
	    dma_channel_set_read_addr(next_dma_chan, val, false);
	    dma_channel_set_trans_count(next_dma_chan, DMA_BYTE_LEN/4, false);

	}

	static void setup_dma_stop(int dma_chan)
	{
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
		dma_channel_config cfg_dma_chan_config = dma_channel_get_default_config(dma_chan);
	    channel_config_set_transfer_data_size(&cfg_dma_chan_config, DMA_SIZE_32);
	    channel_config_set_read_increment(&cfg_dma_chan_config, true);
	    channel_config_set_write_increment(&cfg_dma_chan_config, false);
	    channel_config_set_dreq(&cfg_dma_chan_config, pio_get_dreq(pio, pio_sm, true));
	    channel_config_set_chain_to(&cfg_dma_chan_config, dma_chan);
	    dma_channel_set_write_addr(dma_chan, &pio->txf[pio_sm], false);
	    dma_channel_set_read_addr(dma_chan, nullptr, false);
	    dma_channel_set_trans_count(dma_chan, 0, false);
	    dma_channel_set_config(dma_chan, &cfg_dma_chan_config, false);
	    dma_irqn_set_channel_enabled(IRQN, dma_chan, 1);
	}

	static inline std::atomic<bool> stopping; //isr context
	static inline std::atomic<bool> stopped; //isr context

	template<const int16_t* (*get_buff)(size_t req_buff)>
	static void isr()
	{
		if (dma_irqn_get_channel_status(IRQN, ping_dma_chan)) {
			if(stopping)
			{
			    dma_irqn_set_channel_enabled(IRQN, ping_dma_chan, 0);
				dma_irqn_set_channel_enabled(IRQN, pong_dma_chan, 0);
			    irq_set_enabled(DMA_IRQ_0+IRQN, false);
				irq_remove_handler(DMA_IRQ_0+IRQN,&isr<get_buff>);
				stopped = true;
				return;
			}
			const int16_t* buff = get_buff(DMA_BYTE_LEN);
			if(buff)
			{
				setup_dma_cont(buff,ping_dma_chan,pong_dma_chan);
			}
			else
			{
				setup_dma_stop(ping_dma_chan);
				stopping = true;
			}
			dma_irqn_acknowledge_channel(IRQN, ping_dma_chan);
		}
		if (dma_irqn_get_channel_status(IRQN, pong_dma_chan)) {
			if(stopping)
			{
			    dma_irqn_set_channel_enabled(IRQN, ping_dma_chan, 0);
				dma_irqn_set_channel_enabled(IRQN, pong_dma_chan, 0);
			    irq_set_enabled(DMA_IRQ_0+IRQN, false);
				irq_remove_handler(DMA_IRQ_0+IRQN,&isr<get_buff>);
				stopped = true;
				return;
			}
			const int16_t* buff = get_buff(DMA_BYTE_LEN);
			if(buff)
			{
				setup_dma_cont(buff,pong_dma_chan,ping_dma_chan);
			}
			else
			{
				setup_dma_stop(pong_dma_chan);
				stopping = true;
			}
			dma_irqn_acknowledge_channel(IRQN, pong_dma_chan);
		}
	}

	class Single_playback
	{
		static inline std::atomic<const int16_t*> buff;
		static inline std::atomic<ptrdiff_t> rem_len;

		Single_playback() = delete;

		static const int16_t* get_buff(size_t req_buff)
		{
			if(rem_len <=0)
				return nullptr;
			rem_len = rem_len - req_buff;
			const int16_t* ret = buff;
			buff = buff + req_buff/sizeof(*buff);
			return ret;
		}

	public:
		static void init_playback(const uint32_t sample_rate, const int16_t * _buff, const size_t byte_len)
		{
			buff = _buff;
			rem_len = byte_len;
			update_pio_frequency(sample_rate);
			stopped = false;
			const int16_t* ibuff = get_buff(DMA_BYTE_LEN);
			assert(ibuff);
			setup_dma_cont(ibuff,ping_dma_chan,pong_dma_chan);
			ibuff = get_buff(DMA_BYTE_LEN);
			if(ibuff)
			{
				setup_dma_cont(ibuff,pong_dma_chan,ping_dma_chan);
				stopping = false;
			}
			else
			{
				setup_dma_stop(pong_dma_chan);
				stopping = true;
			}
			irq_add_shared_handler(DMA_IRQ_0+IRQN,&isr<get_buff>,PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
		    dma_irqn_set_channel_enabled(IRQN, ping_dma_chan, 1);
		    dma_irqn_set_channel_enabled(IRQN, pong_dma_chan, 1);
		    dma_channel_acknowledge_irq0(ping_dma_chan);
		    dma_channel_acknowledge_irq0(pong_dma_chan);
		    irq_set_enabled(DMA_IRQ_0+IRQN, true);
		    dma_channel_start(ping_dma_chan);
		}
		static bool is_complete()
		{
			return stopped;
		}
	};

	template<const int16_t* (*get_buff)(size_t req_buff)>
	class GetbuffISR_playback {
	public:
		static void init_playback(const uint32_t sample_rate)
		{
			update_pio_frequency(sample_rate);
			stopped = false;
			const int16_t* ibuff = get_buff(DMA_BYTE_LEN);
			assert(ibuff);
			setup_dma_cont(ibuff,ping_dma_chan,pong_dma_chan);
			ibuff = get_buff(DMA_BYTE_LEN);
			if(ibuff)
			{
				setup_dma_cont(ibuff,pong_dma_chan,ping_dma_chan);
				stopping = false;
			}
			else
			{
				setup_dma_stop(pong_dma_chan);
				stopping = true;
			}
			irq_add_shared_handler(DMA_IRQ_0+IRQN,&isr<get_buff>,PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
		    dma_irqn_set_channel_enabled(IRQN, ping_dma_chan, 1);
		    dma_irqn_set_channel_enabled(IRQN, pong_dma_chan, 1);
		    irq_set_enabled(DMA_IRQ_0+IRQN, true);
		    dma_channel_start(ping_dma_chan);
		}
		static bool is_complete()
		{
			return stopped;
		}
	};
};


#if defined(AUDIO_DIN)
using AudioDMA = AudioDMA_Impl<AUDIO_DIN,AUDIO_BCK,AUDIO_LRCK,AUDIO_PIO, AUDIO_DMA_IRQN>;
#endif
};
