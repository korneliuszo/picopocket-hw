/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "pico/types.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

namespace PSRAM {

#include "psram.pio.h"

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

template<uint PIN_MISO, uint PIN_MOSI, uint PIN_SCK, uint PIN_CS, uint PION>
class PSRAM_Impl {

	static_assert(PIN_CS + 1 == PIN_SCK);

	static inline int sm_no;
	static inline uint program_offset;

	static inline int cfg_dma_chan;
	static inline dma_channel_config cfg_dma_chan_config;
	static inline int write_dma_chan;
	static inline dma_channel_config write_dma_chan_config;
	static inline int read_dma_chan;
	static inline dma_channel_config read_dma_chan_config;

	PSRAM_Impl() = delete;
public:

	static void init(bool read_burst_cross_boundary = true)
	{
		(void)read_burst_cross_boundary; // only matters in QSPI

		volatile PIO pio = PIO_Selector::PIOPort<PION>();

		sm_no = pio_claim_unused_sm(pio,true);
		program_offset = pio_add_program(pio, &spi_psram_dma_program);

		pio_sm_config c;
		c = spi_psram_dma_program_get_default_config(program_offset);

	    sm_config_set_out_pins(&c, PIN_MOSI, 1);
	    sm_config_set_in_pins(&c, PIN_MISO);
	    sm_config_set_sideset_pins(&c, PIN_CS);
	    sm_config_set_out_shift(&c, false, true, 8);
	    sm_config_set_in_shift(&c, false, true, 8);

		constexpr uint64_t clock_ns = 5;
		uint clock_hz = clock_get_hz(clk_sys);
		uint div = (clock_hz*clock_ns)/1000000000UL;
		if ((clock_hz*clock_ns)%1000000000UL)
			div+=1;

		sm_config_set_clkdiv_int_frac(&c, div,0);

	    pio_sm_set_consecutive_pindirs(pio, sm_no, PIN_CS, 2, true);
	    pio_sm_set_consecutive_pindirs(pio, sm_no, PIN_MOSI, 1, true);
	    pio_sm_set_consecutive_pindirs(pio, sm_no, PIN_MISO, 1, false);
	    pio_gpio_init(pio, PIN_MOSI);
	    gpio_set_slew_rate(PIN_MOSI,GPIO_SLEW_RATE_FAST);
	    pio_gpio_init(pio, PIN_CS);
	    gpio_set_slew_rate(PIN_CS,GPIO_SLEW_RATE_FAST);
	    pio_gpio_init(pio, PIN_SCK);
	    gpio_set_slew_rate(PIN_SCK,GPIO_SLEW_RATE_FAST);

	    //are near CLK_SYS speeds and can use it
	    hw_set_bits(&pio->input_sync_bypass, 1u << PIN_MISO);

	    pio_sm_init(pio, sm_no, program_offset + spi_psram_dma_offset_entry, &c);
	    pio_sm_set_enabled(pio, sm_no, true);

	    // Write DMA channel setup
	    write_dma_chan = dma_claim_unused_channel(true);
	    write_dma_chan_config = dma_channel_get_default_config(write_dma_chan);
	    channel_config_set_transfer_data_size(&write_dma_chan_config, DMA_SIZE_8);
	    channel_config_set_read_increment(&write_dma_chan_config, true);
	    channel_config_set_write_increment(&write_dma_chan_config, false);
	    channel_config_set_dreq(&write_dma_chan_config, pio_get_dreq(pio, sm_no, true));
	    dma_channel_set_write_addr(write_dma_chan, &pio->txf[sm_no], false);
	    dma_channel_set_config(write_dma_chan, &write_dma_chan_config, false);

	    // config DMA channel setup
	    cfg_dma_chan = dma_claim_unused_channel(true);
	    cfg_dma_chan_config = dma_channel_get_default_config(cfg_dma_chan);
	    channel_config_set_transfer_data_size(&cfg_dma_chan_config, DMA_SIZE_32);
	    channel_config_set_read_increment(&cfg_dma_chan_config, true);
	    channel_config_set_write_increment(&cfg_dma_chan_config, false);
	    channel_config_set_dreq(&cfg_dma_chan_config, pio_get_dreq(pio, sm_no, true));
	    channel_config_set_chain_to(&cfg_dma_chan_config, write_dma_chan);
	    dma_channel_set_write_addr(cfg_dma_chan, &pio->txf[sm_no], false);
	    dma_channel_set_config(cfg_dma_chan, &cfg_dma_chan_config, false);

	    // Read DMA channel setup
	    read_dma_chan = dma_claim_unused_channel(true);
	    read_dma_chan_config = dma_channel_get_default_config(read_dma_chan);
	    channel_config_set_transfer_data_size(&read_dma_chan_config, DMA_SIZE_8);
	    channel_config_set_read_increment(&read_dma_chan_config, false);
	    channel_config_set_write_increment(&read_dma_chan_config, true);
	    channel_config_set_dreq(&read_dma_chan_config, pio_get_dreq(pio, sm_no, false));
	    dma_channel_set_read_addr(read_dma_chan, &pio->rxf[sm_no], false);
	    dma_channel_set_config(read_dma_chan, &read_dma_chan_config, false);

		uint8_t reset_en = 0x66;
		write_sync(&reset_en,1);

		uint8_t reset = 0x99;
		write_sync(&reset,1);
	}

	static void write_sync(const uint8_t* buff,uint len)
	{
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
		uint32_t cfg[] = {len*8-1,0};

	    dma_channel_set_read_addr(write_dma_chan,buff,false);
	    dma_channel_set_trans_count(write_dma_chan,len,false);
	    dma_channel_transfer_from_buffer_now(cfg_dma_chan, cfg, 2);

	    dma_channel_wait_for_finish_blocking(write_dma_chan);
	}

	static void read_sync(uint8_t cmd, uint32_t addr, bool waitcycle,volatile uint8_t *buff, uint len)
	{
		uint8_t cmd_buff[5] = {
				cmd,
				(uint8_t)(addr>>16),
				(uint8_t)(addr>>8),
				(uint8_t)(addr>>0),
				0
		};
		uint32_t cfg[] = {(waitcycle?5u:4u)*8-1,len*8-1};
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
	    dma_channel_transfer_to_buffer_now(read_dma_chan, buff, len);
	    dma_channel_set_read_addr(write_dma_chan,cmd_buff,false);
	    dma_channel_set_trans_count(write_dma_chan,(waitcycle?5:4),false);
	    dma_channel_transfer_from_buffer_now(cfg_dma_chan, cfg, 2);
	    dma_channel_wait_for_finish_blocking(read_dma_chan);
	}

	static uint64_t read_id_sync()
	{
		uint8_t buff[8];
		read_sync(0x9F,0,false,buff,8);
		return
				(((uint64_t)buff[0]) <<(7*8)) |
				(((uint64_t)buff[1]) <<(6*8)) |
				(((uint64_t)buff[2]) <<(5*8)) |
				(((uint64_t)buff[3]) <<(4*8)) |
				(((uint64_t)buff[4]) <<(3*8)) |
				(((uint64_t)buff[5]) <<(2*8)) |
				(((uint64_t)buff[6]) <<(1*8)) |
				(((uint64_t)buff[7]) <<(0*8));
	}

	class PSRAM_Completion {
		int completion_dma_chan;
	protected:
		friend class PSRAM_Impl;
		PSRAM_Completion() :completion_dma_chan(0xff){};
		PSRAM_Completion(int dma_chan) :completion_dma_chan(dma_chan){};
	public:
		bool complete_trigger()
		{
			return !dma_channel_is_busy(completion_dma_chan);
		}
	};

	static PSRAM_Completion read_async(uint8_t cmd, uint32_t addr, bool waitcycle,volatile uint8_t *buff, uint len)
	{
		uint8_t cmd_buff[5] = {
				cmd,
				(uint8_t)(addr>>16),
				(uint8_t)(addr>>8),
				(uint8_t)(addr>>0),
				0
		};
		uint32_t cfg[] = {(waitcycle?5u:4u)*8-1,len*8-1};
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
	    dma_channel_transfer_to_buffer_now(read_dma_chan, buff, len);
	    dma_channel_set_read_addr(write_dma_chan,cmd_buff,false);
	    dma_channel_set_trans_count(write_dma_chan,(waitcycle?5:4),false);
	    dma_channel_transfer_from_buffer_now(cfg_dma_chan, cfg, 2);
	    dma_channel_wait_for_finish_blocking(write_dma_chan);
	    return {read_dma_chan};
	}

	static PSRAM_Completion read_async_mem(uint32_t addr,volatile uint8_t *buff, uint len)
	{
		return read_async(0x0b,addr,true,buff,len);
	}

	static PSRAM_Completion write_async(uint8_t cmd, uint32_t addr, const uint8_t  *buff, uint len)
	{
		uint32_t cfg[] = {
				(4u+len)*8-1,
				0,
				static_cast<uint32_t>(cmd)<<24,
				addr<<(24-16),
				addr<<(24-8),
				addr<<(24-0),
		};
		volatile PIO pio = PIO_Selector::PIOPort<PION>();

	    dma_channel_set_read_addr(write_dma_chan,buff,false);
	    dma_channel_set_trans_count(write_dma_chan,len,false);
	    dma_channel_transfer_from_buffer_now(cfg_dma_chan, cfg, 2+4);
	    dma_channel_wait_for_finish_blocking(cfg_dma_chan);

	    return {write_dma_chan};
	}

	static PSRAM_Completion write_async_mem(uint32_t addr, const uint8_t *buff, uint len)
	{
		return write_async(0x02,addr,buff,len);
	}

	template<typename BUFF_T, PSRAM_Completion (*FN)(uint32_t, BUFF_T,uint),uint32_t BLOCKSIZE = 1024>
	class PSRAM_WRAPPED_Completion : public PSRAM_Completion {
		uint32_t addr;
		BUFF_T buff;
		uint rem_len;

		void setup_cycle()
		{
			static_assert((BLOCKSIZE&(BLOCKSIZE-1))==0);
			uint32_t pos_in_block = addr & (BLOCKSIZE-1); // faster version of modulo

			uint len = std::min(rem_len,static_cast<uint>(BLOCKSIZE-pos_in_block));
			static_cast<PSRAM_Completion&&>(*this) = FN(addr,buff,len);
			addr+=len;
			buff+=len;
			rem_len-=len;
		}

	public:
		PSRAM_WRAPPED_Completion(uint32_t _addr, BUFF_T _buff, uint _len)
		: addr(_addr)
		, buff(_buff)
		, rem_len(_len)
		{
			setup_cycle();
		}
		bool complete_trigger()
		{
			if(!PSRAM_Completion::complete_trigger())
				return false;
			if(!rem_len)
				return true;
			setup_cycle();
			return false;
		}

	};

	using Read_Mem_Task = PSRAM_WRAPPED_Completion<volatile uint8_t *,read_async_mem>;
	using Write_Mem_Task = PSRAM_WRAPPED_Completion<const uint8_t *,write_async_mem>;
};

#if defined(PSRAM_PIN_CS)
using PSRAM = PSRAM_Impl<PSRAM_PIN_MISO,PSRAM_PIN_MOSI,PSRAM_PIN_SCK,PSRAM_PIN_CS,PSRAM_PIO>;
#endif
};
