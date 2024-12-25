/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "pico/types.h"
#include "hardware/gpio.h"
#include "pico/time.h"

namespace AR1021 {

#include "ar1021.pio.h"

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
class AR1021_Impl {

	static inline int sm_no;
	static inline uint program_offset;

	static inline absolute_time_t timeout;


public:
	static void init()
	{
		volatile PIO pio = PIO_Selector::PIOPort<PION>();

		sm_no = pio_claim_unused_sm(pio,true);
		program_offset = pio_add_program(pio, &spi_shifted_program);

		pio_sm_config c;
		c = spi_shifted_program_get_default_config(program_offset);

	    sm_config_set_out_pins(&c, PIN_MOSI, 1);
	    sm_config_set_in_pins(&c, PIN_MISO);
	    sm_config_set_sideset_pins(&c, PIN_SCK);
	    sm_config_set_out_shift(&c, false, false, 8);
	    sm_config_set_in_shift(&c, false, true, 9);

		constexpr uint64_t clock_ns = 600/2;
		uint clock_hz = clock_get_hz(clk_sys);
		uint div = (clock_hz*clock_ns)/1000000000UL;
		if ((clock_hz*clock_ns)%1000000000UL)
			div+=1;

		sm_config_set_clkdiv_int_frac(&c, div,0);

	    pio_sm_set_consecutive_pindirs(pio, sm_no, PIN_SCK, 1, true);
	    pio_sm_set_consecutive_pindirs(pio, sm_no, PIN_MOSI, 1, true);
	    pio_sm_set_consecutive_pindirs(pio, sm_no, PIN_MISO, 1, false);
	    pio_gpio_init(pio, PIN_MOSI);
	    gpio_set_slew_rate(PIN_MOSI,GPIO_SLEW_RATE_FAST);

	    gpio_set_dir(PIN_CS, GPIO_OUT);
	    gpio_put(PIN_CS, 1);
	    gpio_set_function(PIN_CS, GPIO_FUNC_SIO);

	    gpio_set_slew_rate(PIN_CS,GPIO_SLEW_RATE_FAST);
	    pio_gpio_init(pio, PIN_SCK);
	    gpio_set_slew_rate(PIN_SCK,GPIO_SLEW_RATE_FAST);

	    pio_sm_init(pio, sm_no, program_offset + spi_shifted_offset_entry, &c);
	    pio_sm_set_enabled(pio, sm_no, true);

	    timeout = 0;
	}

	template<class Thread>
	static uint8_t transfer_byte_sync(uint8_t out, Thread * thread)
	{
		while(timeout >get_absolute_time())
			thread->yield();
		volatile PIO pio = PIO_Selector::PIOPort<PION>();
		//pio_sm_put(pio,sm_no,out);
	    gpio_put(PIN_CS, 0);
		busy_wait_at_least_cycles(550e-9*380e6+1);
	    *reinterpret_cast<volatile uint8_t*>(&pio->txf[sm_no]) = out;
		while(pio_sm_is_rx_fifo_empty(pio,sm_no))
			thread->yield();
	    gpio_put(PIN_CS, 1);
		timeout  = delayed_by_us(get_absolute_time(),50*2);
		uint8_t ret = pio_sm_get(pio,sm_no)>>1;
		return ret;
	}

	template<class Thread>
	static void transfer_sync(const uint8_t * tx_buff,uint tx_len,uint8_t * rx_buff,uint rx_len, Thread * thread)
	{
		uint pktlen = std::max(tx_len,rx_len);
		for(uint i=0;i<pktlen;i++)
		{
			uint8_t tx_byte;
			if(tx_len)
			{
				tx_byte = tx_buff[i];
				tx_len--;
			}
			else
			{
				tx_byte = 0x00;
			}
			uint8_t rx_byte = transfer_byte_sync(tx_byte,thread);
			if(rx_len)
			{
				rx_buff[i] = rx_byte;
				rx_len--;
			}
		}
	}


	template<class Thread>
	static void command(uint8_t id, const uint8_t * tx, uint tx_len, Thread * thread)
	{
		transfer_byte_sync(0x55,thread);
		transfer_byte_sync(tx_len+1,thread);
		transfer_byte_sync(id,thread);
		transfer_sync(tx,tx_len,nullptr,0,thread);
	}

	template<class Thread>
	static uint8_t response(uint8_t id, uint8_t * rx, uint &rx_len, Thread * thread, uint sync_tries=3)
	{
		while(sync_tries--)
		{
			uint8_t hdr = transfer_byte_sync(0x00,thread);
			if(hdr==0x55)
				goto found; //try be more concise without goto
		}
		return 0x83; // Header Unrecognized on receiver side
		found:
		uint8_t buff[3];
		transfer_sync(nullptr,0,buff,3,thread);
		uint data_len = buff[0]-2;
		rx_len = std::min(rx_len,data_len);
		transfer_sync(nullptr,0,rx,rx_len,thread);
		uint rest_len = data_len - rx_len;
		while(rest_len--)
		{
			transfer_byte_sync(0x00,thread);
		}
		if(buff[2]!=id)
			return 0x81; //Reply Unrecognized
		return buff[1];
	}

	template<class Thread>
	static bool get_version(uint16_t &version, uint8_t &type, Thread * thread)
	{
		command(0x10,nullptr,0,thread);
		uint8_t buff[3];
		uint buff_len = 3;
		if (response(0x10,buff,buff_len,thread) == 0)
		{
			version = (buff[0]<<8) | buff[1];
			type = buff[2];
			return true;
		}
		return false;
	}

	template<class Thread>
	static bool enable_touch(Thread * thread)
	{
		command(0x12,nullptr,0,thread);
		uint nullptr_len = 0;
		return response(0x12,nullptr,nullptr_len,thread) == 0;
	}

	template<class Thread>
	static bool disable_touch(Thread * thread)
	{
		command(0x13,nullptr,0,thread);
		uint nullptr_len = 0;
		return response(0x13,nullptr,nullptr_len,thread) == 0;
	}
protected:
	static inline uint8_t reg_offset = 0xFF;

	template<class Thread>
	static uint8_t reg_start_addr(Thread * thread)
	{
		if(reg_offset == 0xFF)
		{
			command(0x22,nullptr,0,thread);
			uint8_t buff[1];
			uint buff_len = 1;
			if(response(0x22,buff,buff_len,thread) == 0)
			{
				reg_offset = buff[0];
			}
		}
		return reg_offset;
	}

public:
	template<class Thread>
	static bool regs_read(uint8_t addr, uint8_t * regs,uint regs_len, Thread * thread)
	{
		uint8_t off = reg_start_addr(thread);
		if (off == 0xFF)
			return false;
		uint16_t addr_off = addr+off;
		uint8_t buff[3] =
		{
				addr_off>>8,
				addr_off,
				regs_len,
		};
		uint buff_len = 3;
		command(0x20,buff,buff_len,thread);
		return response(0x20,regs,regs_len,thread) == 0;
	}

	template<class Thread>
	static bool regs_write(uint8_t addr, const uint8_t * regs,uint regs_len, Thread * thread)
	{
		uint8_t off = reg_start_addr(thread);
		if (off == 0xFF)
			return false;
		uint16_t addr_off = addr+off;
		transfer_byte_sync(0x55,thread);
		transfer_byte_sync(regs_len+4,thread);
		transfer_byte_sync(0x21,thread);
		transfer_byte_sync(addr_off>>16,thread);
		transfer_byte_sync(addr_off,thread);
		transfer_byte_sync(regs_len,thread);
		transfer_sync(regs,regs_len,nullptr,0,thread);

		return response(0x21,nullptr,0,thread) == 0;
	}

	struct Report {
		bool valid = false;
		bool pen;
		uint16_t x;
		uint16_t y;
	};

	template<class Thread>
	static Report get_pos(Thread * thread, uint sync_tries=6)
	{
		Report ret = {.valid = false};
		while(sync_tries--)
		{
			uint8_t hdr = transfer_byte_sync(0x00,thread);
			if((hdr&0x80)==0x80)
			{ //no goto but more nested blocks - hdr used + struct ret
				uint8_t buff[4];
				transfer_sync(nullptr,0,buff,4,thread);
				if((buff[0]&0x80)==0x00)
					if((buff[1]&0x80)==0x00)
						if((buff[2]&0x80)==0x00)
							if((buff[3]&0x80)==0x00)
							{
								ret.valid = true;
								ret.pen = (hdr&0x01==0x01);
								ret.x = (buff[1]<<7) | buff[0];
								ret.y = (buff[3]<<7) | buff[2];
							}
			}
		}
		return ret;
	}
};


#if defined(TOUCH_PIN_CS)
using AR1021 = AR1021_Impl<TOUCH_PIN_MISO,TOUCH_PIN_MOSI,TOUCH_PIN_SCK,TOUCH_PIN_CS,TOUCH_PIO>;
#else
class AR1021 {
public:
	static void init() {};
};
#endif
};
