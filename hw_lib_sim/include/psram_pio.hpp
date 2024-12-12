/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <cstring>

namespace PSRAM {


class PSRAM_Impl {

	PSRAM_Impl() = delete;
	static inline uint8_t psram[8*1024*1024];
public:

	static void init(bool read_burst_cross_boundary = true)
	{

	}


	static uint64_t read_id_sync()
	{
		return 0xbad0fa1d;
	}

	class PSRAM_Completion {
	protected:
		friend class PSRAM_Impl;
		PSRAM_Completion() {};
		PSRAM_Completion(int dma_chan) {};
	public:
		bool complete_trigger()
		{
			return true;
		}
	};



	static PSRAM_Completion read_async_mem(uint32_t addr,volatile uint8_t *buff, uint len)
	{
		memcpy((void*)buff,&psram[addr],len);
		return {};
	}


	static PSRAM_Completion write_async_mem(uint32_t addr, const uint8_t *buff, uint len)
	{
		memcpy(&psram[addr],buff,len);
		return {};
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

using PSRAM = PSRAM_Impl;
};
