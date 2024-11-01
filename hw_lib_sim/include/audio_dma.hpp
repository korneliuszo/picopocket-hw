/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <atomic>

namespace AudioDMA {



class AudioDMA_Impl {

	AudioDMA_Impl() = delete;
public:

	static void init()
	{


	}


	class Single_playback
	{
		Single_playback() = delete;


	public:
		static void init_playback(const uint32_t sample_rate, const int16_t * _buff, const size_t byte_len)
		{

		}
		static bool is_complete()
		{
			return 1;
		}
	};
};


using AudioDMA = AudioDMA_Impl;
};
