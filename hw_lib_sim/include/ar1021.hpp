/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

namespace AR1021 {


class AR1021_Impl {


public:
	static void init()
	{

	}


	template<class Thread>
	static bool get_version(uint16_t &version, uint8_t &type, Thread * thread)
	{
		version = 0xbadf;
		type = 0xfe;
		return true;
	}

	template<class Thread>
	static bool enable_touch(Thread * thread)
	{
		return true;
	}

	template<class Thread>
	static bool disable_touch(Thread * thread)
	{
		return true;
	}
protected:

public:

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

		ret.valid = true;
		ret.pen = 0x00;
		ret.x = 0x0100;
		ret.y = 0x0100;
		return ret;
	}
};


using AR1021 = AR1021_Impl;
};
