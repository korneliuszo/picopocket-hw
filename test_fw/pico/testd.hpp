#pragma once

#include <stdint.h>
#include <stddef.h>

#include "jmpcoro_configured.hpp"

[[gnu::aligned(4)]] extern volatile uint8_t monitor_data_window[2048];

void testd_install(Thread * main);
uint8_t testd_get_phase();

