#pragma once

#include "pico/types.h"
#include "hardware/gpio.h"
#include "pico/time.h"

namespace AR1021 {
template<uint PIN_MISO, uint PIN_MOSI, uint PIN_SCK, uint PIN_CS, uint PIO>
class AR1021_Impl {
public:
	static void set_cs_disabled()
	{
	    gpio_put(PIN_CS, 1);
		gpio_set_dir(PIN_CS, GPIO_OUT);
		gpio_set_function(PIN_CS,GPIO_FUNC_SIO);
	}
};


#if defined(TOUCH_PIN_CS)
using AR1021 = AR1021_Impl<TOUCH_PIN_MISO,TOUCH_PIN_MOSI,TOUCH_PIN_SCK,TOUCH_PIN_CS,TOUCH_PIO>;
#endif
};
