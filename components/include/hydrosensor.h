#pragma once

void hydrosensor_init(int adc_channel);
float hydrosensor_read_pressure(void);
float hydrosensor_read_height(void);