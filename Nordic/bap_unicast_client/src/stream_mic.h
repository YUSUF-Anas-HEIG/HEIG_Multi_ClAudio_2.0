#ifndef STREAM_I2S_H_ // Reworked from I2S code iteration
#define STREAM_I2S_H_

#include <stdint.h>
#include <stddef.h>



int stream_i2s_init(void);


int stream_i2s_start(uint32_t sample_rate_hz);


int stream_i2s_read(int16_t *buf, size_t num_samples);

uint32_t stream_i2s_buf_occupancy(void);

void reset_pdm_stream(void);

void partial_reset_pdm_stream(void);

void stream_i2s_reset(void);


#endif 
