#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "pi_radio.h"

// The deviation specifies how wide the signal is. Use 25.0 for WBFM
// (broadcast radio) and about 3.5 for NBFM (walkie-talkie style radio)
#define DEVIATION       25.0


void signal_handler(int signal);


int main(int argc, char** argv) {
    // Open the .wav file specified on the cmdline
    int fd = 0;

    if (argc > 1) {
        fd = open(argv[1], 'r');

        if (fd < 0) {
            fatal("Failed to open .wav file\n");
        }
    } else {
        fatal("Usage: %s <wav file>\n", argv[0]);
        exit(-1);
    }

    int i;

    // Catch all signals possible - it is vital we kill the DMA engine
    // on process exit!
    for (i = 0; i < 64; i++) {
        struct sigaction sa;

        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = signal_handler;
        sigaction(i, &sa, NULL);
    }

    // Calculate the frequency control word
    // The fractional part is stored in the lower 12 bits
    const int freq_ctl = ((float)(PLLFREQ / CARRIERFREQ)) * ( 1 << 12 );

    const int silence = 0x5a << 24 | freq_ctl;
    initialize_dma();
    initialize_dma_memory(silence);

    short wav_data[1024];
    int wav_data_len = read(fd, wav_data, sizeof(wav_data));
    if (wav_data_len < 0) {
        fatal("Failed to read .wav file\n");
    }
    wav_data_len /= 2;
    if (wav_data_len < 23) {
        fatal("Initial read of .wav file too short\n");
    }

    int wav_data_index = 22;

    int last_offset = 0;
    uint32_t dma_data[NUM_SAMPLES];

    for (;;) {
        usleep(10000);

        int free_slots = get_refill_size();
        const int offset = free_slots % 10;
        int dma_data_index = 0;
        int j;
        float dval = (float)(wav_data[wav_data_index]) / 65536.0 * DEVIATION;
        int intval = (int)((floor)(dval));
        int frac = (int)((dval - (float)intval) * 10.0);

        for (j = last_offset; j < 10; ++j) {
            dma_data[dma_data_index] = (0x5A << 24 | freq_ctl) + (frac > j ? intval + 1 : intval);
        }
        free_slots -= offset;
        last_offset = offset;

        while (free_slots >= 10) {
            dval = (float)(wav_data[wav_data_index]) / 65536.0 * DEVIATION;
            intval = (int)((floor)(dval));
            frac = (int)((dval - (float)intval) * 10.0);

            // I'm sure this code could do a better job of subsampling, either by
            // distributing the '+1's evenly across the 10 subsamples, or maybe
            // by taking the previous and next samples in to account too.
            for (j = 0; j < 10; j++) {
                dma_data[dma_data_index] = (0x5A << 24 | freq_ctl) + (frac > j ? intval + 1 : intval);
                ++dma_data_index;
            }
            free_slots -= 10;
            if (++wav_data_index >= wav_data_len) {
                wav_data_len = read(fd, wav_data, sizeof(wav_data));
                wav_data_index = 0;
                if (wav_data_len < 0) {
                    fatal("Error reading data: %m\n");
                }
                // Should really wait for outstanding samples to be processed here..
                wav_data_len /= 2;
                if (wav_data_len == 0) {
                    terminate();
                }
            }
        }

        refill(dma_data);
    }

    terminate();

    return 0;
}


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void signal_handler(const int signal_) {
    terminate();
}
#pragma GCC diagnostic pop
