// Original PiFM created by Oliver Mattos and Oskar Weigl.
// RC control stuff added by Brandon Skari.

#include <cassert>
#include <limits>
#include <cstdlib>
using std::numeric_limits;
using std::atof;
using std::atoi;

#include "pi_radio.hpp"


void playSignal(
    const int burstsCount,
    const int burstMicroseconds,
    const int spacingMicroseconds,
    SampleSink* const ss
) {
    typedef float data_t;
    data_t data[0x1000];
    int dataIndex = 0;

    const int burstLength = burstMicroseconds / ss->getSampleRate() * 1000000;
    const int spacingLength = spacingMicroseconds / ss->getSampleRate() * 1000000;
    for (int i = 0; i < burstsCount; ++i) {
        for (int j = 0; j < burstLength; ++j) {
            data[dataIndex] = numeric_limits<data_t>::max();
            ++dataIndex;
            assert(dataIndex < int(sizeof(data) / sizeof(data[0])));
        }
        for (int j = 0; j < spacingLength; ++j) {
            data[dataIndex] = numeric_limits<data_t>::min();
            ++dataIndex;
            assert(dataIndex < int(sizeof(data) / sizeof(data[0])));
        }
        ss->consume(&data, dataIndex);
    }
}


int main(int argc, char** argv) {
    setupFm();
    const float channel1 = 49.830f;
    setupDMA(argc > 1 ? atof(argv[1]) : channel1);

    const int iterations = (argc > 2 ? atoi(argv[2]) : 1000);

    Outputter out(44100);
    for (int i = 0; i < iterations; ++i) {
        playSignal(4, 700 * 3, 700, &out);
        playSignal(16, 700, 700, &out);
    }
    /*
    for (int pulses = 4; pulses < 100; pulses += 1) {
        printf("Trying %d pulses\n", pulses);
        fflush(stdout);
        // Value chosen experimentally for 49.830 MHz
        const int oneSecondIterations = 66666 / pulses;
        for (int i = 0; i < oneSecondIterations; ++i) {
            // Synchronization signal
            playSignal(4, 700 * 3, 700, &out);
            // Command
            playSignal(pulses, 700, 700, &out);
        }
    }
    */

    return 0;
}
