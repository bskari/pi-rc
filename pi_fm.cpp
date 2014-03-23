#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
using std::cerr;
using std::ifstream;

#include "pi_radio.hpp"


bool playWav(const char* fileName, float sampleRate, bool stereo);


int main(int argc, char** argv) {
    if (argc > 1) {
        setupFm();
        // We could use boost::lexical_cast here instead of atof, but just
        // including the header for it increased the compile time from 4s to
        // 15s, and that's terrible
        const float frequency = (argc > 2 ? atof(argv[2]) : 103.3f);
        setupDMA(frequency);
        const float sampleRate = (argc > 3 ? atof(argv[3]) : 22050.0f);
        const bool stereo = (argc > 4);
        const bool success = playWav(argv[1], sampleRate, stereo);
        if (!success) {
            cerr << "Unable to play file\n";
            exit(-1);
        }
    } else {
        cerr <<  "Usage:  program wavfile.wav [freq] [sample rate] [stereo]\n\n"
             "Where wavfile is 16 bit 22.5kHz Stereo.  Set wavfile to '-' to use stdin.\n"
             "freq is in Mhz (default 103.3)\n"
             "sample rate of wav file in Hz\n\n"
             "Play an empty file to transmit silence\n";
    }

    exit(0);
}


bool playWav(const char* const fileName, const float sampleRate, const bool stereo) {
    ifstream fin(fileName, ifstream::binary);
    if (!fin) {
        return false;
    }

    char data[1024];

    SampleSink* ss;

    if (stereo) {
        StereoModulator* sm = new StereoModulator(new RdsEncoder(new Outputter(152000)));
        ss = new StereoSplitter(
            // left
            new PreEmp(sampleRate, new Resamp(sampleRate, 152000, sm->getChannel(0))),

            // Right
            new PreEmp(sampleRate, new Resamp(sampleRate, 152000, sm->getChannel(1)))
        );
    } else {
        ss = new Mono(new PreEmp(sampleRate, new Outputter(sampleRate)));
    }


    // Read past header
    fin.seekg(44, fin.beg);

    int readBytes;
    do {
        fin.read(data, 1024);
        readBytes = fin.gcount();
        ss->consume(data, readBytes);
    } while (readBytes > 0 && !!fin);
    fin.close();
    return true;
}
