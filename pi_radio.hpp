void setupDMA(float centerFreq);
void unSetupDMA();
void setupFm();


class SampleSink {
public:
    virtual void consume(float* const, const int) {}; // floating point samples
    virtual void consume(void* const, const int) {}; // raw data, len in bytes.
    virtual ~SampleSink() {};
};


class Outputter : public SampleSink {
public:
    int bufPtr_;
    const float clocksPerSample_;
    const int sleeptime_;
    float fracerror_;
    float timeErr_;

    Outputter(float rate);
    void consume(float* const data, const int num);
};


class PreEmp : public SampleSink {
public:
    const float fmconstant_;
    float dataold_;
    SampleSink* const next_;

    // this isn't the right filter...  But it's close...
    // Something todo with a bilinear transform not being right...
    PreEmp(const float rate, SampleSink* next);

    ~PreEmp();

    void consume(float* const data, const int num);
private:
    PreEmp(const PreEmp&);
    PreEmp& operator=(const PreEmp&);
};


class Resamp : public SampleSink {
public:
    static const int QUALITY = 5;    // comp. complexity goes up linearly with this.
    static const int SQUALITY = 10;  // start time quality (defines max phase error of filter vs ram used & cache thrashing)
    static const int BUFSIZE = 1000;
    float dataOld_[QUALITY];
    float sincLUT_[SQUALITY][QUALITY]; // [startime][samplenum]
    float ratio_;
    float outTimeLeft_;
    float outBuffer_[BUFSIZE];
    int outBufPtr_;
    SampleSink* const next_;

    Resamp(float rateIn, float rateOut, SampleSink* next);
    ~Resamp();
    void consume(float* const data, const int num);
private:
    Resamp(const Resamp&);
    Resamp& operator=(const Resamp&);
};


// decodes a mono wav file
class Mono : public SampleSink {
public:
    SampleSink* const next_;
    Mono(SampleSink* next);

    void consume(void* const data, const int num);  // expects num%2 == 0
    ~Mono();
private:
    Mono(const Mono&);
    Mono& operator=(const Mono&);
};


class StereoSplitter : public SampleSink {
public:
    SampleSink* nextLeft_;
    SampleSink* nextRight_;

    StereoSplitter(SampleSink* nextLeft, SampleSink* nextRight);

    ~StereoSplitter();

    void consume(void* const data, const int num);  // expects num%4 == 0
private:
    StereoSplitter(const StereoSplitter&);
    StereoSplitter& operator=(const StereoSplitter&);
};


class RdsEncoder : public SampleSink {
public:
    float sinLut_[8];
    SampleSink* const next_;
    int bitNum_;
    int lastBit_;
    int time_;
    float lastValue_;

    RdsEncoder(SampleSink* next);
    ~RdsEncoder();

    void consume(float* const data, const int num);
private:
    RdsEncoder(const RdsEncoder&);
    RdsEncoder& operator=(const RdsEncoder&);
};


// Takes 2 input signals at 152kHz and stereo modulates it.
class StereoModulator : public SampleSink {
public:
    // Helper to make two input interfaces for the stereomodulator.   Feels like I'm reimplementing a closure here... :-(
    class ModulatorInput : public SampleSink {
    public:
        StereoModulator* const mod_;
        const int channel_;
        ModulatorInput(StereoModulator* mod, int channel);
        ~ModulatorInput();
        void consume(float* const data, const int num);
    private:
        ModulatorInput(const ModulatorInput&);
        ModulatorInput& operator=(const ModulatorInput&);
    };

    float buffer_[1024];
    int bufferOwner_;
    int bufferLen_;
    int state_; // 8 state state machine.
    float sinLut_[16];

    SampleSink* next_;

    StereoModulator(SampleSink* next);
    ~StereoModulator();
    SampleSink* getChannel(int channel);

    void consume(float* data, int num, const int channel);
private:
    StereoModulator(const StereoModulator&);
    StereoModulator& operator=(const StereoModulator&);
};
