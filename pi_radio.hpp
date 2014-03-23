void setupDMA(float centerFreq);
void unSetupDMA();
void setupFm();


class SampleSink {
public:
    virtual void consume(float*, int) {}; // floating point samples
    virtual void consume(void*, int) {}; // raw data, len in bytes.
    virtual ~SampleSink() {};
    virtual int getSampleRate() const = 0;
};


class Outputter : public SampleSink {
public:
    Outputter(float sampleRate);
    void consume(float* data, int num);
    int getSampleRate() const;

private:
    int bufPtr_;
    const float clocksPerSample_;
    const int sleeptime_;
    float fracerror_;
    float timeErr_;
    const float sampleRate_;
};


class PreEmp : public SampleSink {
public:

    // this isn't the right filter...  But it's close...
    // Something todo with a bilinear transform not being right...
    PreEmp(const float rate, SampleSink* next);
    ~PreEmp();

    void consume(float* data, int num);
    int getSampleRate() const;

private:
    const float fmconstant_;
    float dataold_;
    SampleSink* const next_;

    PreEmp(const PreEmp&);
    PreEmp& operator=(const PreEmp&);
};


class Resamp : public SampleSink {
public:
    void consume(float* data, int num);
    int getSampleRate() const;

    Resamp(float rateIn, float rateOut, SampleSink* next);
    ~Resamp();

private:
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

    Resamp(const Resamp&);
    Resamp& operator=(const Resamp&);
};


// decodes a mono wav file
class Mono : public SampleSink {
public:
    Mono(SampleSink* next);
    ~Mono();

    void consume(void* data, int num);  // expects num%2 == 0
    int getSampleRate() const;

private:
    SampleSink* const next_;

    Mono(const Mono&);
    Mono& operator=(const Mono&);
};


class StereoSplitter : public SampleSink {
public:
    SampleSink* nextLeft_;
    SampleSink* nextRight_;

    StereoSplitter(SampleSink* nextLeft, SampleSink* nextRight);

    ~StereoSplitter();

    void consume(void* data, int num);  // expects num%4 == 0
    int getSampleRate() const;
private:
    StereoSplitter(const StereoSplitter&);
    StereoSplitter& operator=(const StereoSplitter&);
};


class RdsEncoder : public SampleSink {
public:
    RdsEncoder(SampleSink* next);
    ~RdsEncoder();

    void consume(float* data, int num);
    int getSampleRate() const;

private:
    float sinLut_[8];
    SampleSink* const next_;
    int bitNum_;
    int lastBit_;
    int time_;
    float lastValue_;

    RdsEncoder(const RdsEncoder&);
    RdsEncoder& operator=(const RdsEncoder&);
};


// Takes 2 input signals at 152kHz and stereo modulates it.
class StereoModulator : public SampleSink {
public:
    // Helper to make two input interfaces for the stereomodulator.   Feels like I'm reimplementing a closure here... :-(
    class ModulatorInput : public SampleSink {
    public:
        ModulatorInput(StereoModulator* mod, int channel);
        ~ModulatorInput();
        void consume(float* data, int num);
        int getSampleRate() const;

    private:
        StereoModulator* const mod_;
        const int channel_;

        ModulatorInput(const ModulatorInput&);
        ModulatorInput& operator=(const ModulatorInput&);
    };

    StereoModulator(SampleSink* next);
    ~StereoModulator();
    SampleSink* getChannel(int channel);

    void consume(float* data, int num, int channel);
    int getSampleRate() const;
private:
    float buffer_[1024];
    int bufferOwner_;
    int bufferLen_;
    int state_; // 8 state state machine.
    float sinLut_[16];
    SampleSink* next_;

    StereoModulator(const StereoModulator&);
    StereoModulator& operator=(const StereoModulator&);
};
