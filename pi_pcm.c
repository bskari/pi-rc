/*
 * RaspberryPi based FM transmitter.  For the original idea, see:
 *
 * http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
 *
 * All credit to Oliver Mattos and Oskar Weigl for creating the original code.
 *
 * I have taken their idea and reworked it to use the Pi DMA engine, so
 * reducing the CPU overhead for playing a .wav file from 100% to about 1.6%.
 *
 * I have implemented this in user space, using an idea I picked up from Joan
 * on the Raspberry Pi forums - credit to Joan for the DMA from user space
 * idea.
 *
 * The idea of feeding the PWM FIFO in order to pace DMA control blocks comes
 * from ServoBlaster, and I take credit for that :-)
 *
 * This code uses DMA channel 0 and the PWM hardware, with no regard for
 * whether something else might be trying to use it at the same time (such as
 * the 3.5mm jack audio driver).
 *
 * I know nothing much about sound, subsampling, or FM broadcasting, so it is
 * quite likely the sound quality produced by this code can be improved by
 * someone who knows what they are doing.  There may be issues realting to
 * caching, as the user space process just writes to its virtual address space,
 * and expects the DMA controller to see the data; it seems to work for me
 * though.
 *
 * NOTE: THIS CODE MAY WELL CRASH YOUR PI, TRASH YOUR FILE SYSTEMS, AND
 * POTENTIALLY EVEN DAMAGE YOUR HARDWARE.  THIS IS BECAUSE IT STARTS UP THE DMA
 * CONTROLLER USING MEMORY OWNED BY A USER PROCESS.  IF THAT USER PROCESS EXITS
 * WITHOUT STOPPING THE DMA CONTROLLER, ALL HELL COULD BREAK LOOSE AS THE
 * MEMORY GETS REALLOCATED TO OTHER PROCESSES WHILE THE DMA CONTROLLER IS STILL
 * USING IT.  I HAVE ATTEMPTED TO MINIMISE ANY RISK BY CATCHING SIGNALS AND
 * RESETTING THE DMA CONTROLLER BEFORE EXITING, BUT YOU HAVE BEEN WARNED.  I
 * ACCEPT NO LIABILITY OR RESPONSIBILITY FOR ANYTHING THAT HAPPENS AS A RESULT
 * OF YOU RUNNING THIS CODE.  IF IT BREAKS, YOU GET TO KEEP ALL THE PIECES.
 *
 * NOTE ALSO:  THIS MAY BE ILLEGAL IN YOUR COUNTRY.  HERE ARE SOME COMMENTS
 * FROM MORE KNOWLEDGEABLE PEOPLE ON THE FORUM:
 *
 * "Just be aware that in some countries FM broadcast and especially long
 * distance FM broadcast could get yourself into trouble with the law, stray FM
 * broadcasts over Airband aviation is also strictly forbidden."
 *
 * "A low pass filter is really really required for this as it has strong
 * harmonics at the 3rd, 5th 7th and 9th which sit in licensed and rather
 * essential bands, ie GSM, HAM, emergency services and others. Polluting these
 * frequencies is immoral and dangerous, whereas "breaking in" on FM bands is
 * just plain illegal."
 *
 * "Don't get caught, this GPIO use has the potential to exceed the legal
 * limits by about 2000% with a proper aerial."
 *
 *
 * As for the original code, this code is released under the GPL.
 *
 * Richard Hirst <richardghirst@gmail.com>  December 2012
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

// The .wav file is mono at 22050Hz, which means we have a new sample every
// 45.4us.  We want to adjust the 100MHz core frequency at 10 times that so as
// to provide some level of subsampling to improve quality.  The basic idea is
// to maintain a buffer of 4000 values to write to the clock control register
// and then arrange for the DMA controller to write the values sequentially at
// 4.54us intervals.  The control code can then wake up every 10ms or so and
// populate the buffer with new samples.  At 4.54us per sample, a 4000 sample
// buffer will last a bit over 18ms, so waking every 10ms should be sufficient.
//
// Total memory needed is:
//
// The frequencies      4000 * 4
// CBs to set the frequency 4000 * 32
// CBs to cause delays      4000 * 32
//
// Process can wake every 10ms and update all samples based on where the DMA
// CB is pointed.

#define NUM_SAMPLES     4000
#define NUM_CBS         (NUM_SAMPLES * 2)

#define BCM2708_DMA_NO_WIDE_BURSTS  (1<<26)
#define BCM2708_DMA_WAIT_RESP       (1<<3)
#define BCM2708_DMA_D_DREQ      (1<<6)
#define BCM2708_DMA_PER_MAP(x)      ((x)<<16)
#define BCM2708_DMA_END         (1<<1)
#define BCM2708_DMA_RESET       (1<<31)
#define BCM2708_DMA_INT         (1<<2)

#define DMA_CS          (0x00/4)
#define DMA_CONBLK_AD       (0x04/4)
#define DMA_DEBUG       (0x20/4)

#define DMA_BASE        0x20007000
#define DMA_LEN         0x24
#define PWM_BASE        0x2020C000
#define PWM_LEN         0x28
#define CLK_BASE            0x20101000
#define CLK_LEN         0xA8
#define GPIO_BASE       0x20200000
#define GPIO_LEN        0xB4

#define PWM_CTL         (0x00/4)
#define PWM_DMAC        (0x08/4)
#define PWM_RNG1        (0x10/4)
#define PWM_FIFO        (0x18/4)

#define PWMCLK_CNTL     40
#define PWMCLK_DIV      41

#define GPCLK_CNTL      (0x70/4)
#define GPCLK_DIV       (0x74/4)

#define CM_GP0DIV (0x7e101074)

#define PWMCTL_MODE1        (1<<1)
#define PWMCTL_PWEN1        (1<<0)
#define PWMCTL_CLRF     (1<<6)
#define PWMCTL_USEF1        (1<<5)

#define PWMDMAC_ENAB        (1<<31)
// I think this means it requests as soon as there is one free slot in the FIFO
// which is what we want as burst DMA would mess up our timing..
#define PWMDMAC_THRSHLD     ((15<<8)|(15<<0))

#define GPFSEL0         (0x00/4)

// The deviation specifies how wide the signal is. Use 25.0 for WBFM
// (broadcast radio) and about 3.5 for NBFM (walkie-talkie style radio)
#define DEVIATION       25.0

typedef struct {
    uint32_t info, src, dst, length,
             stride, next, pad[2];
} dma_cb_t;

typedef struct {
    uint8_t* virtaddr;
    uint32_t physaddr;
} page_map_t;

struct command_node {
    struct command_node* next;
    // I could specify smaller datatypes for these... but who cares about
    // saving a few bytes
    float synchronization_burst_us;
    float synchronization_spacing_us;
    int total_synchronizations;
    float signal_burst_us;
    float signal_spacing_us;
    int total_signals;
    int frequency_control;
    int dead_frequency_control;
    float frequency;
};

page_map_t* page_map;

static uint8_t* virtbase;

static volatile uint32_t* pwm_reg;
static volatile uint32_t* clk_reg;
static volatile uint32_t* dma_reg;
static volatile uint32_t* gpio_reg;

struct control_data_s {
    dma_cb_t cb[NUM_CBS];
    uint32_t sample[NUM_SAMPLES];
};

#define PAGE_SIZE   4096
#define PAGE_SHIFT  12
#define NUM_PAGES   (int)((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

static struct control_data_s* ctl;

static void udelay(int us);
static void terminate(int unused);
static void fatal(char* fmt, ...);
static uint32_t mem_virt_to_phys(void* virt);
static uint32_t mem_phys_to_virt(uint32_t phys);
static void* map_peripheral(uint32_t base, uint32_t len);
static int fill_buffer(
    struct command_node** command,
    struct control_data_s* ctl_,
    const volatile uint32_t* dma_reg_
);
static int frequency_to_control(float frequency);
static struct command_node* allocate_command_node(
    float synchronization_burst_us,
    float synchronization_spacing_us,
    int total_synchronizations,
    float signal_burst_us,
    float signal_spacing_us,
    int total_signals,
    float frequency
);


int main(int argc, char** argv) {
    int i, mem_fd, pid;
    char pagemap_fn[64];

    // Catch all signals possible - it is vital we kill the DMA engine
    // on process exit!
    for (i = 0; i < 64; i++) {
        struct sigaction sa;

        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = terminate;
        sigaction(i, &sa, NULL);
    }

    // Calculate the frequency control word
    // The fractional part is stored in the lower 12 bits
    float frequency;
    if (argc > 1) {
        frequency = atof(argv[1]);
        if (frequency > 250.0f || frequency < 1.0f) {
            frequency = 100.0f;
        }
    } else {
        frequency = 100.0f;
    }
    printf("Broadcasting at %0.1f\n", frequency);

    dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
    pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
    clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
    gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

    virtbase = mmap(NULL, NUM_PAGES * PAGE_SIZE, PROT_READ | PROT_WRITE,
                    MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED,
                    -1, 0);
    if (virtbase == MAP_FAILED) {
        fatal("Failed to mmap physical pages: %m\n");
    }
    if ((unsigned long)virtbase & (PAGE_SIZE - 1)) {
        fatal("Virtual address is not page aligned\n");
    }
    printf("Virtual memory mapped at %p\n", virtbase);
    page_map = malloc(NUM_PAGES * sizeof(*page_map));
    if (page_map == 0) {
        fatal("Failed to malloc page_map: %m\n");
    }
    pid = getpid();
    snprintf(
        pagemap_fn,
        sizeof(pagemap_fn) / sizeof(pagemap_fn[0]),
        "/proc/%d/pagemap",
        pid
    );
    mem_fd = open(pagemap_fn, O_RDONLY);
    if (mem_fd < 0) {
        fatal("Failed to open %s: %m\n", pagemap_fn);
    }
    if (
        lseek(mem_fd, (unsigned long)virtbase >> 9, SEEK_SET)
        != (off_t)((unsigned long)virtbase >> 9)
    ) {
        fatal("Failed to seek on %s: %m\n", pagemap_fn);
    }
    for (i = 0; i < NUM_PAGES; i++) {
        uint64_t pfn;
        page_map[i].virtaddr = virtbase + i * PAGE_SIZE;
        // Following line forces page to be allocated
        page_map[i].virtaddr[0] = 0;
        if (read(mem_fd, &pfn, sizeof(pfn)) != sizeof(pfn)) {
            fatal("Failed to read %s: %m\n", pagemap_fn);
        }
        if (((pfn >> 55) & 0xfbf) != 0x10c) { // pagemap bits: https://www.kernel.org/doc/Documentation/vm/pagemap.txt
            fatal("Page %d not present (pfn 0x%016llx)\n", i, pfn);
        }
        page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
    }

    // GPIO4 needs to be ALT FUNC 0 to otuput the clock
    gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12);

    // Program GPCLK to use MASH setting 1, so fractional dividers work
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 6;
    udelay(100);
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 1 << 9 | 1 << 4 | 6;

    ctl = (struct control_data_s*)virtbase;
    dma_cb_t* cbp = ctl->cb;
    uint32_t phys_sample_dst = CM_GP0DIV;
    uint32_t phys_pwm_fifo_addr = 0x7e20c000 + 0x18;

    const int frequency_control = frequency_to_control(frequency);
    for (i = 0; i < NUM_SAMPLES; i++) {
        ctl->sample[i] = 0x5a << 24 | frequency_control; // Silence
        // Write a frequency sample
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
        cbp->src = mem_virt_to_phys(ctl->sample + i);
        cbp->dst = phys_sample_dst;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
        // Delay
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
        cbp->src = mem_virt_to_phys(virtbase);
        cbp->dst = phys_pwm_fifo_addr;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
    }
    cbp--;
    cbp->next = mem_virt_to_phys(virtbase);

    // Initialise PWM to use a 100MHz clock too, and set the range to
    // 454 bits, which is 4.54us, the rate at which we want to update
    // the GPCLK control register.
    pwm_reg[PWM_CTL] = 0;
    udelay(10);
    clk_reg[PWMCLK_CNTL] = 0x5A000006;              // Source=PLLD and disable
    udelay(100);
    clk_reg[PWMCLK_DIV] = 0x5A000000 | (5 << 12);  // set pwm div to 5, for 100MHz
    udelay(100);
    clk_reg[PWMCLK_CNTL] = 0x5A000016;              // Source=PLLD and enable
    udelay(100);
    pwm_reg[PWM_RNG1] = 454;
    udelay(10);
    pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_CLRF;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
    udelay(10);

    // Initialise the DMA
    dma_reg[DMA_CS] = BCM2708_DMA_RESET;
    udelay(10);
    dma_reg[DMA_CS] = BCM2708_DMA_INT | BCM2708_DMA_END;
    dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
    printf("virt %x\n", mem_virt_to_phys(ctl->cb));
    dma_reg[DMA_DEBUG] = 7; // clear debug error flags
    dma_reg[DMA_CS] = 0x10880001;   // go, mid priority, wait for outstanding writes

    struct command_node* root;
    root = allocate_command_node(2100.0f, 700.0f, 4, 700.0f, 700.0f, 53, 100.0f);
    root->next = NULL;
    struct command_node** tail = &(root->next);
    // Fill the buffer with some initial commands
    for (i = 0; i < 10; ++i) {
        // 88.5 is NPR in Boulder, which is easy to find on a radio
        const float test_frequency = 88.5f;
        // For testing, broadcast at a frequency for 0.5 seconds
        *tail = allocate_command_node(500.0f, 1.0f, 1000, 1.0f, 1.0f, 1, test_frequency);
        tail = &((*tail)->next);
    }

    float test_frequency;
    float us;
    int synchronization_burst_count;
    int synchronization_burst_length_multiplier;
    int command_burst_count;
    // Toy RC car frequencies are 49.830, 49.845, 49.860, 49.875, 49.890
    for (test_frequency = 49.780f; test_frequency <= 49.930f; test_frequency += 0.01f) {
        for (us = 200.0f; us < 1000.0f + 1.0f; us += 50.0f) {
            for (synchronization_burst_count = 2; synchronization_burst_count <= 6; ++synchronization_burst_count) {
                for (synchronization_burst_length_multiplier = 2; synchronization_burst_length_multiplier <= 5; ++synchronization_burst_length_multiplier) {
                    for (command_burst_count = 10; command_burst_count <= 100; ++command_burst_count) {
                        printf(
                            "Adding %d %4.1f:%4.1f sync bursts, %d %4.1f:%4.1f signal bursts @ %4.5f\n",
                            synchronization_burst_count,
                            synchronization_burst_length_multiplier * us,
                            us,
                            command_burst_count,
                            us,
                            us,
                            test_frequency
                        );
                        // Each command should run for at least 1/2 second
                        const float us_per_synchronization = us * (1 + synchronization_burst_length_multiplier) * synchronization_burst_count;
                        const float us_per_burst = us * (2 * command_burst_count);
                        const int limit = (int)((1000000.0f * 0.5f) / (us_per_synchronization + us_per_burst));
                        for (i = 0; i < limit;) { // We'll increment i when a node is removed
                            usleep(10000);
                            // We need to keep refilling the command buffer
                            const int nodes_removed = fill_buffer(&root, ctl, dma_reg);
                            i += nodes_removed;
                            int j;
                            for (j = 0; j < nodes_removed; ++j) {
                                *tail = allocate_command_node(
                                    us * synchronization_burst_length_multiplier,
                                    us,
                                    synchronization_burst_count,
                                    us,
                                    us,
                                    command_burst_count,
                                    test_frequency
                                );
                                tail = &((*tail)->next);
                            }
                        }
                    }
                }
            }
        }
    }

    terminate(0);

    return 0;
}


static void udelay(int us) {
    struct timespec ts = { 0, us * 1000 };
    nanosleep(&ts, NULL);
}


static void terminate(const int signal_) {
    printf("Terminating with signal %d\n", signal_);
    if (dma_reg) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(500);
    }
    exit(1);
}


static void fatal(char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    terminate(0);
}


static uint32_t mem_virt_to_phys(void* virt) {
    uint32_t offset = (uint8_t*)virt - virtbase;

    return page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE);
}


static uint32_t mem_phys_to_virt(uint32_t phys) {
    uint32_t pg_offset = phys & (PAGE_SIZE - 1);
    uint32_t pg_addr = phys - pg_offset;
    int i;

    for (i = 0; i < NUM_PAGES; i++) {
        if (page_map[i].physaddr == pg_addr) {
            return (uint32_t)virtbase + i * PAGE_SIZE + pg_offset;
        }
    }
    fatal("Failed to reverse map phys addr %08x\n", phys);

    return 0;
}



static void* map_peripheral(uint32_t base, uint32_t len) {
    int fd = open("/dev/mem", O_RDWR);
    void* vaddr;

    if (fd < 0) {
        fatal("Failed to open /dev/mem: %m\n");
    }
    vaddr = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED) {
        fatal("Failed to map peripheral at 0x%08x: %m\n", base);
    }
    close(fd);

    return vaddr;
}


int fill_buffer(
    struct command_node** command_list,
    struct control_data_s* ctl_,
    const volatile uint32_t* const dma_reg_
) {
    int nodes_processed = 0;
    assert(command_list != NULL);
    struct command_node* first_command = *command_list;
    assert(first_command != NULL);
    if (first_command == NULL) {
        // Uh... Let's just make something up
        *command_list = allocate_command_node(2100.0f, 700.0f, 4, 700.0f, 700.0f, 53, 100.0f);
    }

    static enum {
        SYNCHRONIZATION_BURST,
        SYNCHRONIZATION_SPACING,
        SIGNAL_BURST,
        SIGNAL_SPACING
    } state = SYNCHRONIZATION_BURST;

    static float time_us = 0.0f;
    static int synchronization_count = -1;
    if (synchronization_count == -1) {
        synchronization_count = first_command->total_synchronizations;
    }
    static int signal_count = -1;
    if (signal_count == -1) {
        signal_count = first_command->total_signals;
    }

    static uint32_t last_cb = 0;
    if (last_cb == 0) {
        last_cb = (uint32_t)ctl->cb;
    }
    const uint32_t cur_cb = mem_phys_to_virt(dma_reg_[DMA_CONBLK_AD]);
    int last_sample = (last_cb - (uint32_t)virtbase) / (sizeof(dma_cb_t) * 2);
    const int this_sample = (cur_cb - (uint32_t)virtbase) / (sizeof(dma_cb_t) * 2);
    int free_slots = this_sample - last_sample;

    if (free_slots < 0) {
        free_slots += NUM_SAMPLES;
    }

    while (free_slots >= 10) {
        // I have no idea if this subtraction is right
        // 44,300 is the bit rate of wav files
        time_us -= 44.300f;

        if (time_us <= 0.0f) {
            switch (state) {
                case SYNCHRONIZATION_BURST:
                    time_us = first_command->synchronization_spacing_us;
                    state = SYNCHRONIZATION_SPACING;
                    break;
                case SYNCHRONIZATION_SPACING:
                    time_us = first_command->synchronization_burst_us;
                    --synchronization_count;
                    if (synchronization_count == 0) {
                        synchronization_count = first_command->total_synchronizations;
                        state = SIGNAL_BURST;
                    } else {
                        state = SYNCHRONIZATION_BURST;
                    }
                    break;
                case SIGNAL_BURST:
                    time_us = first_command->signal_spacing_us;
                    state = SIGNAL_SPACING;
                    break;
                case SIGNAL_SPACING:
                    time_us = first_command->signal_burst_us;
                    --signal_count;
                    if (signal_count == 0) {
                        // Process the next command
                        *command_list = first_command->next;
                        free(first_command);
                        first_command = *command_list;
                        ++nodes_processed;
                        signal_count = first_command->total_signals;
                        state = SYNCHRONIZATION_BURST;
                    } else {
                        state = SIGNAL_BURST;
                    }
                    break;
                default:
                    assert(0 && "Unknown state");
            }
        }
        assert(time_us > 0.0f && "Time should be positive");

        // In the original FM radio code, dval would be loaded from a wav file.
        // For the RC car, I don't know what to do.
        const float dval = 0.0f;
        const int intval = (int)(floor(dval));
        const int frac = (int)((dval - (float)intval) * 10.0);
        int j;

        int frequency_control;
        switch (state) {
            case SYNCHRONIZATION_BURST:
            case SIGNAL_BURST:
                frequency_control = first_command->frequency_control;
                break;
            case SYNCHRONIZATION_SPACING:
            case SIGNAL_SPACING:
                frequency_control = first_command->dead_frequency_control;
                break;
            default:
                assert(0 && "Unknown state when looking for frequency control");
                frequency_control = first_command->frequency_control;
        }
        // I'm sure this code could do a better job of subsampling, either by
        // distributing the '+1's evenly across the 10 subsamples, or maybe
        // by taking the previous and next samples in to account too.
        for (j = 0; j < 10; j++) {
            ctl_->sample[last_sample++] = (0x5A << 24 | frequency_control) + (frac > j ? intval + 1 : intval);
            if (last_sample == NUM_SAMPLES) {
                last_sample = 0;
            }
        }
        free_slots -= 10;
    }
    last_cb = (uint32_t)virtbase + last_sample * sizeof(dma_cb_t) * 2;
    return nodes_processed;
}


static int frequency_to_control(const float frequency) {
    const int PLLFREQ = 500000000;  // PLLD is running at 500MHz
    const float PLL_MHZ = (float)PLLFREQ / 1000000.0f;
    const float poll_per_carrier = PLL_MHZ / frequency;
    const int frequency_control = (int)round(poll_per_carrier * (1 << 12));
    return frequency_control;
}


struct command_node* allocate_command_node(
    const float synchronization_burst_us,
    const float synchronization_spacing_us,
    const int total_synchronizations,
    const float signal_burst_us,
    const float signal_spacing_us,
    const int total_signals,
    const float frequency
) {
    struct command_node* const new_node = malloc(sizeof(*new_node));
    new_node->next = NULL;
    new_node->synchronization_burst_us = synchronization_burst_us;
    new_node->synchronization_spacing_us = synchronization_spacing_us;
    new_node->total_synchronizations = total_synchronizations;
    new_node->signal_burst_us = signal_burst_us;
    new_node->signal_spacing_us = signal_spacing_us;
    new_node->total_signals = total_signals;
    new_node->frequency_control = frequency_to_control(frequency);
    // RC toys in the 49 MHz range run 49.830 - 49.890, depending on the
    // channel, so subtract 1 MHz for the dead frequency so that we're still
    // roughly in the range and hopefully not stomping on anything important.
    // We don't want to add 1 MHz because the 50 MHz range is reserved for
    // people with ham radio licenses.
    new_node->dead_frequency_control = frequency_to_control(frequency - 1.0f);
    new_node->frequency = frequency;
    return new_node;
}
