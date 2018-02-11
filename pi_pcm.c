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

#include <arpa/inet.h>
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <memory.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "mailbox.h"
#define MBFILE            DEVICE_FILE_NAME    /* From mailbox.h */

/**
 * RC commands are almost always sent at some multiple of 5 us.  The basic idea
 * is to maintain a buffer of 4000 values to write to the clock control
 * register and then arrange for the DMA controller to write the values
 * sequentially at 5us intervals.  The control code can then wake up every 10ms
 * or so and populate the buffer with new samples.  At 5us per sample, a 4000
 * sample buffer will last 20ms, so waking every 10ms should be sufficient.
 *
 * Total memory needed is:
 *
 * The frequencies      4000 * 4
 * CBs to set the frequency 4000 * 32
 * CBs to cause delays      4000 * 32
 *
 * Process can wake every 10ms and update all samples based on where the DMA
 * CB is pointed.
 */

// Make Syntastic happy
#ifndef RASPI
#   ifdef __x86_64__
#       pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
#       define TEST_COMPILATION 1
#   endif
#endif

#if (RASPI)==1
#define PERIPH_VIRT_BASE 0x20000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0x40000000
#define MEM_FLAG 0x0c
#elif (RASPI)==2
#define PERIPH_VIRT_BASE 0x3f000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0xc0000000
#define MEM_FLAG 0x04
#elif (TEST_COMPILATION)==1
#define PERIPH_VIRT_BASE 0x0
#define PERIPH_PHYS_BASE 0x0
#define DRAM_PHYS_BASE 0x0
#define MEM_FLAG 0x0
#else
#error Unknown Raspberry Pi version (variable RASPI)
#endif

#define NUM_SAMPLES     4000
#define NUM_CBS         (NUM_SAMPLES * 2)

#define BCM2708_DMA_NO_WIDE_BURSTS  (1<<26)
#define BCM2708_DMA_WAIT_RESP       (1<<3)
#define BCM2708_DMA_D_DREQ      (1<<6)
#define BCM2708_DMA_PER_MAP(x)      ((x)<<16)
#define BCM2708_DMA_END         (1<<1)
#define BCM2708_DMA_RESET       (1<<31)
#define BCM2708_DMA_INT         (1<<2)
#define BCM2708_DMA_ABORT       (1<<30)  /* Stop current CB, go to next, WO */
#define BCM2708_DMA_ACTIVE      (1<<0)

#define DMA_CS          (0x00/4)
#define DMA_CONBLK_AD       (0x04/4)
#define DMA_DEBUG       (0x20/4)

#define DMA_BASE_OFFSET        0x00007000
#define PWM_BASE_OFFSET        0x0020C000
#define CLK_BASE_OFFSET            0x00101000
#define GPIO_BASE_OFFSET    0x00200000

#define DMA_VIRT_BASE        (PERIPH_VIRT_BASE + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE        (PERIPH_VIRT_BASE + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE        (PERIPH_VIRT_BASE + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE        (PERIPH_VIRT_BASE + GPIO_BASE_OFFSET)

#define PWM_PHYS_BASE        (PERIPH_PHYS_BASE + PWM_BASE_OFFSET)

#define DMA_LEN         0x24
#define PWM_LEN         0x28
#define CLK_LEN         0xA8
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
/**
 * I think this means it requests as soon as there is one free slot in the FIFO
 * which is what we want as burst DMA would mess up our timing..
 */
#define PWMDMAC_THRSHLD     ((15<<8)|(15<<0))

#define GPFSEL0         (0x00/4)

#define BUFFER_SIZE 20000

// This frequency is used for RC cars in the US and the UK
#define DEFAULT_FREQUENCY 27.045

struct dma_cb_t {
    uint32_t info, src, dst, length,
             stride, next, pad[2];
};

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

struct page_map_t {
    uint8_t* virtaddr;
    uint32_t physaddr;
};


struct command_node_t {
    float burst_us;
    float spacing_us;
    int repeats;
    float frequency;
    float dead_frequency;

    struct command_node_t* next;
};

struct page_map_t* page_map;

static struct {
    int handle;            /* From mbox_open() */
    unsigned mem_ref;    /* From mem_alloc() */
    unsigned bus_addr;    /* From mem_lock() */
    uint8_t *virt_addr;    /* From mapmem() */
} mbox;

#ifndef TEST_COMPILATION
static volatile uint32_t* pwm_reg;
static volatile uint32_t* clk_reg;
static volatile uint32_t* dma_reg;
static volatile uint32_t* gpio_reg;
#endif

struct control_data_s {
    struct dma_cb_t cb[NUM_CBS];
    uint32_t sample[NUM_SAMPLES];
};

#define PAGE_SIZE   4096
#define PAGE_SHIFT  12
#define NUM_PAGES   (int)((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

static struct control_data_s* ctl;
static int udp_fd = 0;
static int tcp_fd = 0;

struct pi_options {
    int verbose;
    int port;
};

enum json_token {
    LSB,
    RSB,
    LCB,
    RCB,
    COMMA,
    NUMBER,
    COLON,
    STRING,
    END
};

static struct pi_options get_args(int argc, char* argv[]);
static void print_usage(const char* program);
static void udelay(int us);
static void terminate(int unused);
static void fatal(char* fmt, ...);
static uint32_t mem_virt_to_phys(void* virt);
static uint32_t mem_phys_to_virt(uint32_t phys);
static void* map_peripheral(uint32_t base, uint32_t len);
static int fill_buffer(
    const struct command_node_t* command,
    const struct command_node_t* next_command,
    struct control_data_s* ctl_,
    const volatile uint32_t* dma_reg_);
static int frequency_to_control(float frequency);
static struct dma_cb_t* write_samples(struct dma_cb_t* cbp, float frequency);
static void set_up_signal_handlers(void);
static void set_up_sockets(int port);
static void initialize_dma(void);
static void initialize_mbox(void);
static void serve_forever(int verbose);
static int parse_json(
        const char* json,
        struct command_node_t** new_command);
static const char* get_json_token(
        const char* json,
        enum json_token* token,
        char** id);
static void free_command(struct command_node_t* command);
static int read_from_fd(int fd,
        int verbose,
        struct command_node_t** command);


int main(int argc, char** argv) {
#ifdef TEST_COMPILATION
    printf("Testing on non-Pi hardware, no radio signal will be generated\n");
#endif
    set_up_signal_handlers();
    struct pi_options options = get_args(argc, argv);
    set_up_sockets(options.port);
    initialize_dma();
    serve_forever(options.verbose);
    assert(!"This should not be reachable");
}


static struct dma_cb_t* write_samples(struct dma_cb_t* cbp, const float frequency) {
    int i;
    const int frequency_control = frequency_to_control(frequency);
    uint32_t phys_sample_dst = CM_GP0DIV;
    uint32_t phys_pwm_fifo_addr = PWM_PHYS_BASE + 0x18;

    for (i = 0; i < NUM_SAMPLES; i++) {
        ctl->sample[i] = 0x5a << 24 | frequency_control; /* Silence */
        /* Write a frequency sample */
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
        cbp->src = mem_virt_to_phys(ctl->sample + i);
        cbp->dst = phys_sample_dst;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
        /* Delay */
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
        cbp->src = mem_virt_to_phys(mbox.virt_addr);
        cbp->dst = phys_pwm_fifo_addr;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
    }
    cbp--;
    return cbp;
}


static void set_up_signal_handlers(void) {
    struct sigaction sa;
    /**
     * Catch all signals possible - it is vital we kill the DMA engine
     * on process exit!
     */
    int i;
    for (i = 1; i < 64; i++) {
        /* These are uncatchable or harmless */
        if (
            i != SIGKILL
            && i != SIGSTOP
            && i != SIGVTALRM
            && i != SIGWINCH
            && i != SIGPROF
        ) {
            memset(&sa, 0, sizeof(sa));
            sa.sa_handler = terminate;
            sigaction(i, &sa, NULL);
        }
    }
}


static void initialize_dma(void) {
#ifdef TEST_COMPILATION
    ctl = malloc(sizeof(struct control_data_s));
#else
    dma_reg = map_peripheral(DMA_VIRT_BASE, DMA_LEN);
    pwm_reg = map_peripheral(PWM_VIRT_BASE, PWM_LEN);
    clk_reg = map_peripheral(CLK_VIRT_BASE, CLK_LEN);
    gpio_reg = map_peripheral(GPIO_VIRT_BASE, GPIO_LEN);

    /* GPIO4 needs to be ALT FUNC 0 to otuput the clock */
    gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12);

    /* Program GPCLK to use MASH setting 1, so fractional dividers work */
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 6;
    udelay(100);
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 1 << 9 | 1 << 4 | 6;

    initialize_mbox();
    ctl = (struct control_data_s*)mbox.virt_addr;
    struct dma_cb_t* cbp = ctl->cb;
    cbp = write_samples(cbp, DEFAULT_FREQUENCY);
    cbp->next = mem_virt_to_phys(mbox.virt_addr);

    /**
     * Initialise PWM to use a 100MHz clock too, and set the range to 500 bits,
     * which is 5us, the rate at which we want to update the GPCLK control
     * register.
     */
    pwm_reg[PWM_CTL] = 0;
    udelay(10);
    clk_reg[PWMCLK_CNTL] = 0x5A000006;              /* Source=PLLD and disable */
    udelay(100);
    clk_reg[PWMCLK_DIV] = 0x5A000000 | (5 << 12);  /* set pwm div to 5, for 100MHz */
    udelay(100);
    clk_reg[PWMCLK_CNTL] = 0x5A000016;              /* Source=PLLD and enable */
    udelay(100);
    pwm_reg[PWM_RNG1] = 500;
    udelay(10);
    pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_CLRF;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
    udelay(10);

    /* Initialise the DMA */
    dma_reg[DMA_CS] = BCM2708_DMA_RESET;
    udelay(10);
    dma_reg[DMA_CS] = BCM2708_DMA_INT | BCM2708_DMA_END;
    dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
    dma_reg[DMA_DEBUG] = 7; /* clear debug error flags */
    dma_reg[DMA_CS] = 0x10880001;   /* go, mid priority, wait for outstanding writes */
#endif
}


static void set_up_sockets(const int port) {
    // Socket stuff
    struct sockaddr_in serveraddr;
    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serveraddr.sin_port = htons(port);

    // Set up UDP socket
    if ((udp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
        fatal("Unable to create UDP socket: %s\n", strerror(errno));
    }
    if (bind(udp_fd, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0) {
        fatal("UDP bind failed: %s\n", strerror(errno));
        close(udp_fd);
        exit(1);
    }

    // Set up TCP socket
    if ((tcp_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        fatal("Unable to create socket: %s\n", strerror(errno));
    }
    int optionValue = 1;
    if (setsockopt(
            tcp_fd,
            SOL_SOCKET,
            SO_REUSEADDR,
            (const void *)&optionValue,
            sizeof(optionValue)) != 0) {
        fatal("Unable to set socket options: %s\n", strerror(errno));
    }
    if (bind(tcp_fd, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0) {
        fatal("TCP bind failed: %s\n", strerror(errno));
        close(udp_fd);
        close(tcp_fd);
        exit(1);
    }
    if (listen(tcp_fd, 1) < 0) {
        fatal("Unable to listen on TCP: %s\n", strerror(errno));
    }
}


static void initialize_mbox(void) {
    mbox.handle = mbox_open();
    if (mbox.handle < 0) {
        fatal("Failed to open mailbox. Check kernel support for vcio / BCM2708 mailbox.\n");
    }
    if(! (mbox.mem_ref = mem_alloc(mbox.handle, NUM_PAGES * 4096, 4096, MEM_FLAG))) {
        fatal("Could not allocate memory.\n");
    }
    // TODO: How do we know that succeeded?
    if(! (mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref))) {
        fatal("Could not lock memory.\n");
    }
    if(! (mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), NUM_PAGES * 4096))) {
        fatal("Could not map memory.\n");
    }
}


__attribute__((noreturn)) static void serve_forever(const int verbose) {
    struct command_node_t* new_command = NULL;

    /* Calculate the frequency control word */
    /* The fractional part is stored in the lower 12 bits */

    struct command_node_t* command = malloc(sizeof(*command));
    command->burst_us = 100.0f;
    command->spacing_us = 100.0f;
    command->repeats = 1;
    command->frequency = DEFAULT_FREQUENCY;
    command->dead_frequency = DEFAULT_FREQUENCY;
    command->next = NULL;

    // Set up select
    fd_set read_fds, master;
    FD_ZERO(&master);
    FD_SET(tcp_fd, &master);
    FD_SET(udp_fd, &master);

    int max_fd = tcp_fd > udp_fd
      ? tcp_fd
      : udp_fd;

    struct timeval immediate;
    immediate.tv_sec = 0;
    immediate.tv_usec = 0;
    while (1) {
        read_fds = master;
        const int nfds_waiting = select(max_fd + 1, &read_fds, NULL, NULL, &immediate);
        if (nfds_waiting < 0) {
            fatal("Select failed: %s\n", strerror(errno));
        }
        if (nfds_waiting > 0) {
            int fd;
            for (fd = 0; fd < max_fd + 1; ++fd) {
                if (FD_ISSET(fd, &read_fds)) {
                    if (fd == tcp_fd) {
                        // New connection
                        int new_fd;
                        if ((new_fd = accept(tcp_fd, NULL, NULL)) < 0) {
                            fatal(
                                "Unable to accept TCP connection: %s\n",
                                strerror(errno));
                        }
                        FD_SET(new_fd, &master);
                        if (new_fd > max_fd) {
                            max_fd = new_fd;
                        }
                    } else {
                        // Read message
                        if (read_from_fd(fd, verbose, &new_command) <= 0) {
                            close(fd);
                            FD_CLR(fd, &master);
                            while (!FD_ISSET(max_fd, &master)) {
                                --max_fd;
                            }
                        }
                    }
                }
            }
        }

#ifdef TEST_COMPILATION
        const int used = 1;
#else
        const int used = fill_buffer(command, new_command, ctl, dma_reg);
#endif
        if (used > 0 && new_command != NULL) {
            free_command(command);
            command = new_command;
            new_command = NULL;
        }

        usleep(10000);
    }
}


static void udelay(int us) {
    struct timespec ts = { 0, us * 1000 };
    nanosleep(&ts, NULL);
}


static void terminate(const int signal_) {
    printf("Terminating with signal %d\n", signal_);
#ifndef TEST_COMPILATION
    if (dma_reg) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(500);
        /* Abort the whole DMA */
        dma_reg[DMA_CS] = BCM2708_DMA_ABORT | BCM2708_DMA_ACTIVE;
        udelay(500);
    }
#endif
    if (tcp_fd != 0) {
        close(tcp_fd);
    }
    if (udp_fd != 0) {
        close(udp_fd);
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
    uint32_t offset = (uint8_t*)virt - mbox.virt_addr;
    return mbox.bus_addr + offset;
}


static uint32_t mem_phys_to_virt(uint32_t phys) {
    return phys - (uint32_t)mbox.bus_addr + (uint32_t)mbox.virt_addr;
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
    const struct command_node_t* const command,
    const struct command_node_t* const next_command,
    struct control_data_s* const ctl_,
    const volatile uint32_t* const dma_reg_
) {
    assert(command != NULL || next_command != NULL);
    int nodes_processed = 0;

    static enum {
        BURST,
        SPACING,
    } state = BURST;

    static float time_us = 0.0f;

    static int repeat_count = -1;
    static const struct command_node_t* current_command;
    if (repeat_count == -1) {
        repeat_count = command->repeats;
        current_command = command;
    }
    static int frequency_control;

    static uint32_t last_cb = 0;
    if (last_cb == 0) {
        last_cb = (uint32_t)ctl->cb;
    }
    const uint32_t cur_cb = mem_phys_to_virt(dma_reg_[DMA_CONBLK_AD]);
    int last_sample = (last_cb - (uint32_t)mbox.virt_addr) / (sizeof(struct dma_cb_t) * 2);
    const int this_sample = (cur_cb - (uint32_t)mbox.virt_addr) / (sizeof(struct dma_cb_t) * 2);
    int free_slots = this_sample - last_sample;

    if (free_slots < 0) {
        free_slots += NUM_SAMPLES;
    }

    for (; free_slots >= 0; --free_slots) {
        /* From DMA settings above, each sample is 5us */
        time_us -= 5.0f;

        if (time_us <= 0.0f) {
            switch (state) {
                case BURST:
                    time_us += current_command->spacing_us;
                    state = SPACING;
                    frequency_control = frequency_to_control(current_command->dead_frequency);
                    break;
                case SPACING:
                    --repeat_count;
                    if (repeat_count == 0) {
                        current_command = current_command->next;
                        if (current_command == NULL) {
                            if (next_command != NULL) {
                                current_command = next_command;
                            } else {
                                current_command = command;
                            }
                            ++nodes_processed;
                        }
                        time_us += current_command->burst_us;
                        state = BURST;
                        repeat_count = current_command->repeats;
                    } else {
                        time_us += current_command->burst_us;
                        state = BURST;
                    }
                    frequency_control = frequency_to_control(current_command->frequency);
                    break;
                default:
                    assert(0 && "Unknown state");
            }
        }
        assert(time_us >= 0.0f && "Time should be positive");

        ctl_->sample[last_sample++] = (0x5A << 24 | frequency_control);
        if (last_sample == NUM_SAMPLES) {
            last_sample = 0;
        }
    }
    last_cb = (uint32_t)mbox.virt_addr + last_sample * sizeof(struct dma_cb_t) * 2;
    return nodes_processed;
}


static int frequency_to_control(const float frequency) {
    const int PLLFREQ = 500000000;  /* PLLD is running at 500MHz */
    const float PLL_MHZ = (float)PLLFREQ / 1000000.0f;
    const float poll_per_carrier = PLL_MHZ / frequency;
    const int frequency_control = (int)round(poll_per_carrier * (1 << 12));
    return frequency_control;
}


static int parse_json(
    const char* json,
    struct command_node_t** command
) {
    assert(*command == NULL);

    enum json_token token;
    char* id;
    struct command_node_t* parsed_command_ptr = NULL;
    struct command_node_t* root_ptr = NULL;
    float* property;
    int set_properties_bitflag;

    enum {
        BEGIN,
        OBJECT_OR_END_ARRAY,
        PROPERTY_OR_END_OBJECT
    } state;
    state = BEGIN;

    int ntokens = 0;
    const int ntokens_limit = 1000;
    // Failsafe
    while (ntokens < ntokens_limit) {
        json = get_json_token(json, &token, &id);
        ++ntokens;

        if (token == END) {
            fprintf(stderr, "Unexpected end of JSON\n");
            goto CLEANUP;
        }

        switch (state) {
        case BEGIN:
            if (token != LSB) {
                fprintf(stderr, "Root element should be a JSON array\n");
                goto CLEANUP;
            }
            state = OBJECT_OR_END_ARRAY;
            break;

        case OBJECT_OR_END_ARRAY:
            // End array
            if (token == RSB) {
                if (root_ptr == NULL)  {
                    fprintf(stderr, "Empty array\n");
                    goto CLEANUP;
                }
                goto DONE;
            } else if (token == COMMA) {
                // For ease of implementation, we're going to allow multiple
                // leading and trailing commas. I know it's invalid JSON, but
                // I'm not too worried about it.
                break;
            } else if (token == LCB) {
                // Object
                if (root_ptr == NULL) {
                    parsed_command_ptr = malloc(sizeof(struct command_node_t));
                    root_ptr = parsed_command_ptr;
                } else {
                    parsed_command_ptr->next = malloc(sizeof(struct command_node_t));
                    parsed_command_ptr = parsed_command_ptr->next;
                }
                parsed_command_ptr->next = NULL;
                set_properties_bitflag = 0;
                state = PROPERTY_OR_END_OBJECT;
            } else {
                fprintf(stderr, "Invalid JSON or invalid command\n");
                goto CLEANUP;
            }
            break;

        case PROPERTY_OR_END_OBJECT:
            // End object
            if (token == RCB) {
                if (set_properties_bitflag != 0x1f) {
                    fprintf(stderr, "Object is missing required properties\n");
                    goto CLEANUP;
                }
                state = OBJECT_OR_END_ARRAY;
                break;
            } else if (token == COMMA) {
                // For ease of implementation, we're going to allow multiple
                // leading and trailing commas. I know it's invalid JSON, but
                // I'm not too worried about it.
                break;
            } else if (token == STRING) {
                // Object
                property = NULL;
                if (strcmp("burst_us", id) == 0) {
                    property = &(parsed_command_ptr->burst_us);
                    set_properties_bitflag |= 0x1;
                }
                else if (strcmp("spacing_us", id) == 0) {
                    property = &(parsed_command_ptr->spacing_us);
                    set_properties_bitflag |= 0x2;
                }
                else if (strcmp("frequency", id) == 0) {
                    property = &(parsed_command_ptr->frequency);
                    set_properties_bitflag |= 0x4;
                }
                else if (strcmp("dead_frequency", id) == 0) {
                    property = &(parsed_command_ptr->dead_frequency);
                    set_properties_bitflag |= 0x8;
                }
                else if (strcmp("repeats", id) == 0) {
                    property = NULL;  // Special case, parse as int
                    set_properties_bitflag |= 0x10;
                }
                else {
                    fprintf(stderr, "Unrecognized property\n");
                    goto CLEANUP;
                }

                json = get_json_token(json, &token, &id);
                if (token != COLON) {
                    fprintf(stderr, "Invalid JSON\n");
                    goto CLEANUP;
                }

                json = get_json_token(json, &token, &id);
                if (token != NUMBER) {
                    fprintf(stderr, "Invalid JSON\n");
                    goto CLEANUP;
                }

                if (property) {
                    *property = atof(id);
                } else {
                    parsed_command_ptr->repeats = atoi(id);
                }

                // Stay on PROPERTY_OR_END_OBJECT
            }
            break;
        default:
            assert(!"Unexpected state");
            return 1;
        }
    }
DONE:
    if (ntokens == ntokens_limit) {
        assert(!"Too many tokens");
        goto CLEANUP;
    }
    *command = root_ptr;
    return 0;

CLEANUP:
    free_command(root_ptr);
    return 1;
}


static const char* get_json_token(
        const char* json,
        enum json_token* token,
        char** id
) {
    static char identifier[20];
    size_t index = 0;
    const char* iter = json;
    while (isspace(*iter)) {
        ++iter;
    }
    switch (*iter) {
    case '\0':
        *token = END;
        return iter;

    case '{':
        *token = LCB;
        return iter + 1;
    case '}':
        *token = RCB;
        return iter + 1;
    case '[':
        *token = LSB;
        return iter + 1;
    case ']':
        *token = RSB;
        return iter + 1;
    case ':':
        *token = COLON;
        return iter + 1;
    case ',':
        *token = COMMA;
        return iter + 1;

    case '"':
        ++iter;
        while (*iter != '"' && *iter != '\0') {
            identifier[index++] = *iter++;
            if (index >= sizeof(identifier) / sizeof(identifier[0]) - 1) {
                break;
            }
        }
        *token = STRING;
        identifier[index] = '\0';
        *id = identifier;
        return iter + 1;

    default: break;
    }
    if (isdigit(*iter)) {
        while (isdigit(*iter) || *iter == '.') {
            identifier[index++] = *iter++;
            if (index >= sizeof(identifier) / sizeof(identifier[0]) - 1) {
                break;
            }
        }
        *token = NUMBER;
        identifier[index] = '\0';
        *id = identifier;
        return iter;
    }

    assert(!"This should not be reachable");
    *token = END;
    return json;
}


static void free_command(struct command_node_t* command) {
    while (command) {
        struct command_node_t* const previous = command;
        command = command->next;
        free(previous);
    }
}


static int read_from_fd(
        const int fd,
        const int verbose,
        struct command_node_t** command
) {
    static char message_buffer[BUFFER_SIZE];

    const int bytes_count = read(fd, message_buffer, BUFFER_SIZE);
    if (bytes_count < 0) {
        fatal("Error reading from file descriptor: %s\n", strerror(errno));
    } else if (bytes_count == 0) {
        // End of file, I guess?
        return -1;
    }

    /**
     * If the received message is too long, then it will be truncated.
     * All valid messages should always fit in the buffer, so just ignore it
     * if it's too long.
     */
    if (
        bytes_count > 0
        && (size_t)bytes_count < sizeof(message_buffer) / sizeof(message_buffer[0])
    ) {
        message_buffer[bytes_count] = '\0';
        struct command_node_t* parsed_command = NULL;
        char* json_data = message_buffer;

        if (verbose) {
            printf("Received message \"%s\"\n", json_data);
        }
        const int parse_status = parse_json(json_data, &parsed_command);

        if (parse_status == 0) {
            /**
             * Command bursts come after synchronization bursts, and
             * they're the interesting part, so print those
             */
            if (verbose) {
                const struct command_node_t* const print_command = (
                    parsed_command->next == NULL
                    ? parsed_command
                    : parsed_command->next);
                printf(
                    "Sending command %d %d:%d bursts @ %4.3f (%4.3f)\n",
                    print_command->repeats,
                    (int)print_command->burst_us,
                    (int)print_command->spacing_us,
                    print_command->frequency,
                    print_command->dead_frequency);
            }

            free_command(*command);
            *command = parsed_command;
        }
    }
    return bytes_count;
}


static struct pi_options get_args(const int argc, char* argv[]) {
    const struct option long_options[] = {
        {"help", no_argument, NULL, 'h'},
        {"verbose", no_argument, NULL, 'v'},
        {"tcp", no_argument, NULL, 't'},
        {"udp", no_argument, NULL, 'u'},
        {"port", required_argument, NULL, 'p'},
        {0, 0, 0, 0}
    };
    struct pi_options options;
    options.port = 12345;
    options.verbose = 0;
    int opt;
    int long_index = 0;
    while (1) {
        opt = getopt_long(argc, argv, "hvp:tu", long_options, &long_index);
        if (opt == -1) {
            break;
        }
        switch (opt) {
            case 'h':
                print_usage(argv[0]);
                exit(EXIT_SUCCESS);
                break;
            case 'v':
                options.verbose = 1;
                break;
            case 'p':
                options.port = atoi(optarg);
                break;
            default:
                fprintf(stderr, "Unknown option\n");
                print_usage(argv[0]);
                exit(EXIT_FAILURE);
        }
    }
    return options;
}


static void print_usage(const char* program) {
    printf("Usage: %s [OPTIONS]\n", program);
    printf("-p, --port     The port to listen for messags on.\n");
    printf("-h, --help     Print this help message.\n");
    printf("-v, --verbose  Print more debugging information.\n");
}
