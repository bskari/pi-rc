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
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <jansson.h>
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

static volatile uint32_t* pwm_reg;
static volatile uint32_t* clk_reg;
static volatile uint32_t* dma_reg;
static volatile uint32_t* gpio_reg;

struct control_data_s {
    struct dma_cb_t cb[NUM_CBS];
    uint32_t sample[NUM_SAMPLES];
};

#define PAGE_SIZE   4096
#define PAGE_SHIFT  12
#define NUM_PAGES   (int)((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

static struct control_data_s* ctl;
static int socket_handle = 0;
static int tcp_socket_connection = 0;

struct pi_options {
    int verbose;
    int udp;
    int tcp;
    int port;
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
    const volatile uint32_t* dma_reg_
);
static int frequency_to_control(float frequency);
static void write_samples(struct dma_cb_t* cbp, const float frequency);
static void initialize_dma(void);
static void initialize_mbox(void);
static int parse_json(
    const char* json,
    struct command_node_t** new_command,
    int* request_response
);
static void free_command(struct command_node_t* command);
static const char* post_response(const int status);
static int hexit_to_value(char hexit);
static int decode_post_request(char* raw_post_data, char* json_out);


int main(int argc, char** argv) {
#ifdef TEST_COMPILATION
    printf("Testing on non-Pi hardware, no radio signal will be generated\n");
#endif
    struct pi_options options = get_args(argc, argv);
    int i;

    struct sigaction sa;
    /**
     * Catch all signals possible - it is vital we kill the DMA engine
     * on process exit!
     */
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

    /* Calculate the frequency control word */
    /* The fractional part is stored in the lower 12 bits */
    float frequency = 49.830;

#ifndef TEST_COMPILATION
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
#else
    struct control_data_s test_struct;
    ctl = &test_struct;
#endif

    struct dma_cb_t* cbp = ctl->cb;

#ifndef TEST_COMPILATION
    write_samples(cbp, frequency);
#endif

    cbp->next = mem_virt_to_phys(mbox.virt_addr);

#ifndef TEST_COMPILATION
    initialize_dma();
#endif

    struct command_node_t* command = malloc(sizeof(*command));
    command->burst_us = 100.0f;
    command->spacing_us = 100.0f;
    command->repeats = 1;
    command->frequency = frequency;
    command->dead_frequency = frequency;
    command->next = NULL;

    struct command_node_t* new_command = NULL;

    if (options.udp) {
        socket_handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    } else {
        socket_handle = socket(AF_INET, SOCK_STREAM, 0);
    }
    if (socket_handle < 0) {
        fatal("Unable to create socket\n");
    }

    if (options.tcp) {
        int optionValue = 1;
        const int status = setsockopt(
                socket_handle,
                SOL_SOCKET,
                SO_REUSEADDR,
                (const void *)&optionValue,
                sizeof(int));
        if (status) {
            fatal("Unable to set socket options\n");
        }
    }
    struct sockaddr_in server_address;
    struct sockaddr_in client_address;
    socklen_t client_length = sizeof(client_address);
    bzero(&client_address, sizeof(client_address));
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);
    server_address.sin_port = htons(options.port);
    const int bind_status = bind(
        socket_handle,
        (struct sockaddr*)&server_address,
        sizeof(server_address));
    if (bind_status < 0) {
        fatal("Unable to bind socket\n");
    }

    if (options.tcp) {
        const int listen_status = listen(socket_handle, 1);
        if (listen_status < 0) {
            fatal("Unable to listen\n");
        }
        printf("Listening for commands on TCP port %d\n", options.port);
        tcp_socket_connection = accept(socket_handle, NULL, NULL);
        if (tcp_socket_connection < 0) {
            fatal("Unable to accept TCP connection\n");
        }
    }

    char message_buffer[BUFFER_SIZE];
    char post_data[BUFFER_SIZE];

    while (1) {
        /* This is nonblocking because we set it as such as above */
        int bytes_count;
        if (options.udp) {
            bytes_count = recvfrom(
                socket_handle,
                message_buffer,
                sizeof(message_buffer) / sizeof(message_buffer[0]),
                0,
                (struct sockaddr*)&client_address,
                &client_length);
        } else {
            if (tcp_socket_connection != 0) {
                bytes_count = recv(
                    tcp_socket_connection,
                    message_buffer,
                    sizeof(message_buffer) / sizeof(message_buffer[0]),
                    0);
                if (bytes_count == 0) {
                    /* Socket closed, wait for another connection */
                    tcp_socket_connection = 0;
                }
            } else {
                /* Look for another connection */
                tcp_socket_connection = accept(socket_handle, NULL, NULL);
                if (tcp_socket_connection < 0) {
                    fprintf(stderr, "Unable to accept TCP connection: %s\n", strerror(errno));
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        /* No connection ready, just keep running */
                        tcp_socket_connection = 0;
                    } else {
                        fatal("Unable to accept TCP connection: %s\n", strerror(errno));
                    }
                } else {
                    const int fcntl_status = fcntl(tcp_socket_connection, F_SETFL, O_NONBLOCK);
                    if (fcntl_status < 0) {
                        fatal("Unable to set socket options\n");
                    }
                    continue;
                }
            }
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
            int request_response = 0;
            char* json_data = NULL;
            if (strncmp("POST ", message_buffer, 5) == 0) {
                const int status = decode_post_request(message_buffer, post_data);
                json_data = post_data;
                const char* const response = post_response(status);
                send(tcp_socket_connection, response, strlen(response), 0);
                close(tcp_socket_connection);
                tcp_socket_connection = 0;
                if (status) {
                    printf("Bad POST request\n");
                    continue;
                }
            } else {
                json_data = message_buffer;
            }

            if (options.verbose) {
                printf("Received message \"%s\"\n", json_data);
            }
            const int parse_status = parse_json(
                json_data,
                &parsed_command,
                &request_response);
            if (request_response && options.udp) {
                const int length = snprintf(
                    message_buffer,
                    sizeof(message_buffer) / sizeof(message_buffer[0]),
                    "{\"time\": %d}",
                    (int)time(NULL));
                /**
                 * The client should be listening for responses on the port one
                 * above the port that it sent to us
                 */
                client_address.sin_port = htons(ntohs(server_address.sin_port) + 1);
                sendto(
                    socket_handle,
                    message_buffer,
                    length,
                    0,
                    (struct sockaddr*)&client_address,
                    client_length);
            }

            if (parse_status == 0) {
                /**
                 * Command bursts come after synchronization bursts, and
                 * they're the interesting part, so print those
                 */
                const struct command_node_t* const print_command = (
                    parsed_command->next == NULL
                    ? parsed_command
                    : parsed_command->next);
                if (options.verbose) {
                    printf(
                        "Sending command %d %d:%d bursts @ %4.3f (%4.3f)\n",
                        print_command->repeats,
                        (int)print_command->burst_us,
                        (int)print_command->spacing_us,
                        print_command->frequency,
                        print_command->dead_frequency);
                }

                if (new_command == NULL) {
                    new_command = parsed_command;
                } else {
                    free_command(command);
                    command = new_command;
                    new_command = parsed_command;
                }
            } else {
                /* The error message will be printed in the JSON parser */
                free_command(parsed_command);
            }
        }

#ifndef TEST_COMPILATION
        const int used = fill_buffer(command, new_command, ctl, dma_reg);
#else
        const int used = 1;
#endif
        if (used > 0) {
            if (new_command != NULL) {
                free_command(command);
                command = new_command;
                new_command = NULL;
            }
        }

        usleep(10000);
    }

    terminate(0);

    return 0;
}


static void write_samples(struct dma_cb_t* cbp, const float frequency) {
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
}


static void initialize_dma(void) {
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


static void udelay(int us) {
    struct timespec ts = { 0, us * 1000 };
    nanosleep(&ts, NULL);
}


static void terminate(const int signal_) {
    printf("Terminating with signal %d\n", signal_);
    if (dma_reg) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(500);
        /* Abort the whole DMA */
        dma_reg[DMA_CS] = BCM2708_DMA_ABORT | BCM2708_DMA_ACTIVE;
        udelay(500);
    }
    if (tcp_socket_connection != 0) {
        close(tcp_socket_connection);
    }
    if (socket_handle != 0) {
        close(socket_handle);
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


uint32_t mem_virt_to_phys(void* virt) {
    uint32_t offset = (uint8_t*)virt - mbox.virt_addr;
    return mbox.bus_addr + offset;
}


uint32_t mem_phys_to_virt(uint32_t phys) {
  return phys - (uint32_t)mbox.bus_addr + (uint32_t)mbox.virt_addr;
}

void* map_peripheral(uint32_t base, uint32_t len) {
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
        assert(time_us > 0.0f && "Time should be positive");

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
    const char* const json,
    struct command_node_t** command,
    int* const request_response
) {
    json_t* root = NULL;
    json_t* object = NULL;
    json_t* data = NULL;
    json_error_t error;
    int return_value = -1;

    assert(*command == NULL);

    root = json_loads(json, 0, &error);
    if (!root) {
        fprintf(stderr, "error: on line %d: %s\n", error.line, error.text);
        goto CLEANUP;
    }
    if (!json_is_array(root)) {
        fprintf(stderr, "not a JSON array\n");
        goto CLEANUP;
    }

    size_t array_index;
    const size_t array_size = json_array_size(root);
    for (array_index = 0; array_index < array_size; ++array_index) {
        object = json_array_get(root, array_index);
        if (!json_is_object(object)) {
            fprintf(
                stderr,
                "Item %zu in array is not an object\n",
                array_index + 1);
            goto CLEANUP;
        }

        *command = malloc(sizeof(**command));
        (*command)->next = NULL;

        data = json_object_get(object, "burst_us");
        if (json_is_number(data)) {
            (*command)->burst_us = json_number_value(data);
        } else {
            fprintf(
                stderr,
                "In item %zu: missing or invalid field: burst_us\n",
                array_index + 1);
            goto CLEANUP;
        }

        data = json_object_get(object, "spacing_us");
        if (json_is_number(data)) {
            (*command)->spacing_us = json_number_value(data);
        } else {
            fprintf(
                stderr,
                "In item %zu: missing or invalid field: spacing_us\n",
                array_index + 1);
            goto CLEANUP;
        }

        data = json_object_get(object, "repeats");
        if (json_is_number(data)) {
            (*command)->repeats = json_number_value(data);
        } else {
            fprintf(
                stderr,
                "In item %zu: missing or invalid field: repeats\n",
                array_index + 1);
            goto CLEANUP;
        }

        data = json_object_get(object, "frequency");
        if (json_is_number(data)) {
            (*command)->frequency = json_number_value(data);
        } else {
            fprintf(
                stderr,
                "In item %zu: missing or invalid field: frequency\n",
                array_index + 1);
            goto CLEANUP;
        }

        data = json_object_get(object, "dead_frequency");
        if (json_is_number(data)) {
            (*command)->dead_frequency = json_number_value(data);
        } else {
            fprintf(
                stderr,
                "In item %zu: missing or invalid field: dead_frequency\n",
                array_index + 1);
            goto CLEANUP;
        }

        data = json_object_get(object, "request_response");
        /**
         * request_response is optional so if it isn't a port number, ignore it
         */
        if (data != NULL && json_is_true(data)) {
            *request_response = 1;
        } else {
            *request_response = 0;
        }

        command = &((*command)->next);
    }

    return_value = 0;

CLEANUP:
    if (root != NULL) {
        json_decref(root);
    }
    return return_value;
}


void free_command(struct command_node_t* command) {
    while (command) {
        struct command_node_t* const previous = command;
        command = command->next;
        free(previous);
    }
}


struct pi_options get_args(const int argc, char* argv[]) {
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
    options.udp = 0;
    options.tcp = 0;
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
            case 't':
                options.tcp = 1;
                break;
            case 'u':
                options.udp = 1;
                break;
            default:
                fprintf(stderr, "Unknown option\n");
                print_usage(argv[0]);
                exit(EXIT_FAILURE);
        }
    }
    if (options.tcp && options.udp) {
        fprintf(stderr, "Only one of --tcp and --udp may be specified\n");
        exit(EXIT_FAILURE);
    }
    if (!options.tcp && !options.udp) {
        /* Default to TCP */
        options.tcp = 1;
    }
    return options;
}


void print_usage(const char* program) {
    printf("Usage: %s [OPTIONS]\n", program);
    printf("-p, --port     The port to listen for messags on.\n");
    printf("-h, --help     Print this help message.\n");
    printf("-v, --verbose  Print more debugging information.\n");
    printf("-t, --tcp      Listen for messages on a TCP connection.\n");
    printf("-u, --udp      Listen for messages on a UDP connection.\n");
}


const char* post_response(const int status) {
    if (status == 0) {
        return
"HTTP/1.0 200 OK\r\n\
Content-Type: text/plain\r\n";
    }
    return
"HTTP/1.0 422 Unprocessable\r\n\
Content-Type: text/plain\r\n";
}


int hexit_to_value(char hexit) {
    return (hexit > '9') ? (hexit &~ 0x20) - 'A' + 10 : (hexit - '0');
}


int decode_post_request(char* raw_post_data, char* json_out) {
    // Find two CRLFs in a row to get data
    char* message = strstr(raw_post_data, "\r\n\r\n");
    if (!message) {
        // Invalid POST
        return 1;
    }
    message += 4;

    // Find the parameter
    while (*message != '=') {
        ++message;
    }
    ++message;
    // There shouldn't be any other parameters, but just in case, scan to the
    // end. We also need to decode percent-encoding.
    while (*message != '\0') {
        if (*message == '=') {
            return 2;
        }
        if (*message == '%') {
            ++message;
            if (*message == '\0') {
                return 2;
            }
            const int firstHexValue = hexit_to_value(*message);

            ++message;
            if (*message == '\0') {
                return 2;
            }
            const int secondHexValue = hexit_to_value(*message);

            *json_out = (char)(firstHexValue * 16 + secondHexValue);
            ++json_out;
            ++message;

        } else { // Not '%'
            *json_out = *message;
            ++json_out;
            ++message;
        }
    }
    *json_out = '\0';
    return 0;
}
