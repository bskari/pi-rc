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
#include <jansson.h>
#include <math.h>
#include <arpa/inet.h>
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

// RC commands are almost always sent at some multiple of 5 us.  The basic idea
// is to maintain a buffer of 4000 values to write to the clock control
// register and then arrange for the DMA controller to write the values
// sequentially at 5us intervals.  The control code can then wake up every 10ms
// or so and populate the buffer with new samples.  At 5us per sample, a 4000
// sample buffer will last 20ms, so waking every 10ms should be sufficient.
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

typedef struct {
    uint32_t info, src, dst, length,
             stride, next, pad[2];
} dma_cb_t;

typedef struct {
    uint8_t* virtaddr;
    uint32_t physaddr;
} page_map_t;

struct command_node_t {
    float burst_us;
    float spacing_us;
    int repeats;
    float frequency;
    float dead_frequency;

    struct command_node_t* next;
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
static int socket_handle = 0;

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
static int parse_json(
    const char* json,
    struct command_node_t** new_command,
    int* request_response
);
static void free_command(struct command_node_t* command);


int main(int argc, char** argv) {
    int i, mem_fd, pid;
    char pagemap_fn[64];

    struct sigaction sa;
    // Catch all signals possible - it is vital we kill the DMA engine
    // on process exit!
    for (i = 1; i < 64; i++) {
        // These are uncatchable
        if (i != SIGKILL && i != SIGSTOP) {
            memset(&sa, 0, sizeof(sa));
            sa.sa_handler = terminate;
            sigaction(i, &sa, NULL);
        }
    }

    // Calculate the frequency control word
    // The fractional part is stored in the lower 12 bits
    float frequency = 49.830;
    printf("Broadcasting at %0.3f\n", frequency);

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

    // Initialise PWM to use a 100MHz clock too, and set the range to 500 bits,
    // which is 5us, the rate at which we want to update the GPCLK control
    // register.
    pwm_reg[PWM_CTL] = 0;
    udelay(10);
    clk_reg[PWMCLK_CNTL] = 0x5A000006;              // Source=PLLD and disable
    udelay(100);
    clk_reg[PWMCLK_DIV] = 0x5A000000 | (5 << 12);  // set pwm div to 5, for 100MHz
    udelay(100);
    clk_reg[PWMCLK_CNTL] = 0x5A000016;              // Source=PLLD and enable
    udelay(100);
    pwm_reg[PWM_RNG1] = 500;
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

    struct command_node_t* command = malloc(sizeof(*command));
    command->burst_us = 100.0f;
    command->spacing_us = 100.0f;
    command->repeats = 1;
    command->frequency = frequency;
    command->dead_frequency = frequency;
    command->next = NULL;

    struct command_node_t* new_command = NULL;

    socket_handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_handle < 0) {
        fatal("Unable to create socket\n");
    }
    struct sockaddr_in server_address;
    struct sockaddr_in client_address;
    socklen_t client_length = sizeof(client_address);
    bzero(&client_address, sizeof(client_address));
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);
    const short port = (argc > 2) ? atoi(argv[2]) : 12345;
    server_address.sin_port = htons(port);
    const int bind_status = bind(
        socket_handle,
        (struct sockaddr*)&server_address,
        sizeof(server_address)
    );
    if (bind_status < 0) {
        fatal("Unable to bind socket\n");
    }
    const int fcntl_status = fcntl(socket_handle, F_SETFL, O_NONBLOCK);
    if (fcntl_status < 0) {
        fatal("Unable to set socket options\n");
    }

    char json_buffer[1000];

    while (1) {
        // This is nonblocking because we set it as such as above
        const int bytes_count = recvfrom(
            socket_handle,
            json_buffer,
            sizeof(json_buffer) / sizeof(json_buffer[0]),
            0,
            (struct sockaddr*)&client_address,
            &client_length
        );
        // If the received message is too long, then it will be truncated.  All
        // valid messages should always fit in the buffer, so just ignore it if
        // it's too long.
        if (
            bytes_count > 0
            && (size_t)bytes_count < sizeof(json_buffer) / sizeof(json_buffer[0])
        ) {
            json_buffer[bytes_count] = '\0';
            struct command_node_t* parsed_command = NULL;
            int request_response = 0;
            const int parse_status = parse_json(
                json_buffer,
                &parsed_command,
                &request_response
            );

            if (request_response) {
                const int length = snprintf(
                    json_buffer,
                    sizeof(json_buffer) / sizeof(json_buffer[0]),
                    "{\"time\": %d}",
                    (int)time(NULL)
                );
                // The client should be listening for responses on the port one
                // above the port that it sent to us
                client_address.sin_port = htons(ntohs(server_address.sin_port) + 1);
                sendto(
                    socket_handle,
                    json_buffer,
                    length,
                    0,
                    (struct sockaddr*)&client_address,
                    client_length
                );
            }

            if (parse_status == 0) {
                // Command bursts come after synchronization bursts, and
                // they're the interesting part, so print those
                const  struct command_node_t* const print_command = (
                    parsed_command->next == NULL
                    ? parsed_command
                    : parsed_command->next
                );
                printf(
                    "Sending command %d %f:%f bursts @ %4.3f (%4.3f)\n",
                    print_command->repeats,
                    print_command->burst_us,
                    print_command->spacing_us,
                    print_command->frequency,
                    print_command->dead_frequency
                );

                if (new_command == NULL) {
                    new_command = parsed_command;
                } else {
                    free_command(command);
                    command = new_command;
                    new_command = parsed_command;
                }
            } else {
                free_command(parsed_command);
            }
        }

        const int used = fill_buffer(command, new_command, ctl, dma_reg);
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


static void udelay(int us) {
    struct timespec ts = { 0, us * 1000 };
    nanosleep(&ts, NULL);
}


static void terminate(const int signal_) {
    if (signal_ == SIGVTALRM || signal_ == 28) {
        // This is normal and safe, ignore it
        return;
    }
    printf("Terminating with signal %d\n", signal_);
    if (dma_reg) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(500);
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

    for (; free_slots >= 0; --free_slots) {
        // From DMA settings above, each sample is 5us
        time_us -= 5.0f;

        if (time_us <= 0.0f) {
            switch (state) {
                case BURST:
                    time_us += current_command->spacing_us;
                    state = SPACING;
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
                    break;
                default:
                    assert(0 && "Unknown state");
            }
        }
        assert(time_us > 0.0f && "Time should be positive");

        int frequency_control;
        switch (state) {
            case BURST:
                frequency_control = frequency_to_control(current_command->frequency);
                break;
            case SPACING:
                frequency_control = frequency_to_control(current_command->dead_frequency);
                break;
            default:
                assert(0 && "Unknown state when looking for frequency control");
                frequency_control = frequency_to_control(current_command->frequency);
        }
        ctl_->sample[last_sample++] = (0x5A << 24 | frequency_control);
        if (last_sample == NUM_SAMPLES) {
            last_sample = 0;
        }
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
                array_index + 1
            );
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
                array_index + 1
            );
            goto CLEANUP;
        }

        data = json_object_get(object, "spacing_us");
        if (json_is_number(data)) {
            (*command)->spacing_us = json_number_value(data);
        } else {
            fprintf(
                stderr,
                "In item %zu: missing or invalid field: spacing_us\n",
                array_index + 1
            );
            goto CLEANUP;
        }

        data = json_object_get(object, "repeats");
        if (json_is_number(data)) {
            (*command)->repeats = json_number_value(data);
        } else {
            fprintf(
                stderr,
                "In item %zu: missing or invalid field: repeats\n",
                array_index + 1
            );
            goto CLEANUP;
        }

        data = json_object_get(object, "frequency");
        if (json_is_number(data)) {
            (*command)->frequency = json_number_value(data);
        } else {
            fprintf(
                stderr,
                "In item %zu: missing or invalid field: frequency\n",
                array_index + 1
            );
            goto CLEANUP;
        }

        data = json_object_get(object, "dead_frequency");
        if (json_is_number(data)) {
            (*command)->dead_frequency = json_number_value(data);
        } else {
            fprintf(
                stderr,
                "In item %zu: missing or invalid field: dead_frequency\n",
                array_index + 1
            );
            goto CLEANUP;
        }

        data = json_object_get(object, "request_response");
        // request_response is optional so if it isn't a port number, ignore it
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


static void free_command(struct command_node_t* command) {
    while (command) {
        struct command_node_t* const previous = command;
        command = command->next;
        free(previous);
    }
}
