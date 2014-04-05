/*
 * RaspberryPi based FM transmitter.  For the original idea, see:
 *
 * http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
 *
 * All credit to Oliver Mattos and Oskar Weigl for creating the original code.
 * DMA modifications and other improvements by Richard Hirst
 * <richardghirst@gmail.com>  December 2012.
 * RC PWM stuff added by Brandon Skari.
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "pi_radio.h"

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

#define NUM_CBS         (NUM_SAMPLES * 2)

#define BCM2708_DMA_NO_WIDE_BURSTS  (1<<26)
#define BCM2708_DMA_WAIT_RESP       (1<<3)
#define BCM2708_DMA_D_DREQ          (1<<6)
#define BCM2708_DMA_PER_MAP(x)      ((x)<<16)
#define BCM2708_DMA_END             (1<<1)
#define BCM2708_DMA_RESET           (1<<31)
#define BCM2708_DMA_INT             (1<<2)

#define DMA_CS          (0x00/4)
#define DMA_CONBLK_AD   (0x04/4)
#define DMA_DEBUG       (0x20/4)

#define DMA_BASE        0x20007000
#define DMA_LEN         0x24
#define PWM_BASE        0x2020C000
#define PWM_LEN         0x28
#define CLK_BASE        0x20101000
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
#define PWMCTL_CLRF         (1<<6)
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

page_map_t* page_map;

static uint8_t* virtbase;

static volatile uint32_t* pwm_reg;
static volatile uint32_t* clk_reg;
static volatile uint32_t* dma_reg;
static volatile uint32_t* gpio_reg;
static uint32_t last_cb;

struct control_data_s {
    dma_cb_t cb[NUM_CBS];
    uint32_t sample[NUM_SAMPLES];
};

#define PAGE_SIZE   4096
#define PAGE_SHIFT  12
#define NUM_PAGES   (int)((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

static struct control_data_s* ctl = NULL;
static int dma_initialized = 0;

void
udelay(int us) {
    struct timespec ts = { 0, us * 1000 };

    nanosleep(&ts, NULL);
}

// The int is a dummy parameter so that the function can be used as a signal
// handlers
void
terminate() {
    if (dma_reg) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(10);
    }
    exit(1);
}

void
fatal(char* fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    terminate();
}

static uint32_t
mem_virt_to_phys(void* virt) {
    uint32_t offset = (uint8_t*)virt - virtbase;

    return page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE);
}

static uint32_t
mem_phys_to_virt(uint32_t phys) {
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

static void*
map_peripheral(uint32_t base, uint32_t len) {
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

void
initialize_dma(void) {
    dma_initialized = 1;
    int i;

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
    const int pid = getpid();
    char pagemap_fn[64];
    snprintf(
        pagemap_fn,
        sizeof(pagemap_fn) / sizeof(pagemap_fn[0]),
        "/proc/%d/pagemap",
        pid
    );
    const int fd = open(pagemap_fn, O_RDONLY);
    if (fd < 0) {
        fatal("Failed to open %s: %m\n", pagemap_fn);
    }
    if (lseek(fd, (unsigned long)virtbase >> 9, SEEK_SET) != (unsigned long)virtbase >> 9) {
        fatal("Failed to seek on %s: %m\n", pagemap_fn);
    }
//  printf("Page map:\n");
    for (i = 0; i < NUM_PAGES; i++) {
        uint64_t pfn;
        page_map[i].virtaddr = virtbase + i * PAGE_SIZE;
        // Following line forces page to be allocated
        page_map[i].virtaddr[0] = 0;
        if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn)) {
            fatal("Failed to read %s: %m\n", pagemap_fn);
        }
        if (((pfn >> 55) & 0xfbf) != 0x10c) { // pagemap bits: https://www.kernel.org/doc/Documentation/vm/pagemap.txt
            fatal("Page %d not present (pfn 0x%016llx)\n", i, pfn);
        }
        page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
//      printf("  %2d: %8p ==> 0x%08x [0x%016llx]\n", i, page_map[i].virtaddr, page_map[i].physaddr, pfn);
    }

    // GPIO4 needs to be ALT FUNC 0 to output the clock
    gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12);

    // Program GPCLK to use MASH setting 1, so fractional dividers work
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 6;
    udelay(100);
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 1 << 9 | 1 << 4 | 6;

    ctl = (struct control_data_s*)virtbase;
    last_cb = (uint32_t)ctl->cb;

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
    printf("Setting DMA_CONBLK_AD to %p %p\n", ctl, ctl->cb);
    dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
    printf("virt %x\n", mem_virt_to_phys(ctl->cb));
    printf("Set dma_reg[DMA_CONBLK_AD] to %d\n", dma_reg[DMA_CONBLK_AD]);
    dma_reg[DMA_DEBUG] = 7; // clear debug error flags
    dma_reg[DMA_CS] = 0x10880001;   // go, mid priority, wait for outstanding writes
}


void
initialize_dma_memory(const int initial_value) {
    assert(dma_initialized && "Need to call dma memory first");
    int i;
    dma_cb_t* cbp = ctl->cb;
    const uint32_t phys_pwm_fifo_addr = 0x7e20c000 + 0x18;
    for (i = 0; i < NUM_SAMPLES; i++) {
        ctl->sample[i] = initial_value;
        // Write a frequency sample
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
        cbp->src = mem_virt_to_phys(ctl->sample + i);
        cbp->dst = CM_GP0DIV;
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
}


int
get_refill_size(void) {
    printf("dma_reg[DMA_CONBLK_AD] == %d\n", dma_reg[DMA_CONBLK_AD]);
    const uint32_t cur_cb = mem_phys_to_virt(dma_reg[DMA_CONBLK_AD]);

    const int last_sample = (last_cb - (uint32_t)virtbase) / (sizeof(dma_cb_t) * 2);
    const int this_sample = (cur_cb - (uint32_t)virtbase) / (sizeof(dma_cb_t) * 2);
    const int diff = this_sample - last_sample;
    const int free_slots = diff < 0 ? diff + NUM_SAMPLES : diff;
    return free_slots;
}


int
get_refill_position(void) {
    const int last_sample = (last_cb - (uint32_t)virtbase) / (sizeof(dma_cb_t) * 2);
    return last_sample;
}


void
refill(const uint32_t* const data) {
    refill_from_offset(data, 0);
}


void
refill_from_offset(const uint32_t* const data, const int offset) {
    int fill_index = get_refill_position() + offset;

    int i;
    const int limit = get_refill_size();
    for (i = 0; i < limit; ++i) {
        ctl->sample[fill_index++] = data[i];
        if (fill_index == NUM_SAMPLES) {
            fill_index = 0;
        }
    }

    last_cb = (uint32_t)virtbase + fill_index * sizeof(dma_cb_t) * 2;
}
