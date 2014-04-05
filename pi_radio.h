#define PLLFREQ         500000000   // PLLD is running at 500MHz
#define CARRIERFREQ     100000000   // Carrier frequency is 100MHz

#define NUM_SAMPLES     4000

void udelay(int us);
void terminate(void);
void initialize_dma(void);
void initialize_dma_memory(int initial_value);
void fatal(char* fmt, ...);

/**
 * Returns the number of data points that need to be refilled.
 */
int get_refill_size(void);

/**
 * Returns the position of the data point that needs to start to be refilled.
 * Many applications (like an FM radio) won't need to use this, but others such
 * as an RC controller will. In the case of the RC controller, a single command
 * is always a certain length, and the we won't want to overwrite a command
 * that's only been half broadcast.
 */
int get_refill_position(void);

/**
 * Refills the buffer with data. Data should have at least get_refill_size items.
 */
void refill(const uint32_t* data);

/**
 * Refills the buffer with data, starting at get_refill_position + offset. This
 * should be used to avoid clobbering commands that are only partially
 * completed, such as in an RC car controller.
 */
void refill_from_offset(const uint32_t* data, int offset);
