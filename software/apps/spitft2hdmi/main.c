#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "tmds_encode.h"
#include "common_dvi_pin_configs.h"

#include "fourwire.pio.h"

// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

uint16_t framebuf[FRAME_WIDTH * FRAME_HEIGHT];

#define BIT_DEPOSIT(b, i) ((b) ? (1<<(i)) : 0)
#define BIT_EXTRACT(b, i) (((b) >> (i)) & 1)
#define BIT_MOVE(b, src, dest) BIT_DEPOSIT(BIT_EXTRACT(b, src), dest)
#define ENCODED_COMMAND(x) ( \
    (BIT_MOVE(x, 0,  0)) | \
    (BIT_MOVE(x, 1,  2)) | \
    (BIT_MOVE(x, 2,  4)) | \
    (BIT_MOVE(x, 3,  6)) | \
    (BIT_MOVE(x, 4,  8)) | \
    (BIT_MOVE(x, 5, 10)) | \
    (BIT_MOVE(x, 6, 12)) | \
    (BIT_MOVE(x, 7, 14)) \
)

struct dvi_inst dvi0;

uint8_t decode[256];

struct {
    uint8_t x0h, x0l, x1h, x1l, y0h, y0l, y1h, y1l;
} bounds;

#define COMMAND_CASET (0x2a)
#define COMMAND_PASET (0x2b)
#define COMMAND_RAMWR (0x2c)

static int clamped(int x, int lo, int hi) {
    if(x < lo) { return lo; }
    if(x > hi) { return hi; }
    return x;
}

#define GET_WORD() (word = pio_sm_get_blocking(pio, sm))
#define GET_DATA() do { GET_WORD(); if(IS_COMMAND()) goto next_command; } while(0)
#define IS_COMMAND() (!(word & 0x2))
#define DECODED() (decode[(word & 0x55) | ((word >> 7) & 0xaa)])

static inline void __not_in_flash_func(_dvi_prepare_scanline_16bpp)(struct dvi_inst *inst, uint32_t *scanbuf) {
    uint32_t *tmdsbuf;
    queue_remove_blocking_u32(&inst->q_tmds_free, &tmdsbuf);
    uint pixwidth = inst->timing->h_active_pixels;
    uint words_per_channel = pixwidth / DVI_SYMBOLS_PER_WORD;
    tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_16BPP_BLUE_MSB,  DVI_16BPP_BLUE_LSB );
    tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_16BPP_GREEN_MSB, DVI_16BPP_GREEN_LSB);
    tmds_encode_data_channel_16bpp(scanbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_16BPP_RED_MSB,   DVI_16BPP_RED_LSB  );
    queue_add_blocking_u32(&inst->q_tmds_valid, &tmdsbuf);
}

// Ugh copy/paste but it lets us garbage collect the TMDS stuff that is not being used from .scratch_x
void __not_in_flash_func(framebuf_main_16bpp)(struct dvi_inst *inst) {
    uint y = 0;
    while (1) {
        _dvi_prepare_scanline_16bpp(inst, (uint32_t*)&framebuf[(y % FRAME_HEIGHT) * FRAME_WIDTH]);
        ++y;
        if (y == inst->timing->v_active_lines) {
            y = 0;
        }
    }
    __builtin_unreachable();
}

void core1_main() {
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
    dvi_start(&dvi0);
    framebuf_main_16bpp(&dvi0);
}

// -----------------------------------------------------------------------------

int main() {
    vreg_set_voltage(VREG_VSEL);
    sleep_ms(10);
    set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);


    setup_default_uart();

    for(int i=0; i<256; i++) {
        int j = (BIT_MOVE(i,  0, 0)) |
                (BIT_MOVE(i,  2, 1)) |
                (BIT_MOVE(i,  4, 2)) |
                (BIT_MOVE(i,  6, 3)) |
                (BIT_MOVE(i,  1, 4)) |
                (BIT_MOVE(i,  3, 5)) |
                (BIT_MOVE(i,  5, 6)) |
                (BIT_MOVE(i,  7, 7));
        decode[i] = j;
    }

    dvi0.timing = &DVI_TIMING;
    dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    // DVI will start immediately & make its own gravy
    multicore_launch_core1(core1_main);

    PIO pio = pio1;

    uint offset = pio_add_program(pio, &fourwire_program);
    uint sm = pio_claim_unused_sm(pio, true);
    fourwire_program_init(pio, sm, offset, 18, 21);

    uint16_t word = 0;

    int x=0, y=0;
    while(true) {
        GET_WORD();
next_command:
        switch(word) {
        case ENCODED_COMMAND(COMMAND_CASET):
            GET_DATA();
            bounds.x0h = DECODED();

            GET_DATA();
            bounds.x0l = DECODED();

            GET_DATA();
            bounds.x1h = DECODED();

            GET_DATA();
            bounds.x1l = DECODED();
            break;

        case ENCODED_COMMAND(COMMAND_PASET):
            GET_DATA();
            bounds.y0h = DECODED();

            GET_DATA();
            bounds.y0l = DECODED();

            GET_DATA();
            bounds.y1h = DECODED();

            GET_DATA();
            bounds.y1l = DECODED();
            break;

        case ENCODED_COMMAND(COMMAND_RAMWR):
            {
                int X0 = bounds.x0l | (bounds.x0h << 8);
                X0 = clamped(X0, 0, FRAME_WIDTH);
                int X1 = bounds.x1l | (bounds.x1h << 8);
                X1 = clamped(X1, X0, FRAME_WIDTH) + 1;

                int Y0 = bounds.y0l | (bounds.y0h << 8);
                Y0 = clamped(Y0, 0, FRAME_HEIGHT);
                int Y1 = bounds.y1l | (bounds.y1h << 8);
                Y1 = clamped(Y1, Y0, FRAME_HEIGHT) + 1;
                while(1) {
                    for(y=Y0; y<Y1; y++) {
                        for(x=X0; x<X1; x++) {
                            GET_DATA();
                            uint16_t pixel = DECODED() << 8;
                            GET_DATA();
                            pixel |= DECODED();
                            framebuf[x + y * FRAME_WIDTH] = pixel;
                        }
                    }
                }
            }
        }
    }
}
