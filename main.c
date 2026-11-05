
#include "project.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>

/* =====================================================
   HELICOPTER GAME - PSoC 5LP CY8C5888

   VDAC8_1 : X output
   VDAC8_2 : Y output
   UART_1  : Serial debugging
   Pin_3   : P2[2], active LOW switch
   Timer_1   : 10 ms game interrupt (1 kHz clock, period 9)
   Timer_DAC : 50 kHz DMA sample trigger (1 MHz clock, period 19)
   DMA_X/Y   : Buffered X/Y coordinate playback
   ===================================================== */

/* DISPLAY */

#define X_MIN               200
#define X_MAX               3880
#define Y_MIN               200
#define Y_MAX               3880

#define DAC_MAX             4095
#define DRAW_STEP           12

/* DMA frame buffer. A PSoC DMA TD is limited to 4095 bytes. */
#define MAX_POINTS          4000u
#define DMA_SAMPLE_HZ       50000u

/* MAP */

#define MAP_LENGTH          3680
#define BASE_SPEED          8
#define SPEED_STEP          2
#define MAX_LEVEL           10
#define GROUND_Y            650

/* HELICOPTER */

#define HELI_START_X        1500
#define HELI_START_Y        2050

#define HELI_SPEED          35
#define HELI_WIDTH          460
#define HELI_HEIGHT         280

/* TIMING */

#define TIMER_PERIOD_MS     10
#define MOVE_INTERVAL       5
#define SCORE_INTERVAL      100
#define GAME_OVER_TICKS     300

/* =====================================================
   STRUCTURES
   ===================================================== */

typedef struct
{
    int x;
    int y;
} Cloud;

typedef struct
{
    int x;
    int height;
} Tree;

typedef enum
{
    SCREEN_START = 0,
    SCREEN_GAMEPLAY,
    SCREEN_GAME_OVER
} GameState;

/* =====================================================
   DMA GRAPHICS BUFFERS
   ===================================================== */

typedef struct
{
    uint8 x[MAX_POINTS];
    uint8 y[MAX_POINTS];
    uint16 count;
} GraphicsBuffer;

/* Keep both buffers in SRAM. DMA reads one while CPU fills the other. */
static GraphicsBuffer graphics_buffer[2];
static volatile uint8 buffer_ready[2] = {0u, 0u};
static volatile uint8 dma_playing = 0u;
static volatile uint8 dma_buffer_index = 0u;
static volatile uint8 dma_x_done = 0u;
static volatile uint8 dma_y_done = 0u;
static volatile uint8 dma_frame_done = 0u;
static GraphicsBuffer *active_buffer = NULL;

/* DMA channel and transaction descriptor handles. */
static uint8 DMA_X_Chan;
static uint8 DMA_Y_Chan;
static uint8 DMA_X_TD;
static uint8 DMA_Y_TD;

/* =====================================================
   MAP DATA
   ===================================================== */

static const Cloud clouds[] =
{
    {150, 3200},
    {750, 3450},
    {1400, 3100},
    {2150, 3400},
    {2900, 3250},
    {3450, 3500}
};

#define CLOUD_COUNT \
    (sizeof(clouds)/sizeof(clouds[0]))

static const Tree trees[] =
{
    {100, 500},
    {400, 750},
    {850, 600},
    {1250, 850},
    {1750, 550},
    {2200, 800},
    {2650, 650},
    {3100, 900},
    {3500, 550}
};

#define TREE_COUNT \
    (sizeof(trees)/sizeof(trees[0]))

/* =====================================================
   GLOBAL VARIABLES
   ===================================================== */

/* Game state */

static GameState game_state = SCREEN_START;

/* Helicopter coordinates */

static int helicopter_x = HELI_START_X;
static int helicopter_y = HELI_START_Y;

/* Map coordinates */

static int map_scroll = 0;
static int map_speed = BASE_SPEED;
static int level = 1;

/* Scores */

static uint32_t score = 0;
static uint32_t previous_score = 0;

/* Timer */

static volatile uint32_t timer_ticks = 0;
/* Game Over timer */

static volatile uint16 game_over_ticks = 0;

/* Switch */

static volatile uint8 switch_pressed = 0;
static volatile uint8 switch_event = 0;

static uint8 previous_switch_state = 1;
static uint8 candidate_switch_state = 1;
static uint8 debounce_counter = 0;

#define DEBOUNCE_TICKS 3

/* =====================================================
   UART FUNCTIONS
   ===================================================== */

static void uart_print(const char *message)
{
    UART_1_PutString((char *)message);
}

static void uart_number(const char *label, uint32_t value)
{
    char buffer[64];

    sprintf(buffer, "%s%lu\r\n",
            label, (unsigned long)value);

    uart_print(buffer);
}

static void uart_state(const char *message)
{
    uart_print("\r\n============================\r\n");
    uart_print(message);
    uart_print("\r\n============================\r\n");
}

/* =====================================================
   SWITCH SCANNING AND DEBOUNCING

   Pin_3 = P2[2], active LOW
   HIGH = Released
   LOW  = Pressed

   IMPORTANT:
   The working non-DMA version scanned the GPIO from the
   main loop and from plot(). Preserve that behaviour here.
   DMA only changes how DAC coordinates are output; it must
   not move switch detection into the timer/DMA interrupt.

   scan_switch() is therefore called only in main context:
   - once on every main-loop pass
   - while CPU is building a graphics frame in plot()

   No UART calls are made from an ISR.
   ===================================================== */

static volatile uint8 switch_log_event = 0u;
static volatile uint8 switch_log_state = 1u;

static void scan_switch(void)
{
    uint8 raw_state = Pin_3_Read();

    if (raw_state != candidate_switch_state)
    {
        candidate_switch_state = raw_state;
        debounce_counter = 0u;
        return;
    }

    if (debounce_counter < DEBOUNCE_TICKS)
        debounce_counter++;

    if ((debounce_counter >= DEBOUNCE_TICKS) &&
        (candidate_switch_state != previous_switch_state))
    {
        previous_switch_state = candidate_switch_state;
        switch_log_state = previous_switch_state;
        switch_log_event = 1u;

        if (previous_switch_state == 0u)
        {
            switch_pressed = 1u;
            switch_event = 1u;
        }
        else
        {
            switch_pressed = 0u;
        }
    }
}

/* =====================================================
   DAC CONVERSION

   Original coordinates: 0 to 4095
   VDAC8 output: 0 to 255
   ===================================================== */

static uint8 convert_dac(int value)
{
    if (value < 0)
        value = 0;

    if (value > DAC_MAX)
        value = DAC_MAX;

    return (uint8)(((int32)value * 255L) / 4095L);
}

/* =====================================================
   DAC PLOTTING
   ===================================================== */

static void plot(int x, int y)
{
    uint16 index;

    /* Keep the same GPIO polling point as the working non-DMA code. */
    scan_switch();

    if (active_buffer == NULL)
        return;

    if (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX)
        return;

    if (active_buffer->count >= MAX_POINTS)
        return;

    index = active_buffer->count;
    active_buffer->x[index] = convert_dac(x);
    active_buffer->y[index] = convert_dac(y);
    active_buffer->count++;
}

/* =====================================================
   LINE DRAWING
   ===================================================== */

static void draw_line(int x1, int y1, int x2, int y2)
{
    int dx = x2 - x1;
    int dy = y2 - y1;

    int ax = abs(dx);
    int ay = abs(dy);

    int steps = (ax > ay) ? ax : ay;

    int count;
    int i;

    if (steps == 0)
    {
        plot(x1, y1);
        return;
    }

    count = (steps + DRAW_STEP - 1) / DRAW_STEP;

    for (i = 0; i <= count; i++)
    {
        int x = x1 + (int)(((int32)dx * i) / count);
        int y = y1 + (int)(((int32)dy * i) / count);

        plot(x, y);
    }
}

/* =====================================================
   FONT
   3 x 5 pixel characters

   A-Z, 0-9, colon and hyphen
   ===================================================== */

static const uint8 font[38][5] =
{
    {2,5,7,5,5}, {6,5,6,5,6}, {3,4,4,4,3},
    {6,5,5,5,6}, {7,4,6,4,7}, {7,4,6,4,4},
    {3,4,5,5,3}, {5,5,7,5,5}, {7,2,2,2,7},
    {1,1,1,5,2}, {5,5,6,5,5}, {4,4,4,4,7},
    {5,7,7,5,5}, {5,7,7,7,5}, {2,5,5,5,2},
    {6,5,6,4,4}, {2,5,5,7,3}, {6,5,6,5,5},
    {3,4,2,1,6}, {7,2,2,2,2}, {5,5,5,5,7},
    {5,5,5,5,2}, {5,5,7,7,5}, {5,5,2,5,5},
    {5,5,2,2,2}, {7,1,2,4,7},

    {7,5,5,5,7}, {2,6,2,2,7}, {6,1,2,4,7},
    {6,1,2,1,6}, {5,5,7,1,1}, {7,4,6,1,6},
    {3,4,7,5,7}, {7,1,2,2,2}, {7,5,7,5,7},
    {7,5,7,1,6},

    {0,2,0,2,0}, {0,0,7,0,0}
};

static int font_index(char c)
{
    if (c >= 'a' && c <= 'z')
        c = c - 'a' + 'A';

    if (c >= 'A' && c <= 'Z')
        return c - 'A';

    if (c >= '0' && c <= '9')
        return 26 + c - '0';

    if (c == ':')
        return 36;

    if (c == '-')
        return 37;

    return -1;
}

/* =====================================================
   TEXT DRAWING
   ===================================================== */

static void draw_char(char c, int x, int y, int scale)
{
    int index = font_index(c);
    int row;
    int col;

    if (index < 0)
        return;

    for (row = 0; row < 5; row++)
    {
        for (col = 0; col < 3; col++)
        {
            if (font[index][row] & (1 << (2-col)))
            {
                int px = x + col*scale;
                int py = y + (4-row)*scale;

                draw_line(px, py, px+scale, py);

                draw_line(px+scale, py,
                          px+scale, py+scale);
            }
        }
    }
}

static void draw_text(const char *str,
                      int x, int y, int scale)
{
    while (*str)
    {
        if (*str == ' ')
        {
            x += scale*4;
        }
        else
        {
            draw_char(*str, x, y, scale);
            x += scale*4;
        }

        str++;
    }
}

static void draw_number(uint32_t number,
                        int x, int y, int scale)
{
    char buffer[12];

    sprintf(buffer, "%lu", (unsigned long)number);

    draw_text(buffer, x, y, scale);
}

/* =====================================================
   HELICOPTER MODULE
   ===================================================== */

static void draw_helicopter(void)
{
    int x = helicopter_x;
    int y = helicopter_y;

    /* Main body */

    draw_line(x-180, y, x-130, y+65);
    draw_line(x-130, y+65, x+40, y+75);
    draw_line(x+40, y+75, x+130, y+35);
    draw_line(x+130, y+35, x+180, y);
    draw_line(x+180, y, x+130, y-50);
    draw_line(x+130, y-50, x-130, y-65);
    draw_line(x-130, y-65, x-180, y);

    /* Cockpit */

    draw_line(x+20, y+65, x+105, y+30);
    draw_line(x+105, y+30, x+145, y-10);
    draw_line(x+145, y-10, x+105, y-40);
    draw_line(x+105, y-40, x+20, y-55);

    /* Tail */

    draw_line(x-180, y, x-390, y+15);
    draw_line(x-390, y+15, x-400, y-15);

    /* Tail rotor */

    draw_line(x-420, y-65, x-420, y+65);
    draw_line(x-460, y, x-380, y);

    /* Rotor mast */

    draw_line(x, y+75, x, y+140);

    /* Static main rotor */

    draw_line(x-220, y+140, x+220, y+140);

    /* Landing gear */

    draw_line(x-100, y-65, x-130, y-130);
    draw_line(x+90, y-60, x+120, y-130);
    draw_line(x-160, y-130, x+160, y-130);
}

/* =====================================================
   HELICOPTER MOVEMENT
   Independent of map coordinates
   ===================================================== */

static void update_helicopter(void)
{
    if (switch_pressed)
    {
        helicopter_y += HELI_SPEED;
    }
    else
    {
        helicopter_y -= HELI_SPEED;
    }

    if (helicopter_y > Y_MAX - 250)
        helicopter_y = Y_MAX - 250;

    if (helicopter_y < Y_MIN + 250)
        helicopter_y = Y_MIN + 250;
}

static void reset_helicopter(void)
{
    helicopter_x = HELI_START_X;
    helicopter_y = HELI_START_Y;
}

/* =====================================================
   MAP MODULE
   ===================================================== */

static void draw_cloud(int x, int y)
{
    draw_line(x, y, x+45, y+35);
    draw_line(x+45, y+35, x+90, y+55);
    draw_line(x+90, y+55, x+135, y+35);
    draw_line(x+135, y+35, x+180, y);
    draw_line(x+180, y, x+220, y);

    draw_line(x+220, y, x+170, y-20);
    draw_line(x+170, y-20, x+70, y-20);
    draw_line(x+70, y-20, x, y);
}

static void draw_tree(int x, int base, int height)
{
    int h = height;

    /* Trunk */

    draw_line(x, base, x, base+h/3);

    /* Lower branches */

    draw_line(x, base+h/3,
              x-h/3, base+h/5);

    draw_line(x-h/3, base+h/5,
              x, base+2*h/3);

    draw_line(x, base+2*h/3,
              x+h/3, base+h/5);

    draw_line(x+h/3, base+h/5,
              x, base+h/3);

    /* Middle branches */

    draw_line(x, base+2*h/3,
              x-h/4, base+h/2);

    draw_line(x-h/4, base+h/2,
              x, base+5*h/6);

    draw_line(x, base+5*h/6,
              x+h/4, base+h/2);

    draw_line(x+h/4, base+h/2,
              x, base+2*h/3);

    /* Top */

    draw_line(x, base+h,
              x-h/7, base+5*h/6);

    draw_line(x-h/7, base+5*h/6,
              x, base+3*h/4);

    draw_line(x, base+3*h/4,
              x+h/7, base+5*h/6);

    draw_line(x+h/7, base+5*h/6,
              x, base+h);
}

static void draw_cloud_map(void)
{
    unsigned int i;
    int k;
    int x;

    for (i = 0; i < CLOUD_COUNT; i++)
    {
        for (k = -1; k <= 1; k++)
        {
            x = X_MIN + clouds[i].x
                + k*MAP_LENGTH - map_scroll;

            if (x > X_MIN-250 && x < X_MAX)
            {
                draw_cloud(x, clouds[i].y);
            }
        }
    }
}

static void draw_tree_map(void)
{
    unsigned int i;
    int k;
    int x;

    for (i = 0; i < TREE_COUNT; i++)
    {
        for (k = -1; k <= 1; k++)
        {
            x = X_MIN + trees[i].x
                + k*MAP_LENGTH - map_scroll;

            if (x > X_MIN-300 && x < X_MAX)
            {
                draw_tree(x, GROUND_Y, trees[i].height);
            }
        }
    }
}

static void draw_map(void)
{
    draw_cloud_map();
    draw_tree_map();

    draw_line(X_MIN, GROUND_Y,
              X_MAX, GROUND_Y);
}

static void update_map(void)
{
    map_scroll += map_speed;

    if (map_scroll >= MAP_LENGTH)
    {
        map_scroll -= MAP_LENGTH;

        if (level < MAX_LEVEL)
        {
            level++;

            map_speed =
                BASE_SPEED + (level-1)*SPEED_STEP;

            uart_print("\r\nNEW LEVEL\r\n");
            uart_number("LEVEL: ", level);
            uart_number("MAP SPEED: ", map_speed);
        }
    }
}

static void reset_map(void)
{
    map_scroll = 0;
    map_speed = BASE_SPEED;
    level = 1;
}

/* =====================================================
   COLLISION DETECTION (NOT USED IN THIS VERSION)
   The routine is retained for later re-enabling, but game_tick()
   does not call it, so it cannot end the game.
   ===================================================== */

static uint8 check_collision(void)
{
    unsigned int i;

    int hx_left = helicopter_x - HELI_WIDTH/2;
    int hx_right = helicopter_x + HELI_WIDTH/2;

    int hy_bottom = helicopter_y - HELI_HEIGHT/2;
    int hy_top = helicopter_y + HELI_HEIGHT/2;

    /* Screen boundaries */

    if (hy_bottom <= Y_MIN || hy_top >= Y_MAX)
        return 1;

    /* Ground collision */

    if (hy_bottom <= GROUND_Y)
        return 1;

    /* Tree collision */

    for (i = 0; i < TREE_COUNT; i++)
    {
        int k;

        for (k = -1; k <= 1; k++)
        {
            int tx = X_MIN + trees[i].x
                     + k*MAP_LENGTH - map_scroll;

            int tree_width = trees[i].height/3;

            int tree_left = tx-tree_width;
            int tree_right = tx+tree_width;

            int tree_top = GROUND_Y + trees[i].height;

            if (hx_right > tree_left &&
                hx_left < tree_right &&
                hy_bottom < tree_top &&
                hy_top > GROUND_Y)
            {
                return 1;
            }
        }
    }

    return 0;
}

/* =====================================================
   GAME RESET
   ===================================================== */

static void reset_game(void)
{
    score = 0;

    reset_helicopter();
    reset_map();

    uart_state("GAME STARTED");
}

/* =====================================================
   GAME OVER
   ===================================================== */

static void start_game_over(void)
{
    previous_score = score;

    game_over_ticks = 0;

    game_state = SCREEN_GAME_OVER;

    uart_state("GAME OVER");
    uart_number("FINAL SCORE: ", score);
}

/* =====================================================
   GAME TIMING
   Called once for every elapsed 10 ms timer tick.
   Helicopter movement and map scrolling update every 50 ms.
   Collision detection is disabled in this version.
   ===================================================== */

static void game_tick(void)
{
    static uint8 move_counter = 0;
    static uint8 score_counter = 0;

    if (game_state == SCREEN_GAMEPLAY)
    {
        /* Update helicopter and map every 50 ms */

        move_counter++;

        if (move_counter >= MOVE_INTERVAL)
        {
            move_counter = 0;

            /*
             * Helicopter movement:
             *   Pin_3 LOW  -> switch_pressed = 1 -> move UP
             *   Pin_3 HIGH -> switch_pressed = 0 -> move DOWN
             *
             * Movement is updated every 50 ms (5 x 10 ms game ticks).
             * Collision detection is intentionally disabled for now.
             */
            update_helicopter();
            update_map();

            /* Debug: verify that the CPU is changing the helicopter Y. */
            uart_number("HELI Y: ", (uint32_t)helicopter_y);
        }

        /* Score increases every second */

        score_counter++;

        if (score_counter >= SCORE_INTERVAL)
        {
            score_counter = 0;

            score++;

            uart_number("SCORE: ", score);
        }
    }
    else if (game_state == SCREEN_GAME_OVER)
    {
        game_over_ticks++;

        if (game_over_ticks >= GAME_OVER_TICKS)
        {
            game_over_ticks = 0;

            game_state = SCREEN_START;

            uart_state("RETURNING TO START SCREEN");
            uart_number("PREVIOUS SCORE: ", previous_score);
        }
    }
    else
    {
        move_counter = 0;
        score_counter = 0;
    }
}

/* =====================================================
   SCREEN 1 - START SCREEN
   ===================================================== */

static void draw_start_screen(void)
{
    draw_text("HELICOPTER", 1150, 2850, 65);
    draw_text("GAME", 1750, 2550, 75);

    draw_text("BY ANAND AND JIJO", 900, 2100, 35);

    draw_text("PREVIOUS SCORE", 900, 1500, 35);
    draw_number(previous_score, 2600, 1500, 50);

    draw_text("PRESS SWITCH", 1050, 800, 45);
    draw_text("TO START", 1500, 550, 45);
}

/* =====================================================
   SCREEN 2 - GAMEPLAY
   ===================================================== */

static void draw_gameplay_screen(void)
{
    draw_text("T", 350, 3500, 55);
    draw_text(":", 480, 3500, 55);

    draw_number(score, 600, 3500, 55);

    /*
     * Draw the helicopter FIRST. The DMA frame buffer is limited to
     * MAX_POINTS samples. The map contains many line segments and can
     * otherwise fill the buffer before the helicopter is added.
     * Keeping the helicopter first guarantees that its latest Y position
     * is present in every gameplay frame.
     */
    draw_helicopter();

    draw_map();
}

/* =====================================================
   SCREEN 3 - GAME OVER
   ===================================================== */

static void draw_game_over_screen(void)
{
    draw_text("GAME", 1450, 2650, 100);
    draw_text("OVER", 1450, 2250, 100);

    draw_text("SCORE", 950, 1500, 60);
    draw_text(":", 1900, 1500, 60);

    draw_number(score, 2150, 1500, 65);
}

/* =====================================================
   SCREEN CONTROLLER
   ===================================================== */

static void draw_current_screen(void)
{
    switch (game_state)
    {
        case SCREEN_START:
            draw_start_screen();
            break;

        case SCREEN_GAMEPLAY:
            draw_gameplay_screen();
            break;

        case SCREEN_GAME_OVER:
            draw_game_over_screen();
            break;

        default:
            break;
    }
}

/* =====================================================
   TIMER INTERRUPT

   Timer_1 terminal-count output connected to isr_1.
   Timer input clock = 1 kHz
   Period = 9
   Interrupt interval = 10 ms
   ===================================================== */

CY_ISR(Timer_ISR)
{
    /* Clear the Timer_1 terminal-count interrupt source. */
    Timer_1_ReadStatusRegister();

    /* One tick every 10 ms. Switch scanning is kept in main context. */
    timer_ticks++;
}

/* DMA_X nrq and DMA_Y nrq must each connect to a separate ISR component. */
CY_ISR(DMA_X_ISR)
{
    isr_dma_x_ClearPending();
    dma_x_done = 1u;

    if (dma_y_done)
    {
        Timer_DAC_Stop();
        dma_frame_done = 1u;
    }
}

CY_ISR(DMA_Y_ISR)
{
    isr_dma_y_ClearPending();
    dma_y_done = 1u;

    if (dma_x_done)
    {
        Timer_DAC_Stop();
        dma_frame_done = 1u;
    }
}

/* Initialize DMA channels. DMA_X/Y component names must match the schematic. */
static void dma_init(void)
{
    DMA_X_Chan = DMA_X_DmaInitialize(
        1u, 1u,
        HI16((uint32)graphics_buffer[0].x),
        HI16((uint32)VDAC8_1_Data_PTR));

    DMA_Y_Chan = DMA_Y_DmaInitialize(
        1u, 1u,
        HI16((uint32)graphics_buffer[0].y),
        HI16((uint32)VDAC8_2_Data_PTR));

    DMA_X_TD = CyDmaTdAllocate();
    DMA_Y_TD = CyDmaTdAllocate();

    if (DMA_X_Chan == DMA_INVALID_CHANNEL ||
        DMA_Y_Chan == DMA_INVALID_CHANNEL ||
        DMA_X_TD == DMA_INVALID_TD || DMA_Y_TD == DMA_INVALID_TD)
    {
        uart_state("DMA ALLOCATION ERROR");
        for (;;) { }
    }
}

/* Configure and start one finite X/Y frame. Timer_DAC starts only after
   both DMA channels are armed, so the first sample is not lost. */
static uint8 start_dma_frame(uint8 index)
{
    uint16 count = graphics_buffer[index].count;

    if (count == 0u || count > MAX_POINTS)
        return 0u;

    Timer_DAC_Stop();

    CyDmaChDisable(DMA_X_Chan);
    CyDmaChDisable(DMA_Y_Chan);
    CyDmaClearPendingDrq(DMA_X_Chan);
    CyDmaClearPendingDrq(DMA_Y_Chan);

    dma_x_done = 0u;
    dma_y_done = 0u;
    dma_frame_done = 0u;
    dma_buffer_index = index;

    CyDmaTdSetConfiguration(
        DMA_X_TD, count, DMA_INVALID_TD,
        DMA_X__TD_TERMOUT_EN | TD_INC_SRC_ADR);
    CyDmaTdSetAddress(
        DMA_X_TD,
        LO16((uint32)graphics_buffer[index].x),
        LO16((uint32)VDAC8_1_Data_PTR));

    CyDmaTdSetConfiguration(
        DMA_Y_TD, count, DMA_INVALID_TD,
        DMA_Y__TD_TERMOUT_EN | TD_INC_SRC_ADR);
    CyDmaTdSetAddress(
        DMA_Y_TD,
        LO16((uint32)graphics_buffer[index].y),
        LO16((uint32)VDAC8_2_Data_PTR));

    CyDmaChSetInitialTd(DMA_X_Chan, DMA_X_TD);
    CyDmaChSetInitialTd(DMA_Y_Chan, DMA_Y_TD);

    CyDmaChEnable(DMA_X_Chan, 1u);
    CyDmaChEnable(DMA_Y_Chan, 1u);

    dma_playing = 1u;
    Timer_DAC_Start();
    return 1u;
}

/* Render into a buffer that DMA is not currently reading and which is not
   already queued for playback. */
static uint8 render_one_free_buffer(void)
{
    uint8 index;

    for (index = 0u; index < 2u; index++)
    {
        if (!buffer_ready[index] &&
            (!dma_playing || index != dma_buffer_index))
        {
            graphics_buffer[index].count = 0u;
            active_buffer = &graphics_buffer[index];
            draw_current_screen();
            active_buffer = NULL;

            if (graphics_buffer[index].count == 0u)
                return 0u;

            buffer_ready[index] = 1u;
            return 1u;
        }
    }

    return 0u;
}

static void service_dma_graphics(void)
{
    uint8 index;

    if (dma_frame_done)
    {
        CyGlobalIntDisable;
        dma_frame_done = 0u;
        dma_playing = 0u;
        buffer_ready[dma_buffer_index] = 0u;
        CyGlobalIntEnable;
    }

    if (!dma_playing)
    {
        if (buffer_ready[0])
            index = 0u;
        else if (buffer_ready[1])
            index = 1u;
        else
        {
            render_one_free_buffer();
            return;
        }

        if (start_dma_frame(index))
            return;
    }

    /* While DMA plays one buffer, CPU renders the other. */
    render_one_free_buffer();
}

/* =====================================================
   MAIN PROGRAM
   ===================================================== */

int main(void)
{
    uint8 event;
    uint8 log_event;
    uint8 log_state;

    /* Keep global interrupts disabled while components and state are set up. */
    CyGlobalIntDisable;

    UART_1_Start();
    CyDelay(500u);

    uart_print("\r\n================================\r\n");
    uart_print("HELICOPTER GAME BY ANAND AND JIJO\r\n");
    uart_print("PSoC 5LP CPU + DMA GRAPHICS\r\n");
    uart_print("================================\r\n");

    VDAC8_1_Start();
    VDAC8_2_Start();
    VDAC8_1_SetValue(0u);
    VDAC8_2_SetValue(0u);
    uart_print("VDAC8 CHANNELS READY\r\n");

    uart_print("GPIO SWITCH READY - P2[2] ACTIVE LOW\r\n");

    reset_helicopter();
    reset_map();
    score = 0u;
    previous_score = 0u;
    game_over_ticks = 0u;
    timer_ticks = 0u;
    switch_pressed = 0u;
    switch_event = 0u;
    switch_log_event = 0u;

    previous_switch_state = Pin_3_Read();
    candidate_switch_state = previous_switch_state;
    debounce_counter = 0u;
    game_state = SCREEN_START;

    Clock_DAC_Start();

    isr_dma_x_StartEx(DMA_X_ISR);
    isr_dma_y_StartEx(DMA_Y_ISR);
    dma_init();

    uart_print("DMA CHANNELS INITIALIZED\r\n");
    uart_print("DAC DMA TIMER READY: 50 kHz\r\n");

    /* Keep Timer_1 configured for diagnostics, but do not rely on its ISR
       for game movement in this version. The main loop supplies the 10 ms
       game tick so movement remains functional if Timer_1 is not firing. */
    isr_1_StartEx(Timer_ISR);
    Clock_2_Start();
    Timer_1_Start();

    uart_print("GAME TIMER READY: SOFTWARE 10 ms TICK\r\n");
    uart_print("GAME INITIALIZED - WAITING FOR SWITCH\r\n");
    uart_print("HELICOPTER CONTROL: PRESS=UP, RELEASE=DOWN\r\n");
    uart_print("MOVEMENT UPDATE: 50 ms; COLLISION DETECTION DISABLED\r\n");

    CyGlobalIntEnable;

    render_one_free_buffer();

    for (;;)
    {
        /* Preserve the working non-DMA GPIO polling method. */
        scan_switch();

        /* Atomically consume switch and UART-log events. */
        CyGlobalIntDisable;
        event = switch_event;
        switch_event = 0u;
        log_event = switch_log_event;
        log_state = switch_log_state;
        switch_log_event = 0u;
        CyGlobalIntEnable;

        if (log_event)
        {
            if (log_state == 0u)
                uart_print("SWITCH PRESSED\r\n");
            else
                uart_print("SWITCH RELEASED\r\n");
        }

        if ((game_state == SCREEN_START) && event)
        {
            reset_game();
            game_state = SCREEN_GAMEPLAY;
            uart_state("GAMEPLAY SCREEN");
        }

        /* One software game tick per 10 ms. DMA continues autonomously
           while the CPU waits; interrupts remain enabled during CyDelay. */
        CyDelay(10u);
        game_tick();

        /* Service DMA frequently so completed frames are replaced. */
        service_dma_graphics();
    }
}
