
#include "project.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

/* =====================================================
   HELICOPTER GAME - PSoC 5LP CY8C5888

   VDAC8_1 : X output
   VDAC8_2 : Y output
   UART_1  : Serial debugging
   Pin_3   : P2[2], active LOW switch
   Timer_1 : 10 ms interrupt
   ===================================================== */

/* DISPLAY */

#define X_MIN               200
#define X_MAX               3880
#define Y_MIN               200
#define Y_MAX               3880

#define DAC_MAX             4095
#define DRAW_STEP           12
#define DAC_DELAY           5

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
   CONTINUOUS GPIO SWITCH SCANNING

   Pin_3 = P2[2]
   HIGH = Released
   LOW  = Pressed

   This function is called from:
   1. Main loop
   2. DAC plotting function

   Therefore, GPIO is checked during drawing too.
   ===================================================== */

static void scan_switch(void)
{
    uint8 raw_state;

    raw_state = Pin_3_Read();

    /* Check whether raw input has changed */

    if (raw_state != candidate_switch_state)
    {
        candidate_switch_state = raw_state;
        debounce_counter = 0;
        return;
    }

    /* Require 3 consecutive timer samples */

    if (debounce_counter < DEBOUNCE_TICKS)
    {
        debounce_counter++;
    }

    if (debounce_counter >= DEBOUNCE_TICKS)
    {
        /* Accept only a confirmed state change */

        if (candidate_switch_state != previous_switch_state)
        {
            previous_switch_state = candidate_switch_state;

            if (previous_switch_state == 0)
            {
                switch_pressed = 1;
                switch_event = 1;

                uart_print("SWITCH PRESSED\r\n");
            }
            else
            {
                switch_pressed = 0;

                uart_print("SWITCH RELEASED\r\n");
            }
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
    /* Continuously scan GPIO during drawing */

    scan_switch();

    if (x < X_MIN || x > X_MAX ||
        y < Y_MIN || y > Y_MAX)
    {
        return;
    }

    VDAC8_1_SetValue(convert_dac(x));
    VDAC8_2_SetValue(convert_dac(y));

    CyDelayUs(DAC_DELAY);
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
   COLLISION DETECTION
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
   Called once for every elapsed 10 ms timer tick
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

            update_helicopter();
            update_map();

            if (check_collision())
            {
                start_game_over();
                return;
            }
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

    draw_map();

    draw_helicopter();
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
    Timer_1_ReadStatusRegister();

    timer_ticks++;

    scan_switch();
}

/* =====================================================
   MAIN PROGRAM
   ===================================================== */

int main(void)
{
    uint32_t current_ticks;
    uint32_t previous_ticks = 0;
    uint32_t elapsed_ticks;

    uint8 event;

    CyGlobalIntEnable;

    /* UART initialization */

    UART_1_Start();

    CyDelay(500);

    uart_print("\r\n");
    uart_print("================================\r\n");
    uart_print("HELICOPTER GAME BY ANAND AND JIJO\r\n");
    uart_print("PSoC 5LP INITIALIZATION\r\n");
    uart_print("================================\r\n");

    /* DAC initialization */

    VDAC8_1_Start();
    VDAC8_2_Start();

    uart_print("VDAC8_1 READY\r\n");
    uart_print("VDAC8_2 READY\r\n");

    /* Switch initialization */

    uart_print("GPIO SWITCH READY\r\n");
    uart_print("PIN: P2[2]\r\n");

    /* Timer initialization */

    Timer_1_Start();

    isr_1_StartEx(Timer_ISR);

    uart_print("TIMER READY\r\n");

    /* Game initialization */

    reset_helicopter();
    reset_map();

    score = 0;
    previous_score = 0;
    game_over_ticks = 0;

    switch_pressed = 0;
    switch_event = 0;
    previous_switch_state = 1;

    game_state = SCREEN_START;

    uart_print("GAME INITIALIZED\r\n");
    uart_print("WAITING FOR SWITCH PRESS\r\n");
    uart_print("================================\r\n");

    /* =================================================
       MAIN LOOP
       ================================================= */

    for (;;)
    {
        /*
         * 1. Continuously scan physical GPIO.
         */

        scan_switch();

        /*
         * 2. Read timer and switch event atomically.
         */

        CyGlobalIntDisable;

        current_ticks = timer_ticks;

        event = switch_event;
        switch_event = 0;

        CyGlobalIntEnable;

        /*
         * 3. Process elapsed timer ticks.
         */

        elapsed_ticks = current_ticks - previous_ticks;

        previous_ticks = current_ticks;

        while (elapsed_ticks > 0)
        {
            elapsed_ticks--;

            game_tick();
        }

        /*
         * 4. Change screen on one switch press.
         */

        if (game_state == SCREEN_START && event)
        {
            reset_game();

            game_state = SCREEN_GAMEPLAY;

            uart_state("GAMEPLAY SCREEN");
        }

        /*
         * 5. Draw the current screen.
         */

        draw_current_screen();
    }
}
