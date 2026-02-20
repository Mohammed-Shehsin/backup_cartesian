/*
 * app_control.c
 */
#include "app_control.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "stepper.h"
#include "tb6612.h"
#include "magnet.h"
#include "modbus_regs.h"
#include "freertos.h"
#include "freertos_user.h"
#include "home_task.h"
#include "box1_task.h"
#include "box2_task.h"
#include "pick_task.h"
#include "uart3_link.h"

extern UART_HandleTypeDef huart2;   // Arduino link
extern UART_HandleTypeDef huart3;   // PuTTY/printf if used

extern Stepper_t g_stepX;
extern Stepper_t g_stepY;
extern TB6612_t  g_zMotor;
extern Magnet_t  g_magnet;

#define ST_BUSY  (1u << 0)
#define ST_DONE  (1u << 1)
#define ST_ERR   (1u << 2)

#define XY_STEP_DELAY_MS  2

static int32_t g_pos_x_steps = 0;
static int32_t g_pos_y_steps = 0;

volatile uint16_t g_dbg_v1_mm = 0;
volatile uint16_t g_dbg_v2_mm = 0;
volatile uint16_t g_dbg_us_mm = 0;
volatile uint32_t g_dbg_rx_ok  = 0;
volatile uint32_t g_dbg_rx_bad = 0;

static int16_t clamp_i16(int32_t v){
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static inline GPIO_PinState dir_from_bit(uint16_t bits, uint16_t bitIndex){
    return (bits & (1u << bitIndex)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static inline motor_dir_t zdir_from_bit(uint16_t bits){
    return (bits & (1u << 2)) ? MOTOR_FWD : MOTOR_REV;
}

static void set_status_bits(uint16_t mask, uint8_t set){
    uint16_t s = mb_fb[FB_STATUS];
    if (set) s |= mask; else s &= (uint16_t)~mask;
    mb_fb[FB_STATUS] = s;
}

static void pulse_done(uint32_t ms){
    set_status_bits(ST_DONE, 1);
    osDelay(ms);
    set_status_bits(ST_DONE, 0);
}

/*
 * Arduino sends: "V1=123,V2=456,US=789\n" (mm)
 * This prints ALWAYS (rate limited) using last latched values.
 */
void uart_publish_sensors_to_plc(void)
{
    static uint32_t lastPrint = 0;
    static uint32_t lastBadPrint = 0;

    char line[64];
    uint8_t got_line = UART3_Link_GetLine(line, sizeof(line));

    if (got_line) {
        int v1=-1,v2=-1,us=-1;
        if (sscanf(line, "V1=%d,V2=%d,US=%d", &v1, &v2, &us) == 3) {

            if (v1 < 0) v1 = 0; if (v1 > 65535) v1 = 65535;
            if (v2 < 0) v2 = 0; if (v2 > 65535) v2 = 65535;
            if (us < 0) us = 0; if (us > 65535) us = 65535;

            mb_fb[FB_US1_CM10] = (uint16_t)v1;
            mb_fb[FB_US2_CM10] = (uint16_t)v2;
            mb_fb[FB_US3_CM10] = (uint16_t)us;

            g_dbg_v1_mm = (uint16_t)v1;
            g_dbg_v2_mm = (uint16_t)v2;
            g_dbg_us_mm = (uint16_t)us;
            g_dbg_rx_ok++;

            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        } else {
            g_dbg_rx_bad++;
            uint32_t now = HAL_GetTick();
            if (now - lastBadPrint >= 500) {
                lastBadPrint = now;
                printf("[SENS] Parse fail: %s\r\n", line);
            }
        }
    }

    uint32_t now = HAL_GetTick();
    if (now - lastPrint >= 200) {
        lastPrint = now;
        printf("[SENS] V1=%u mm, V2=%u mm, US=%u mm | ok=%lu bad=%lu\r\n",
               (unsigned)mb_fb[FB_US1_CM10],
               (unsigned)mb_fb[FB_US2_CM10],
               (unsigned)mb_fb[FB_US3_CM10],
               (unsigned long)g_dbg_rx_ok,
               (unsigned long)g_dbg_rx_bad);
    }
}

static void execute_move(uint16_t sx, uint16_t sy, uint16_t sz_cycles,
                         uint16_t dir_bits, uint16_t mag)
{
    if (mag) Magnet_On(&g_magnet);
    else     Magnet_Off(&g_magnet);

    if (sx > 0) {
        GPIO_PinState dx = dir_from_bit(dir_bits, 0);
        Stepper_Step(&g_stepX, (int)sx, XY_STEP_DELAY_MS, dx);
        g_pos_x_steps += (dx == GPIO_PIN_SET) ? (int32_t)sx : -(int32_t)sx;
    }

    if (sy > 0) {
        GPIO_PinState dy = dir_from_bit(dir_bits, 1);
        Stepper_Step(&g_stepY, (int)sy, XY_STEP_DELAY_MS, dy);
        g_pos_y_steps += (dy == GPIO_PIN_SET) ? (int32_t)sy : -(int32_t)sy;
    }

    if (sz_cycles > 0) {
        TB6612_Enable(&g_zMotor, 1);
        TB6612_SetDir(&g_zMotor, zdir_from_bit(dir_bits));
        for (uint16_t i = 0; i < sz_cycles; i++) TB6612_SoftPwmOnce(&g_zMotor, 80, 50);
        TB6612_Enable(&g_zMotor, 0);
    }

    mb_fb[FB_POS_X] = (uint16_t)(int16_t)clamp_i16(g_pos_x_steps);
    mb_fb[FB_POS_Y] = (uint16_t)(int16_t)clamp_i16(g_pos_y_steps);
}

extern void AutoTest_RunOnce(void);

void Control_Task(void const *argument)
{
    (void)argument;

    mb_fb[FB_STATUS]   = 0;
    mb_fb[FB_ERR_CODE] = 0;
    mb_fb[FB_POS_X]    = 0;
    mb_fb[FB_POS_Y]    = 0;

    UART3_Link_Init(&huart2);

    printf("\r\n[CTRL] started (USART2=Arduino, USART3=PuTTY)\r\n");

    uint16_t last_cmd = MB_CMD_IDLE;

    for (;;)
    {
        uint16_t cmd = mb_cmd[CMD_CODE];

        // keep printing sensors when idle
        if (cmd == MB_CMD_IDLE) {
            uart_publish_sensors_to_plc();
            last_cmd = MB_CMD_IDLE;
            osDelay(20);
            continue;
        }

        // ANTI-REPEAT: ignore same cmd again and again (except STOP / 99)
        if ((cmd == last_cmd) && (cmd != MB_CMD_STOP) && (cmd != 99)) {
            // clear it so it doesn't keep retriggering
            mb_cmd[CMD_CODE] = MB_CMD_IDLE;
            osDelay(20);
            continue;
        }
        last_cmd = cmd;

        set_status_bits(ST_ERR,  0);
        set_status_bits(ST_DONE, 0);
        set_status_bits(ST_BUSY, 1);
        mb_fb[FB_ERR_CODE] = 0;

        if (cmd == MB_CMD_EXEC_MOVE) {
            execute_move(mb_cmd[CMD_STEP_X], mb_cmd[CMD_STEP_Y], mb_cmd[CMD_STEP_Z],
                         mb_cmd[CMD_DIR_BITS], mb_cmd[CMD_MAGNET]);
            set_status_bits(ST_BUSY, 0);
            pulse_done(200);
        }
        else if (cmd == MB_CMD_AUTOTEST) {
            AutoTest_RunOnce();
            set_status_bits(ST_BUSY, 0);
            pulse_done(200);
        }
        else if (cmd == MB_CMD_HOME) {
            uint8_t ok = Home_RunOnce();
            set_status_bits(ST_BUSY, 0);
            if (ok) pulse_done(200); else set_status_bits(ST_ERR, 1);
        }
        else if (cmd == MB_CMD_BOX1) {
            uint8_t ok = Box1_RunOnce();
            set_status_bits(ST_BUSY, 0);
            if (ok) pulse_done(200); else set_status_bits(ST_ERR, 1);
        }
        else if (cmd == MB_CMD_BOX2) {
            uint8_t ok = Box2_RunOnce();
            set_status_bits(ST_BUSY, 0);
            if (ok) pulse_done(200); else set_status_bits(ST_ERR, 1);
        }
        else if (cmd == MB_CMD_PICK) {
            uint8_t ok = Pick_RunOnce();
            set_status_bits(ST_BUSY, 0);
            if (ok) pulse_done(200); else set_status_bits(ST_ERR, 1);
        }
        else if (cmd == MB_CMD_STOP || cmd == 99) {
            TB6612_Enable(&g_zMotor, 0);
            Magnet_Off(&g_magnet);
            set_status_bits(ST_BUSY, 0);
            pulse_done(200);
        }
        else {
            mb_fb[FB_ERR_CODE] = 1;
            set_status_bits(ST_ERR, 1);
            set_status_bits(ST_BUSY, 0);
        }

        mb_cmd[CMD_CODE] = MB_CMD_IDLE;
        osDelay(20);
    }
}
