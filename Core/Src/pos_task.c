/*
 * pos_task.c
 *
 *  Created on: Feb 19, 2026
 *      Author: moham
 */
#include "pos_task.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdint.h>

#include "modbus_regs.h"
#include "stepper.h"
#include "tb6612.h"
#include "main.h"

// HW objects
extern Stepper_t g_stepX;
extern Stepper_t g_stepY;
extern TB6612_t  g_zMotor;

// from app_control.c
extern void uart_publish_sensors_to_plc(void);

// --- tuning ---
#define X_STEP_CHUNK      5
#define Y_STEP_CHUNK      5
#define XY_STEP_DELAY_MS  2
#define Z_PWM_CYCLES      5
#define MAX_ITERS         1500

// Direction mapping (flip if needed)
#define X_DIR_POS   GPIO_PIN_SET
#define X_DIR_NEG   GPIO_PIN_RESET
#define Y_DIR_POS   GPIO_PIN_SET
#define Y_DIR_NEG   GPIO_PIN_RESET
#define Z_DIR_POS   MOTOR_FWD
#define Z_DIR_NEG   MOTOR_REV

// status bits
#define ST_BUSY  (1u << 0)
#define ST_DONE  (1u << 1)
#define ST_ERR   (1u << 2)

static inline int abs_i(int v){ return (v < 0) ? -v : v; }
static inline int in_band(int err, int tol){ return abs_i(err) <= tol; }

static void status_set(uint16_t mask, uint8_t set)
{
    uint16_t s = mb_fb[FB_STATUS];
    if (set) s |= mask; else s &= (uint16_t)~mask;
    mb_fb[FB_STATUS] = s;
}

static void done_pulse(uint32_t ms)
{
    status_set(ST_DONE, 1);
    osDelay(ms);
    status_set(ST_DONE, 0);
}

static int get_pos_mm(int *x, int *y, int *z)
{
    int xx = (int)mb_fb[FB_US1_CM10]; // V1->X
    int yy = (int)mb_fb[FB_US2_CM10]; // V2->Y
    int zz = (int)mb_fb[FB_US3_CM10]; // US->Z

    if (xx < 0 || yy < 0 || zz < 0) return 0;
    if (xx > 5000 || yy > 5000 || zz > 5000) return 0;

    *x = xx; *y = yy; *z = zz;
    return 1;
}

uint8_t MoveToTarget_RunOnce(const TargetPos_t *tgt)
{
    if (!tgt) return 0;

    status_set(ST_ERR,  0);
    status_set(ST_DONE, 0);
    status_set(ST_BUSY, 1);
    mb_fb[FB_ERR_CODE] = 0;

    printf("\r\n[MOVE] Start (SEQ): X=%d Y=%d Z=%d tol=±%d\r\n",
           tgt->x_mm, tgt->y_mm, tgt->z_mm, tgt->tol_mm);

    for (int iter = 0; iter < MAX_ITERS; iter++)
    {
        // refresh UART->mb_fb (and your [SENS] printing inside it)
        uart_publish_sensors_to_plc();

        // STOP from PLC (optional)
        if (mb_cmd[CMD_CODE] == MB_CMD_STOP) {
            mb_fb[FB_ERR_CODE] = 99;
            status_set(ST_BUSY, 0);
            status_set(ST_ERR,  1);
            printf("[MOVE] Aborted (STOP)\r\n");
            return 0;
        }

        int x,y,z;
        if (!get_pos_mm(&x,&y,&z)) {
            mb_fb[FB_ERR_CODE] = 10;
            status_set(ST_BUSY, 0);
            status_set(ST_ERR,  1);
            printf("[MOVE] Sensor invalid\r\n");
            return 0;
        }

        int dx = tgt->x_mm - x;
        int dy = tgt->y_mm - y;
        int dz = tgt->z_mm - z;

        uint8_t x_ok = in_band(dx, tgt->tol_mm);
        uint8_t y_ok = in_band(dy, tgt->tol_mm);
        uint8_t z_ok = in_band(dz, tgt->tol_mm);

        // progress print (rate-limited)
        static uint32_t lastProg = 0;
        uint32_t now = HAL_GetTick();
        if (now - lastProg >= 250) {
            lastProg = now;
            printf("[MOVE] X=%d Y=%d Z=%d | dx=%d dy=%d dz=%d | ok=%u%u%u\r\n",
                   x,y,z,dx,dy,dz, (unsigned)x_ok,(unsigned)y_ok,(unsigned)z_ok);
        }

        // done check (first thing after reading)
        if (x_ok && y_ok && z_ok)
        {
            status_set(ST_BUSY, 0);
            mb_fb[FB_ERR_CODE] = 0;
            printf("[MOVE] Reached ✅  X=%d Y=%d Z=%d\r\n", x,y,z);
            done_pulse(200);
            return 1;
        }

        // ---------------- SEQUENTIAL CONTROL ----------------
        // 1) Move X until OK
        if (!x_ok) {
            GPIO_PinState dir = (dx > 0) ? X_DIR_POS : X_DIR_NEG;
            Stepper_Step(&g_stepX, X_STEP_CHUNK, XY_STEP_DELAY_MS, dir);
            osDelay(20);
            continue;
        }

        // 2) Move Y until OK
        if (!y_ok) {
            GPIO_PinState dir = (dy > 0) ? Y_DIR_NEG : Y_DIR_POS;
            Stepper_Step(&g_stepY, Y_STEP_CHUNK, XY_STEP_DELAY_MS, dir);
            osDelay(20);
            continue;
        }

        // 3) Move Z until OK
        if (!z_ok) {
            motor_dir_t zdir = (dz > 0) ? Z_DIR_POS : Z_DIR_NEG;
            TB6612_Enable(&g_zMotor, 1);
            TB6612_SetDir(&g_zMotor, zdir);
            for (int i = 0; i < Z_PWM_CYCLES; i++) {
                TB6612_SoftPwmOnce(&g_zMotor, 80, 50);
            }
            TB6612_Enable(&g_zMotor, 0);
            osDelay(20);
            continue;
        }

        // If we get here, something is inconsistent, but keep looping safely
        osDelay(20);
    }

    mb_fb[FB_ERR_CODE] = 11;
    status_set(ST_BUSY, 0);
    status_set(ST_ERR,  1);
    printf("[MOVE] Timeout ❌\r\n");
    return 0;
}
