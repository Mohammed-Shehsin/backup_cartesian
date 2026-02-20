#include "pick_task.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdint.h>

#include "modbus_regs.h"
#include "pos_task.h"
#include "tb6612.h"
#include "magnet.h"

extern TB6612_t  g_zMotor;
extern Magnet_t  g_magnet;

// from app_control.c
extern void uart_publish_sensors_to_plc(void);

// --------- PICK SETTINGS ----------
#define PICK_X_MM           100
#define PICK_Y_MM           300
#define PICK_TOL_MM         10

#define PICK_SAFE_Z_MM      95
#define PICK_SAFE_TOL_MM    10

// “near the block” based on US (mm). Tune!
#define PICK_NEAR_MM        60

#define Z_PWM_CYCLES        5
#define PICK_MAX_ITERS      2000

// Lift time (2.5 seconds)
#define LIFT_TIME_MS        2500
#define LIFT_BURST_DELAY_MS 20

// IMPORTANT: Flip these if direction is wrong
#define Z_DOWN_DIR          MOTOR_FWD
#define Z_UP_DIR            MOTOR_REV

static inline void z_burst(motor_dir_t dir)
{
    TB6612_Enable(&g_zMotor, 1);
    TB6612_SetDir(&g_zMotor, dir);
    for (int i = 0; i < Z_PWM_CYCLES; i++) {
        TB6612_SoftPwmOnce(&g_zMotor, 80, 50);
    }
    TB6612_Enable(&g_zMotor, 0);
}

uint8_t Pick_RunOnce(void)
{
    mb_fb[FB_ERR_CODE] = 0;
    printf("\r\n[PICK] Start\r\n");

    // Magnet ON from the start
    Magnet_On(&g_magnet);
    printf("[PICK] Magnet ON (start)\r\n");

    // 1) Go to pick XY at SAFE Z
    TargetPos_t pre = {
        .x_mm = PICK_X_MM,
        .y_mm = PICK_Y_MM,
        .z_mm = PICK_SAFE_Z_MM,
        .tol_mm = PICK_TOL_MM
    };
    if (!MoveToTarget_RunOnce(&pre)) {
        mb_fb[FB_ERR_CODE] = 31;
        printf("[PICK] FAIL: move to pre-pick\r\n");
        return 0;
    }

    // 2) Z DOWN until near block
    for (int iter = 0; iter < PICK_MAX_ITERS; iter++)
    {
        uart_publish_sensors_to_plc();

        if (mb_cmd[CMD_CODE] == MB_CMD_STOP) {
            mb_fb[FB_ERR_CODE] = 99;
            printf("[PICK] Aborted (STOP)\r\n");
            return 0;
        }

        uint16_t us_mm = mb_fb[FB_US3_CM10];

        static uint32_t last = 0;
        uint32_t now = HAL_GetTick();
        if (now - last >= 250) {
            last = now;
            printf("[PICK] DOWN  US=%u mm (need <= %u)\r\n",
                   (unsigned)us_mm, (unsigned)PICK_NEAR_MM);
        }

        if (us_mm <= PICK_NEAR_MM) {
            printf("[PICK] Near block ✅ (US=%u)\r\n", (unsigned)us_mm);
            break;
        }

        z_burst(Z_DOWN_DIR);
        osDelay(20);

        if (iter == (PICK_MAX_ITERS - 1)) {
            mb_fb[FB_ERR_CODE] = 21;
            printf("[PICK] FAIL: down timeout (US=%u)\r\n", (unsigned)us_mm);
            return 0;
        }
    }

    // 3) LIFT UP for fixed 2.5 seconds (magnet stays ON)
    printf("[PICK] UP (time-based) 2.5s\r\n");
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < LIFT_TIME_MS)
    {
        uart_publish_sensors_to_plc();

        if (mb_cmd[CMD_CODE] == MB_CMD_STOP) {
            mb_fb[FB_ERR_CODE] = 99;
            printf("[PICK] Aborted (STOP) during LIFT\r\n");
            return 0;
        }

        z_burst(Z_UP_DIR);
        osDelay(LIFT_BURST_DELAY_MS);
    }

    printf("[PICK] Done ✅ (magnet ON)\r\n");

    // Your request: after finishing, cmd should be 99
    // (Note: Control_Task may overwrite this to IDLE after returning)
    mb_cmd[CMD_CODE] = 99;

    return 1;
}
