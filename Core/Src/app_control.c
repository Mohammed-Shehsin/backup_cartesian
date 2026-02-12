/*
 * app_control.c
 */

#include "app_control.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdint.h>

#include "stepper.h"
#include "tb6612.h"
#include "magnet.h"
#include "hcsr04.h"
#include "modbus_regs.h"   // mb_cmd[], mb_fb[], CMD_*, FB_*, MB_CMD_*
#include "freertos.h"
#include "cmsis_os.h"
#include "freertos_user.h"

extern Stepper_t g_stepX;
extern Stepper_t g_stepY;
extern TB6612_t  g_zMotor;
extern Magnet_t  g_magnet;
extern HCSR04_t  g_us1, g_us2, g_us3;

// FB_STATUS bits
#define ST_BUSY  (1u << 0)
#define ST_DONE  (1u << 1)
#define ST_ERR   (1u << 2)

#define XY_STEP_DELAY_MS  2


// ---- position tracking (in steps) ----
static int32_t g_pos_x_steps = 0;
static int32_t g_pos_y_steps = 0;

// OPTIONAL: convert steps to mm (set these correctly for your mechanics)
// Example: lead screw 8mm/rev, motor 200 steps/rev -> 0.04 mm/step
// #define X_MM_PER_STEP_NUM  4     // 0.04 = 4/100
// #define X_MM_PER_STEP_DEN  100
// If you don’t know yet, keep steps.
static int16_t clamp_i16(int32_t v) {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static inline GPIO_PinState dir_from_bit(uint16_t bits, uint16_t bitIndex) {
    return (bits & (1u << bitIndex)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static inline motor_dir_t zdir_from_bit(uint16_t bits) {
    return (bits & (1u << 2)) ? MOTOR_FWD : MOTOR_REV;
}

static void set_status_bits(uint16_t mask, uint8_t set) {
    uint16_t s = mb_fb[FB_STATUS];
    if (set) s |= mask; else s &= (uint16_t)~mask;
    mb_fb[FB_STATUS] = s;
}

static void pulse_done(uint32_t ms) {
    set_status_bits(ST_DONE, 1);
    osDelay(ms);
    set_status_bits(ST_DONE, 0);
}

// Convert cm->mm (rounded) but your HCSR04 returns cm from pulse_us/58.
// Better: compute mm directly from pulse time. For now, convert safely here.
static int16_t cm_to_mm_i16(float cm)
{
    if (cm < 0) return (int16_t)cm;     // keep -1,-2,-3 error codes
    float mm = cm * 10.0f;              // cm -> mm
    if (mm > 32767.0f) mm = 32767.0f;
    return (int16_t)(mm + 0.5f);
}


static void update_sensors_once(void)
{
    float d1 = HCSR04_ReadCmSafe(&g_us1, 60);
    osDelay(70);

    float d2 = HCSR04_ReadCmSafe(&g_us2, 60);
    osDelay(70);

    float d3 = HCSR04_ReadCmSafe(&g_us3, 60);
    osDelay(70);

    mb_fb[FB_US1_CM10] = (uint16_t)cm_to_mm_i16(d1);
    mb_fb[FB_US2_CM10] = (uint16_t)cm_to_mm_i16(d2);
    mb_fb[FB_US3_CM10] = (uint16_t)cm_to_mm_i16(d3);


    // PRINT IN MM (cm × 10 = mm)
    printf("US1=%u mm | US2=%u mm | US3=%u mm\r\n",
           mb_fb[FB_US1_CM10],
           mb_fb[FB_US2_CM10],
           mb_fb[FB_US3_CM10]);
}


static void execute_move(uint16_t sx, uint16_t sy, uint16_t sz_cycles,
                         uint16_t dir_bits, uint16_t mag)
{
    // Magnet
    if (mag) Magnet_On(&g_magnet);
    else     Magnet_Off(&g_magnet);

//    // X
//    if (sx > 0) {
//        GPIO_PinState dx = dir_from_bit(dir_bits, 0);
//        Stepper_Step(&g_stepX, (int)sx, XY_STEP_DELAY_MS, dx);
//        g_pos_x_steps += (dx == GPIO_PIN_SET) ? (int32_t)sx : -(int32_t)sx;
//    }
//
//    // Y
//    if (sy > 0) {
//        GPIO_PinState dy = dir_from_bit(dir_bits, 1);
//        Stepper_Step(&g_stepY, (int)sy, XY_STEP_DELAY_MS, dy);
//        g_pos_y_steps += (dy == GPIO_PIN_SET) ? (int32_t)sy : -(int32_t)sy;
//    }
    // X position feedback in steps
    if (sx > 0) {
        int16_t dx = dir_from_bit(dir_bits, 0) == GPIO_PIN_SET ? (int16_t)sx : -(int16_t)sx;
        mb_fb[FB_POS_X] = (uint16_t)((int16_t)mb_fb[FB_POS_X] + dx);
    }

    // Y position feedback in steps
    if (sy > 0) {
        int16_t dy = dir_from_bit(dir_bits, 1) == GPIO_PIN_SET ? (int16_t)sy : -(int16_t)sy;
        mb_fb[FB_POS_Y] = (uint16_t)((int16_t)mb_fb[FB_POS_Y] + dy);
    }

    // Z (cycles)
    if (sz_cycles > 0) {
        TB6612_Enable(&g_zMotor, 1);
        TB6612_SetDir(&g_zMotor, zdir_from_bit(dir_bits));
        for (uint16_t i = 0; i < sz_cycles; i++) TB6612_SoftPwmOnce(&g_zMotor, 80, 50);
        TB6612_Enable(&g_zMotor, 0);
    }

    // publish positions (currently in steps; change to mm when you know mm/step)
    mb_fb[FB_POS_X] = (uint16_t)(int16_t)clamp_i16(g_pos_x_steps);
    mb_fb[FB_POS_Y] = (uint16_t)(int16_t)clamp_i16(g_pos_y_steps);
}

extern void AutoTest_RunOnce(void);

static void print_windows(const char *tag)
{
    printf("\r\n[%s] CMD: ", tag);
    for (uint16_t i = 0; i < MB_WINDOW_LEN; i++) {
        printf("%u", mb_cmd[i]);
        if (i != (MB_WINDOW_LEN-1)) printf(",");
    }
    printf("\r\n[%s]  FB: ", tag);
    for (uint16_t i = 0; i < MB_WINDOW_LEN; i++) {
        printf("%u", mb_fb[i]);
        if (i != (MB_WINDOW_LEN-1)) printf(",");
    }
    printf("\r\n");
}

void Control_Task(void const *argument)
{
    (void)argument;
//   osSemaphoreWait(lwipReadySem, osWaitForever);
// b osDelay(200);
    mb_fb[FB_STATUS]   = 0;
    mb_fb[FB_ERR_CODE] = 0;
    mb_fb[FB_POS_X]    = 0;
    mb_fb[FB_POS_Y]    = 0;

    uint16_t last_cmd_win[MB_WINDOW_LEN];
    for (uint16_t i = 0; i < MB_WINDOW_LEN; i++) last_cmd_win[i] = 0xFFFF;

    printf("\r\n[CTRL] started\r\n");

    for (;;)
    {
        update_sensors_once();

        // detect change in CMD window -> print
        uint8_t changed = 0;
        for (uint16_t i = 0; i < MB_WINDOW_LEN; i++) {
            if (mb_cmd[i] != last_cmd_win[i]) { changed = 1; break; }
        }
        if (changed) {
            for (uint16_t i = 0; i < MB_WINDOW_LEN; i++) last_cmd_win[i] = mb_cmd[i];
            print_windows("NEW");
        }

        uint16_t cmd = mb_cmd[CMD_CODE];

        if (cmd != MB_CMD_IDLE)
        {
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
            else if (cmd == MB_CMD_STOP) {
                TB6612_Enable(&g_zMotor, 0);
                Magnet_Off(&g_magnet);
                set_status_bits(ST_BUSY, 0);
                pulse_done(200);
            }
            else {
                mb_fb[FB_ERR_CODE] = 1;
                set_status_bits(ST_ERR,  1);
                set_status_bits(ST_BUSY, 0);
            }

            // critical: clear so command runs once
            mb_cmd[CMD_CODE] = MB_CMD_IDLE;
        }

        osDelay(50);
    }
}
