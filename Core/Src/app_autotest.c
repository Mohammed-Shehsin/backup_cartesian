#include "app_autotest.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "modbus_regs.h"   // mb_cmd[], CMD_CODE index etc.

// If you use symbolic command codes, define them ONE place (preferably modbus_regs.h)
#ifndef CMD_STOP
#define CMD_STOP 99
#endif

extern Stepper_t g_stepX;
extern Stepper_t g_stepY;
extern TB6612_t  g_zMotor;
extern Magnet_t  g_magnet;
extern HCSR04_t  g_us1, g_us2, g_us3;

static inline int should_abort(void)
{
    return (mb_cmd[CMD_CODE] == CMD_STOP);
}

static void PrintSensors(void)
{
    float d1 = HCSR04_ReadCmSafe(&g_us1, 60); osDelay(70);
    float d2 = HCSR04_ReadCmSafe(&g_us2, 60); osDelay(70);
    float d3 = HCSR04_ReadCmSafe(&g_us3, 60); osDelay(70);

    int i1 = (d1 < 0) ? (int)d1 : (int)(d1 + 0.5f);
    int i2 = (d2 < 0) ? (int)d2 : (int)(d2 + 0.5f);
    int i3 = (d3 < 0) ? (int)d3 : (int)(d3 + 0.5f);

    printf("S1:%d S2:%d S3:%d | edges:%lu %lu %lu\r\n",
           i1, i2, i3,
           (unsigned long)g_us1.edges,
           (unsigned long)g_us2.edges,
           (unsigned long)g_us3.edges);
}

static void Z_Run(motor_dir_t dir, int cycles)
{
    TB6612_Enable(&g_zMotor, 1);
    TB6612_SetDir(&g_zMotor, dir);

    for (int i = 0; i < cycles; i++) {
        if (should_abort()) break;
        TB6612_SoftPwmOnce(&g_zMotor, 80, 50);
        osDelay(1); // yield
    }

    TB6612_Enable(&g_zMotor, 0);
}

void AutoTest_RunOnce(void)
{
    printf("\r\n=== AUTO TEST START ===\r\n");

    if (should_abort()) goto abort;
    printf("[0] Sensors baseline\r\n");
    PrintSensors();

    if (should_abort()) goto abort;
    printf("[1] X stepper forward/back\r\n");
    Stepper_Step(&g_stepX, 800, 2, GPIO_PIN_SET);
    osDelay(200);
    PrintSensors();

    if (should_abort()) goto abort;
    Stepper_Step(&g_stepX, 800, 2, GPIO_PIN_RESET);
    osDelay(200);
    PrintSensors();

    if (should_abort()) goto abort;
    printf("[2] Y stepper forward/back\r\n");
    Stepper_Step(&g_stepY, 2000, 2, GPIO_PIN_SET);
    osDelay(200);
    PrintSensors();

    if (should_abort()) goto abort;
    Stepper_Step(&g_stepY, 2000, 2, GPIO_PIN_RESET);
    osDelay(200);
    PrintSensors();

    if (should_abort()) goto abort;
    printf("[3] Z DC motor forward/back\r\n");
    Z_Run(MOTOR_FWD, 120);
    osDelay(200);
    PrintSensors();

    if (should_abort()) goto abort;
    Z_Run(MOTOR_REV, 120);
    osDelay(200);
    PrintSensors();

    if (should_abort()) goto abort;
    printf("[4] Magnet on/off\r\n");
    Magnet_On(&g_magnet);
    osDelay(300);
    PrintSensors();

    if (should_abort()) goto abort;
    Magnet_Off(&g_magnet);
    osDelay(300);
    PrintSensors();

    printf("=== AUTO TEST END ===\r\n\r\n");
    return;

abort:
    printf("=== AUTO TEST ABORTED (CMD_STOP) ===\r\n\r\n");
    // safe shutdown
    TB6612_Enable(&g_zMotor, 0);
    Magnet_Off(&g_magnet);
}
