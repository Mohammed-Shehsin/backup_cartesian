/*
 * uart3_link.c
 */

#include "uart3_link.h"
#include <string.h>
#include <stdlib.h>   // for strtol


#ifndef UART3_LINK_LINE_MAX
#define UART3_LINK_LINE_MAX 96
#endif

static UART_HandleTypeDef *s_huart = NULL;

static uint8_t  s_rx_ch;
static char     s_buf[UART3_LINK_LINE_MAX];
static uint32_t s_idx = 0;

// "last complete line"
static char     s_line[UART3_LINK_LINE_MAX];
static volatile bool s_has_line = false;

// optional integer parse support
static volatile bool    s_has_new_int = false;
static volatile int32_t s_latest_int  = 0;

static void arm_rx(void)
{
    if (s_huart)
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
}

void UART3_Link_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_idx = 0;
    memset(s_buf, 0, sizeof(s_buf));
    memset(s_line, 0, sizeof(s_line));

    s_has_line = false;
    s_has_new_int = false;
    s_latest_int = 0;

    arm_rx();
}

void UART3_Link_OnRxByteIRQ(UART_HandleTypeDef *huart)
{
    if (!s_huart || !huart) return;
    if (huart->Instance != s_huart->Instance) return;

    char c = (char)s_rx_ch;

    // Treat \r or \n as end-of-line
    if (c == '\n' || c == '\r')
    {
        if (s_idx > 0)
        {
            // terminate current buffer
            if (s_idx >= UART3_LINK_LINE_MAX) s_idx = UART3_LINK_LINE_MAX - 1;
            s_buf[s_idx] = '\0';

            // copy to last line (atomic-ish for reader)
            strncpy(s_line, s_buf, UART3_LINK_LINE_MAX - 1);
            s_line[UART3_LINK_LINE_MAX - 1] = '\0';
            s_has_line = true;

            // optional: parse integer inside line
            // find first digit or '-' then parse
            const char *p = s_line;
            while (*p && ((*p < '0' || *p > '9') && *p != '-')) p++;
            if (*p) {
                s_latest_int = (int32_t)strtol(p, NULL, 10);
                s_has_new_int = true;
            }

            // reset for next line
            s_idx = 0;
        }
        // else ignore empty lines
    }
    else
    {
        // store character
        if (s_idx < (UART3_LINK_LINE_MAX - 1))
        {
            s_buf[s_idx++] = c;
        }
        else
        {
            // overflow -> reset buffer to avoid stuck state
            s_idx = 0;
        }
    }

    arm_rx();
}

bool UART3_Link_GetLine(char *dst, uint32_t dst_len)
{
    if (!dst || dst_len == 0) return false;

    if (!s_has_line) return false;

    // copy out last completed line
    strncpy(dst, s_line, dst_len - 1);
    dst[dst_len - 1] = '\0';

    // consume the line
    s_has_line = false;
    return true;
}

bool UART3_Link_GetLatestInt(int32_t *out_value)
{
    if (!out_value) return false;

    if (s_has_new_int)
    {
        *out_value = s_latest_int;
        s_has_new_int = false;
        return true;
    }
    return false;
}

const char* UART3_Link_GetLastLine(void)
{
    return s_line;
}

void UART3_Link_OnUartErrorIRQ(UART_HandleTypeDef *huart)
{
    if (!s_huart || !huart) return;
    if (huart->Instance != s_huart->Instance) return;

    // Clear error and restart RX
    (void)HAL_UART_AbortReceive_IT(s_huart);
    arm_rx();
}
