#include <stdint.h>
#include <string.h>

#include "cmsis_os2.h"
#include "ble_uart.h"
#include "board.h"
#include "lib.h"
#include "servo.h"
#include "controller.h"
#include "estimator.h"

/* OS objects */
osThreadId_t ble_task;

/* Buffer to hold the command received from UART or BLE
 * We use single buffer assuming command-response protocol,
 * that the next command will be sent after receiving the
 * response for the current command.
 */
uint8_t cmd_buf[256];
uint32_t cmd_len;

uint8_t frame_buffer[LED_NUM_ROWS][LED_NUM_COLS] =
{
    { 0, 1, 0, 1, 0},
    { 0, 0, 1, 0, 0},
    { 1, 0, 0, 0, 1},
    { 0, 1, 1, 1, 0},
    { 0, 0, 0, 0, 0}
};

#define MAX_COUNT 100
static int count;

uint32_t r, c;
uint32_t r0, c0;

/* Called from BLE softdevice using SWI2_EGU2_IRQHandler */
static void ble_recv_handler(const uint8_t s[], uint32_t len)
{
    /* Make a local copy so that BLE can receive next characters */
    memcpy(cmd_buf, s, len);

    /* Remove trailing new line or carriange return characters. */
    while ((s[len - 1] == '\n') || (s[len - 1] == '\r'))
        len--;
    cmd_buf[len] = '\0';            // null-terminate the string
    cmd_len = len;

    /* Signal the waiting task. */
    osThreadFlagsSet(ble_task, 1); 
}

static void task1(void *arg)
{
    printf("hello, task1!\n");
    
    while (1)
    {
        // forward(1);
        led_display(frame_buffer);
        count++;
        if (count == MAX_COUNT)
        {
            count = 0;
            osThreadYield();
        }
    }
}

// static void task2(void *arg)
// {
//     printf("hello, task2!\n");

//     while (1)
//     {
//         reverse(1);
//         led_display(frame_buffer);
//         count++;
//         if (count == MAX_COUNT)
//         {
//             count = 0;
//             osThreadYield();
//         }
//     }
// }

void bluetooth(void *arg)
{
    while (1)
    {
        /* Receive a command from BLE */
        osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);

        /* Echo on UART */
        puts((char *) cmd_buf);
        puts("\n");

        /* Echo on BLE */
        ble_send((uint8_t *) cmd_buf, strlen((char *) cmd_buf));

        /* Buggy Control */
        if (strlen((char *) cmd_buf) == 1)
        {
            switch (cmd_buf[0])
            {
            case 'u':
                puts("Move forward\n");
                forward(1);
                break;
            case 'd':
                puts("Move reverse\n");
                reverse(1);
                break;
            case 'l':
                puts("Turn left\n");
                turn_left(1);
                break;
            case 'r':
                puts("Turn right\n");
                turn_right(1);
                break;
            default:
                break;
            }
        }
    }
}

void task_ctrl(void *arg)
{
    osThreadId_t tid1;
    // osThreadId_t tid2;

    tid1 = osThreadNew(task1, NULL, NULL);
    // tid2 = osThreadNew(task2, NULL, NULL);

    osThreadSetPriority(tid1, osPriorityNormal);
    // osThreadSetPriority(tid2, osPriorityNormal);

    ble_task = osThreadNew(bluetooth, NULL, NULL);
    osThreadSetPriority(ble_task, osPriorityLow);
}

void task_imu(void *arg){
    float accData[3];
    float magData[3];
    float angles[3];
    float heading;
    while (1){
        accReadXYZ(accData);
        magReadXYZ(magData);
        // estimate_angles(accData, magData, angles);
        heading = estimate_heading(accData, magData, angles);

        // printf("X:");
        // print_float(accData[0],4);
        // printf(",Y:");
        // print_float(accData[1],4);
        // printf("Z:");
        // print_float(accData[2],4);
        // printf("X:");
        // print_float(magData[0],4);
        // printf(",Y:");
        // print_float(magData[1],4);
        // printf("Z:");
        // print_float(magData[2],4);
        // printf(",");
        printf("X:");
        print_float(angles[0],4);
        printf(",Y:");
        print_float(angles[1],4);
        printf(",Z:");
        print_float(angles[2],4);
        printf(",Heading:");
        print_float(heading, 4);

        float val = movAvg(heading);
        printf(", AVG:");
        print_float(val,4);
        printf("\n");
    }
}

int main(void)
{
    osThreadId_t tid_ctrl;
    /* BSP initializations before BLE because we are using printf from BSP */
    board_init();
    ble_init(ble_recv_handler);

    /* Greetings */
    printf("hello, world!\n");
    audio_sweep(500, 2000, 100);

    /* Initialize and start the kernel */
    osKernelInitialize();

    /* controller task */
    tid_ctrl = osThreadNew(task_ctrl, NULL, NULL);
    osThreadSetPriority(tid_ctrl, osPriorityNormal);

    osKernelStart();
    /* never returns */

    led_blink(2, 2, BLINK_FOREVER);

    return 0;
}