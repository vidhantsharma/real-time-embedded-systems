#include <stdint.h>
#include <string.h>

#include "cmsis_os2.h"
#include "ble_uart.h"
#include "board.h"
#include "lib.h"
#include "servo.h"
#include "controller.h"
#include "estimator.h"
#include "lsm303agr.h"

#define FLAGS_MSK1 0x00000001U

/* OS objects */
osThreadId_t ble_task;
osEventFlagsId_t evt_frwd, evt_bwd, evt_left, evt_ryt, evt_stop;

/* Buffer to hold the command received from UART or BLE
 * We use single buffer assuming command-response protocol,
 * that the next command will be sent after receiving the
 * response for the current command.
 */
uint8_t cmd_buf[256];
uint32_t cmd_len;
float heading, val;
float ang_des;

uint8_t frame_buffer[LED_NUM_ROWS][LED_NUM_COLS] =
{
    { 0, 1, 0, 1, 0},
    { 0, 0, 1, 0, 0},
    { 1, 0, 0, 0, 1},
    { 0, 1, 1, 1, 0},
    { 0, 0, 0, 0, 0}
};

#define MAX_COUNT 50
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

void task_imu(void *arg){
    float accData[3];
    float magData[3];
    float angles[3];
    // printf("hello, task_imu!!!!\n");
    while (1){
        accReadXYZ(accData);
        magReadXYZ(magData);
        heading = estimate_heading(accData, magData, angles);
        val = movAvg(heading);
        // printf("\n AVG:");
        // print_float(val,4);
        
        count++;
        if (count == MAX_COUNT)
        {
            count = 0;
            osThreadYield();
        }
    }
}

void task_cmd(void *arg){   // Only turning
    
    while(1){
        if(osEventFlagsGet(evt_frwd)){
            move_ctrlr(1);
            printf("\nFrwd flag, %d", osEventFlagsGet(evt_frwd));
        }
        else if(osEventFlagsGet(evt_bwd))
            move_ctrlr(-1);
        else if(osEventFlagsGet(evt_left))
            turn_ctrlr(ang_des, val);
        else if(osEventFlagsGet(evt_ryt))
            turn_ctrlr(ang_des, val); 
        else if(osEventFlagsGet(evt_stop))
            move_ctrlr(0);
        
        count++;
        if (count == MAX_COUNT)
        {
            count = 0;
            osThreadYield();
        }
    }
}

void bluetooth(void *arg){
    while (1)
    {
        /* Receive a command from BLE */
        osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);

        // osEventFlagsClear(evt_frwd, FLAGS_MSK1); osEventFlagsClear(evt_bwd, FLAGS_MSK1);
        // osEventFlagsClear(evt_left, FLAGS_MSK1); osEventFlagsClear(evt_ryt, FLAGS_MSK1); osEventFlagsClear(evt_stop, FLAGS_MSK1);
        
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
                osEventFlagsSet(evt_frwd, FLAGS_MSK1);
                break;
            case 'd':
                puts("Move reverse\n");
                osEventFlagsSet(evt_bwd, FLAGS_MSK1);
                break;
            case 'l':
                puts("Turn left\n");
                ang_des = 90;
                osEventFlagsSet(evt_left, FLAGS_MSK1);
                break;
            case 'r':
                puts("Turn right\n");
                ang_des = 270.0; // Range: [0,360)
                osEventFlagsSet(evt_ryt, FLAGS_MSK1);
                break;
            case 's':
                puts("STOP!!\n");
                osEventFlagsSet(evt_stop, FLAGS_MSK1);
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

    tid1 = osThreadNew(task_imu, NULL, NULL);
    osThreadSetPriority(tid1, osPriorityNormal);
    
    ble_task = osThreadNew(bluetooth, NULL, NULL);
    osThreadSetPriority(ble_task, osPriorityNormal);

    evt_frwd = osEventFlagsNew(NULL); evt_bwd = osEventFlagsNew(NULL);
    evt_left = osEventFlagsNew(NULL); evt_ryt = osEventFlagsNew(NULL); evt_stop = osEventFlagsNew(NULL);
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
    osThreadSetPriority(tid_ctrl, osPriorityLow);

    osKernelStart();
    /* never returns */

    led_blink(2, 2, BLINK_FOREVER);

    return 0;
}