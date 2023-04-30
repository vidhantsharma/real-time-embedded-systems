#include <stdint.h>
#include <string.h>

#include "cmsis_os2.h"
#include "ble_uart.h"
#include "board.h"
#include "lib.h"
#include "estimator.h"

#define FLAGS_MSK1 0x00000001U
#define FLAGS_MSK2 0x00000002U
#define FLAGS_MSK3 0x00000003U

/* OS objects */
osThreadId_t ble_task;
osEventFlagsId_t evt_frwd, evt_bwd, evt_left, evt_ryt, evt_stop;
extern osEventFlagsId_t evt_clap;
extern osEventFlagsId_t sid;
osTimerId_t timer_clap;
osThreadId_t tid1, tid2;
// extern osSemaphoreId_t sid_killswitch;  
// extern osMutexId_t mid_killswitch;

/* Buffer to hold the command received from UART or BLE
 * We use single buffer assuming command-response protocol,
 * that the next command will be sent after receiving the
 * response for the current command.
 */
uint8_t cmd_buf[256];
uint32_t cmd_len;
float heading, val;
float ang_des;

uint16_t samples[CLAP_FRAMELEN];     // to collect the ADC samples

volatile extern int clap_flag;

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

/* Debug counters */
int b0d, b0p;

/* button-press routine is called by the GPIO interrupt. We may
 * get multiple interrupts in the beginning or end of a button-press.
 * Hence, we start a timer and check if the button was still pressed
 * after the delay to debounce.
 */

/* Function prototype for printing to serial from the given task*/
void print_task(char* task_name, char* val){
    printf("[%s] ",task_name);
    printf(val);
    printf("\n");
}
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

void KILLSWITCH(void *arg){
    print_task("KILLSWITCH", "killing process begin ...");
    while (1){
        osEventFlagsWait(sid, FLAGS_MSK3, osFlagsWaitAny, osWaitForever);
        // osSemaphoreAcquire(sid_killswitch ,1U);
        // osMutexAcquire(mid_killswitch, osWaitForever);
        // uint32_t val = osSemaphoreGetCount (sid_killswitch);
        printf("[KILLSWITCH] AFTER... = %d\n", val);
        for (int i = 0; i < LED_NUM_ROWS; i++){
            led_on(i,i);
        }
        // stop the motors
        stop();
        // osMutexRelease(mid_killswitch);
        // osSemaphoreRelease(sid_killswitch);
        osTimerStop(timer_clap);
        osThreadSuspend(tid2);
        osThreadSuspend(ble_task);
        
    }
}

void task_cmd(void *arg){   
    print_task("task_cmd", "I am here");
    while(1){
        if(osEventFlagsGet(evt_clap)){
            print_task("task_cmd_clap", "stop");
            stop();
            print_task("task_cmd_clap", "stop done!!!");
        }
        else if(osEventFlagsGet(evt_stop)){
            print_task("task_cmd", "stop");
            stop();
            print_task("task_cmd", "stop done!!!");
        }
        else if(osEventFlagsGet(evt_frwd)){
            print_task("task_cmd","forward");
            forward(1);
            print_task("task_cmd", "forward done!");
        }
        else if(osEventFlagsGet(evt_bwd)){
            print_task("task_cmd", "reverse");
            reverse(1);
            print_task("task_cmd", "reverse done!!!");
        }
        else if(osEventFlagsGet(evt_left)){
            print_task("task_cmd", "Left");
            turn_left(1);
            print_task("task_cmd", "left done!!");
        }
        else if(osEventFlagsGet(evt_ryt)){
            print_task("task_cmd", "right");
            turn_right(1);
            print_task("task_cmd", "right done!!!");
        }
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

        osEventFlagsClear(evt_frwd, FLAGS_MSK1); osEventFlagsClear(evt_bwd, FLAGS_MSK1);
        osEventFlagsClear(evt_left, FLAGS_MSK1); osEventFlagsClear(evt_ryt, FLAGS_MSK1); osEventFlagsClear(evt_stop, FLAGS_MSK1);
        osEventFlagsClear(evt_clap, FLAGS_MSK2);
        
        /* Echo on UART */
        print_task("bluetooth", (char* ) cmd_buf);

        /* Echo on BLE */
        ble_send((uint8_t *) cmd_buf, strlen((char *) cmd_buf));

        /* Buggy Control */
        if (strlen((char *) cmd_buf) == 1)
        {
            switch (cmd_buf[0])
            {
            case 'u':
                print_task("bluetooth", "Move forward");
                osEventFlagsSet(evt_frwd, FLAGS_MSK1);
                break;
            case 'd':
                print_task("bluetooth", "Move reverse");
                osEventFlagsSet(evt_bwd, FLAGS_MSK1);
                break;
            case 'l':
                print_task("bluetooth", "Turn left");
                ang_des = 90;
                osEventFlagsSet(evt_left, FLAGS_MSK1);
                break;
            case 'r':
                print_task("bluetooth", "Turn right");
                ang_des = 270.0; // Range: [0,360)
                osEventFlagsSet(evt_ryt, FLAGS_MSK1);
                break;
            case 's':
                print_task("bluetooth", "STOP!!");
                osEventFlagsSet(evt_stop, FLAGS_MSK1);
                break;
            default:
                print_task("bluetooth", "STOP!!");
                osEventFlagsSet(evt_stop, FLAGS_MSK1);
                break;
            }
        } 
    }
}

void timer_callback_clap(void *arg)
{
    printf("reading ADC Sample\n");
    adc_read(samples,CLAP_FRAMELEN);
    if(clap_flag==1)
    {
        printf("clap is detected");
        clap_flag = 0;
        // osEventFlagsSet(evt_clap, FLAGS_MSK2);
    }
}

/* Mutex Definition*/
 
const osMutexAttr_t kill_switch_thread_mu = {
  "killswitch_mutex",     // human readable mutex name
  osMutexPrioInherit,    // attr_bits
  NULL,                // memory for control block   
  0U                   // size for control block
};

void task_ctrl(void *arg)
{
    // osTimerId_t timer_clap;

    // kill switch
    tid1 = osThreadNew(KILLSWITCH, NULL, NULL);
    osThreadSetPriority(tid1, osPriorityNormal);
    // sid_killswitch = osSemaphoreNew(1U, 0U, NULL);
    // mid_killswitch = osMutexNew(&kill_switch_thread_mu);
    
    ble_task = osThreadNew(bluetooth, NULL, NULL);
    osThreadSetPriority(ble_task, osPriorityNormal);

    tid2 = osThreadNew(task_cmd, NULL, NULL);
    osThreadSetPriority(tid2, osPriorityNormal);

    timer_clap = osTimerNew (timer_callback_clap, osTimerPeriodic, NULL, NULL);
    osTimerStart (timer_clap, 100);

    evt_frwd = osEventFlagsNew(NULL); evt_bwd = osEventFlagsNew(NULL);
    evt_left = osEventFlagsNew(NULL); evt_ryt = osEventFlagsNew(NULL); evt_stop = osEventFlagsNew(NULL);
    evt_clap = osEventFlagsNew(NULL);
    sid = osEventFlagsNew(NULL);

}

int main(void)
{
    osThreadId_t tid_ctrl;
    /* BSP initializations before BLE because we are using printf from BSP */
    board_init();
    servo_init();
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