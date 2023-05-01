#include <stdint.h>
#include <string.h>

#include "cmsis_os2.h"
#include "ble_uart.h"
#include "board.h"
#include "lib.h"
#include "estimator.h"

// #define FLAGS_MSK1 0x00000001U
#define FLAGS_MSK2 0x00000002U     // EVENT CLAP
#define FLAGS_MSK3 0x00000003U     // EVENT COLLISION
#define FLAGS_MSK4 0x00000004U     // EVENT COMMAND
#define FLAGS_MSK_f 0x00000005U    // 5, FORWARD
#define FLAGS_MSK_b 0x00000006U    // 6, REVERSE
#define FLAGS_MSK_l 0x0000005AU    // 90deg LEFT
#define FLAGS_MSK_r 0x0000010EU    // 270deg RIGHT
#define MAX_COUNT 50               // MAXIMUM COUNTER

/* OS objects */
osEventFlagsId_t evt_linear, evt_angular, evt_stop, evt_cmd;
extern osEventFlagsId_t evt_clap;
extern osEventFlagsId_t sid;
osThreadId_t killer_switch_id, collision_id, clap_id, command_id, bluetooth_id;


/* Buffer to hold the command received from UART or BLE
 * We use single buffer assuming command-response protocol,
 * that the next command will be sent after receiving the
 * response for the current command.
 */
uint8_t cmd_buf[256];
uint32_t cmd_len;

// controller specifics
float ang_des;
int headDes_old, headDes = INT32_MIN;

uint16_t samples[CLAP_FRAMELEN];     // to collect the ADC samples

volatile extern int clap_flag;

// LED 
uint8_t frame_buffer[LED_NUM_ROWS][LED_NUM_COLS] =
{
    { 0, 1, 0, 1, 0},
    { 0, 0, 1, 0, 0},
    { 1, 0, 0, 0, 1},
    { 0, 1, 1, 1, 0},
    { 0, 0, 0, 0, 0}
};

static int count;

uint32_t r, c;
uint32_t r0, c0;

/* Debug counters */
int b0d, b0p;

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
    osThreadFlagsSet(bluetooth_id, 1); 
}

void KILLSWITCH(void *arg){
    print_task("KILLSWITCH", "killing process begin ...");
    while (1){
        osEventFlagsWait(sid, FLAGS_MSK3, osFlagsWaitAny, osWaitForever);
        printf("[KILLSWITCH] Safety Mode = %d\n");
        for (int i = 0; i < LED_NUM_ROWS; i++){
            led_on(i,i);
        }
        // stop the motors
        servo_out(1,0,3);
        osThreadSuspend(collision_id);
        osThreadSuspend(clap_id);
        osThreadSuspend(command_id);
        osThreadSuspend(bluetooth_id);

    }
}

void task_cmd(void *arg){   
    delay_ms(10);
    float headCur;
    uint32_t lin_flag, ang_flag, stop_flag; 
    print_task("TASK CMD","Entering task_cmd");
    // with pins facing forward(180deg), left-ang increases, 
    while(1){
        headCur = computeHeading(); 
        // osMessageQueueGet(ang_dataQ, &headCur, NULL, 0U); 
        lin_flag = osEventFlagsGet(evt_linear);
        ang_flag = osEventFlagsGet(evt_angular);
        stop_flag = osEventFlagsGet(evt_stop);
        uint32_t clap_evt_id = osEventFlagsGet(evt_clap);
        // printf("\n[task_cmd] flags are: %d, %d, %d", lin_flag, ang_flag, stop_flag);
        if (clap_evt_id){
            printf("[TASK COMMAND] Clap detected stopping the robot\n");
            servo_out(1,0,3);
        }
        else if (stop_flag){
            printf("[TASK COMMAND] Stop Event\n");
            servo_out(1,0,3);
        }
        else if(lin_flag){
            if(headDes == INT32_MIN){
                headDes = headCur;
                printf("Initial Orientation Set[task_cmd if]\n");
            }
            if(lin_flag == 5){
                printf("[task_cmd] Frwd flag, %d\n", lin_flag);
                // delay_ms(10);
                move_ctrlr(1, headDes, headCur);
            }
            else{
                printf("[task_cmd] Bwd flag, %d\n", lin_flag);
                // delay_ms(10);
                move_ctrlr(-1, headDes,headCur);
            }         
        }
        else if(ang_flag){
            if(ang_flag == 90){
                if(headDes == INT32_MIN){
                    headDes = headDes_old;
                    headDes += 90;
                    headDes = headDes%360; 
                }
                // delay_ms(10);
                print_task("TASK COMMAND", "turn");
                turn_ctrlr(1,headDes,headCur);
                // printf("left flag done");
                
            }
            else{
                if(headDes == INT32_MIN){
                    headDes = headDes_old;
                    headDes += 270;
                    headDes = headDes%360; 
                }
                // delay_ms(10);
                print_task("TASK COMMAND", "turn");
                turn_ctrlr(-1,headDes,headCur);
                // printf("ryt flag done");
            }
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

        osEventFlagsClear(evt_linear, FLAGS_MSK_f); osEventFlagsClear(evt_linear, FLAGS_MSK_b);
        osEventFlagsClear(evt_angular, FLAGS_MSK_l); osEventFlagsClear(evt_angular, FLAGS_MSK_r); 
        osEventFlagsClear(evt_stop, FLAGS_MSK_f);
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
                print_task("bluetooth","forward FlagSet\n");
                // delay_ms(50);
                osEventFlagsSet(evt_linear, FLAGS_MSK_f);
                headDes = INT32_MIN;
                break;
            case 'd':
                print_task("bluetooth","reverse FlagSet\n");
                osEventFlagsSet(evt_linear, FLAGS_MSK_b);
                headDes = INT32_MIN;
                break;
            case 'l':
                print_task("bluetooth ","left FlagSet\n");
                osEventFlagsSet(evt_angular, FLAGS_MSK_l); // make mask headCurue of desired angle
                headDes_old = headDes;
                headDes = INT32_MIN;
                break;
            case 'r':
                print_task("bluetooth","right FlagSet\n");
                osEventFlagsSet(evt_angular, FLAGS_MSK_r);
                headDes_old = headDes;
                headDes = INT32_MIN;
                break;
            case 's':
                print_task("bluetooth","STOP!!\n");
                osEventFlagsSet(evt_stop, FLAGS_MSK_f);
                break;
            default:
                print_task("bluetooth", "STOP!!");
                osEventFlagsSet(evt_stop, FLAGS_MSK_f);
                break;
            }
        } 
    }
}

void clap_event(void *arg)
{
    printf("reading ADC Sample\n");
    adc_read(samples,CLAP_FRAMELEN);
    if(clap_flag==1)
    {
        printf("clap is detected");
        clap_flag = 0;
        // osEventFlagsSet(evt_clap, FLAGS_MSK2);
    }
    // uint32_t val = osEventFlagsGet(evt_cmd);
    // printf("[COMMAND] val = %d", val);
    count++;
    if (count == MAX_COUNT)
    {
        count = 0;
        osThreadYield();
    }
}
void collision_event(void *arg){
    print_task("COLLISION", "Thread started");
    while (1){
        print_task("CALLBACK", "check collision");
        float accData[3], val;
        accReadXYZ(accData);
        if (isCollision(accData, &val)){
            printf("[COLLISION] Collision Happened value = " );
            print_float(val,4);
            osEventFlagsSet(sid, FLAGS_MSK3);
        }
        count++;
        if (count == MAX_COUNT)
        {
            count = 0;
            osThreadYield();
        }
    }
}

void task_ctrl(void *arg)
{
    // osTimerId_t timer_clap;

    // kill switch
    killer_switch_id = osThreadNew(KILLSWITCH, NULL, NULL);
    osThreadSetPriority(killer_switch_id, osPriorityNormal);
    // sid_killswitch = osSemaphoreNew(1U, 0U, NULL);
    // mid_killswitch = osMutexNew(&kill_switch_thread_mu);
    
    bluetooth_id = osThreadNew(bluetooth, NULL, NULL);
    osThreadSetPriority(bluetooth_id, osPriorityNormal);

    command_id = osThreadNew(task_cmd, NULL, NULL);
    osThreadSetPriority(command_id, osPriorityNormal);

    clap_id = osThreadNew(clap_event, NULL, NULL);
    osThreadSetPriority(clap_id, osPriorityNormal);

    collision_id = osThreadNew(collision_event, NULL, NULL);
    osThreadSetPriority(collision_id, osPriorityNormal);
    // timer_clap = osTimerNew (timer_callback_clap, osTimerPeriodic, NULL, NULL);
    // osTimerStart (timer_clap, 100);

    evt_linear = osEventFlagsNew(NULL); evt_angular = osEventFlagsNew(NULL);
    evt_stop = osEventFlagsNew(NULL);
    evt_clap = osEventFlagsNew(NULL);
    sid = osEventFlagsNew(NULL);

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