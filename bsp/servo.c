#include <string.h>
#include "servo.h"
#include "lib.h"
// #define concat(a, b) a##b
/*
 * We will use PWM to provide a square wave to the speaker.
 * We can play various tones by setting the frequency of PWM.
 */

#ifndef IOREG32
#define IOREG32(addr) (*(volatile unsigned long *) (addr))
#endif

/*
 * NRF52 has a PWM that has more functionality than what we need.
 * Hence, we are using a subset of the PWM registers here.
 */

/* Register offsets */
#define STOP_PWM_A       		  IOREG32(0x40021004)     // stop pwm
#define SEQ0START_PWM_A  		  IOREG32(0x40021008)     // start sequence 0
#define EVENTS_STOPPED_PWM_A     IOREG32(0x40021104)     // event to indicate PWM stopped
#define EVENTS_SEQ0END_PWM_A     IOREG32(0x40021110)     // event to indicate sequnce 0 ended
#define EVENTS_PWMEND_PWM_A     IOREG32(0x40021118)     // event to indicate pwm period ended
#define ENABLE_PWM_A             IOREG32(0x40021500)     // enable / disable pwm
#define MODE_PWM_A               IOREG32(0x40021504)     // countertop mode up/upDown
#define COUNTERTOP_PWM_A         IOREG32(0x40021508)     //   countertop = pwm clock / pwm freq
#define PRESCALER_PWM_A          IOREG32(0x4002150C)     // divisor for PWM clock (2 power n)
#define DECODER_PWM_A          IOREG32(0x40021510)
#define SEQ0PTR_PWM_A            IOREG32(0x40021520)     // address where the sequence is stored
#define SEQ0CNT_PWM_A            IOREG32(0x40021524)     // number of values in the sequence
#define SEQ0REFRESH_PWM_A        IOREG32(0x40021528)     // additional pwm cycles between two values of the sequence
#define SEQ0ENDDELAY_PWM_A       IOREG32(0x4002152C)     // additional pwm cycles at the end
#define DECODER_PWM_A            IOREG32(0x40021510)     
// to set indivudial duty cycles of all 4 PWM channels we will use common configuration to set all 4 PWM to same duty cycle                                                   
#define PSEL_PWM_A               IOREG32(0x40021560)     // pin select

#define PSEL_PWM_B               IOREG32(0x40022560)     // pin select
#define STOP_PWM_B       		 IOREG32(0x40022004)     // stop pwm
#define SEQ0START_PWM_B  		 IOREG32(0x40022008)     // start sequence 0
#define EVENTS_STOPPED_PWM_B     IOREG32(0x40022104)     // event to indicate PWM stopped
#define EVENTS_SEQ0END_PWM_B     IOREG32(0x40022110)     // event to indicate sequnce 0 ended
#define EVENTS_PWMEND_PWM_B      IOREG32(0x40022118)     // event to indicate pwm period ended
#define ENABLE_PWM_B             IOREG32(0x40022500)     // enable / disable pwm
#define MODE_PWM_B               IOREG32(0x40022504)     // countertop mode up/upDown
#define COUNTERTOP_PWM_B         IOREG32(0x40022508)     //   countertop = pwm clock / pwm freq
#define PRESCALER_PWM_B          IOREG32(0x4002250C)     // divisor for PWM clock (2 power n)
#define DECODER_PWM_B            IOREG32(0x40022510)
#define SEQ0PTR_PWM_B            IOREG32(0x40022520)     // address where the sequence is stored
#define SEQ0CNT_PWM_B            IOREG32(0x40022524)     // number of values in the sequence
#define SEQ0REFRESH_PWM_B        IOREG32(0x40022528)     
#define SEQ0ENDDELAY_PWM_B       IOREG32(0x4002252C)     // additional pwm cycles at the end
#define DECODER_PWM_B            IOREG32(0x40022510)     

#define   PRESCALER_DIV16   4                       // 1MHz clock
#define PWM_CLK 1000000

// static uint16_t s_sequenceA[2], s_sequenceB[2];
static uint16_t s_sequence_A[1], s_sequence_B[1];

/* APIs */
void servo_init(void){
    PRESCALER_PWM_A = PRESCALER_DIV16;  // .5MHz clock
    ENABLE_PWM_A = 1;
    MODE_PWM_A = 0;     // only UP mode
    // DECODER_PWM_A = 2;    // Individual Decoder Mode

    PRESCALER_PWM_B = PRESCALER_DIV16;
    ENABLE_PWM_B = 1;
    MODE_PWM_B = 0;     // only UP mode
    // DECODER_PWM_B = 2;    // Individual Decoder Mode

    PSEL_PWM_A = A_PIN;   //Connect the port to the pin
    PSEL_PWM_B = B_PIN;    
}
/*
traj=1 => straight; dirn=1 => frwd; dirn=-1 => bwd;
traj=0 => circle; dirn=1 => CCW; dirn=-1 => CW;
dirn=0 => stop
speed levels limited to 6
*/ 
void servo_out(int traj, int dirn, int speed ){
    // printf("Start\n");
    int freq = 250; // Also look at base, diff, speed levels
    int base = 100, diff= 300; 
    int seqA, seqB;
   
    uint16_t countertop;
    countertop = PWM_CLK / freq;
    int mid_counter = countertop/2;
    COUNTERTOP_PWM_A = countertop;
    COUNTERTOP_PWM_B = countertop;

    SEQ0PTR_PWM_A = (uint32_t) s_sequence_A;
    SEQ0CNT_PWM_A = 1;
    SEQ0REFRESH_PWM_A = 0;

    SEQ0PTR_PWM_B = (uint32_t) s_sequence_B;
    SEQ0CNT_PWM_B = 1;
    SEQ0REFRESH_PWM_B = 0;

    if(dirn == 0){
        seqB = mid_counter; seqA = mid_counter; //Stop
        // seqB = countertop; seqA = countertop; //Stop
        printf("[SERVO] STOP here I am ");
    }
    else{
        if(traj == 1){
            if(dirn==1){    // Frwd
                seqB = base + speed*diff;
                seqA = seqB + mid_counter; 
                printf("[SERVO] forward");
            }
            else{   //bwd
                seqB = base + speed*diff +mid_counter;
                seqA = seqB -mid_counter; 
                printf("[SERVO] reverse");
            }
        }
        else if(traj==0){
            if(dirn==1){    // CCW
                seqB = base + speed*diff ;
                seqA = seqB; 
                printf("[SERVO] left");
            }
            else{   //CW
                seqB = base + speed*diff +mid_counter;
                seqA = seqB; 
                printf("[SERVO] right");
            }
        }
        else{
            seqB = mid_counter; seqA = mid_counter;   //Stop
            printf("[SERVO] stop");
        }
    }
    
    /* Set count values for specified duty cycle 
    ForA >2000 Vehicle Frwd */
    s_sequence_A[0] = seqA;
    s_sequence_B[0] = seqB;
    // s_sequence_A[1] = (countertop * dutyA) ;
    // s_sequence_B[1] = (countertop * dutyB) ;
    
    /* Start sequence 0 */
    SEQ0START_PWM_A = 1;
    SEQ0START_PWM_B = 1;

    delay_ms(10);
    /* Wait for the sequence to complete */
    while (EVENTS_SEQ0END_PWM_A == 0 || EVENTS_SEQ0END_PWM_B == 0){
        // printf("[servo_out] inside seqEnd");
        ;
    }

    EVENTS_SEQ0END_PWM_A = 0;
    EVENTS_SEQ0END_PWM_B = 0;

    STOP_PWM_A = 1;
    STOP_PWM_B = 1;
    while (EVENTS_STOPPED_PWM_A == 0 || EVENTS_STOPPED_PWM_B == 0){
        // printf("[servo_out] inside Stop");   
        ;     
    }
    EVENTS_STOPPED_PWM_A = 0;
    EVENTS_STOPPED_PWM_B = 0;

    // printf("End\n");
}
