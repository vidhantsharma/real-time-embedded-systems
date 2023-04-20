#include <stdint.h>
#include <string.h>
#include "servo.h"

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
#define STOP_PWM_A       		 IOREG32(0x40021004)     // stop pwm
#define SEQ0START_PWM_A  		 IOREG32(0x40021008)     // start sequence 0
#define EVENTS_STOPPED_PWM_A     IOREG32(0x40021104)     // event to indicate PWM stopped
#define EVENTS_SEQ0END_PWM_A     IOREG32(0x40021110)     // event to indicate sequnce 0 ended
#define EVENTS_PWMEND_PWM_A      IOREG32(0x40021118)     // event to indicate pwm period ended
#define ENABLE_PWM_A             IOREG32(0x40021500)     // enable / disable pwm
#define MODE_PWM_A               IOREG32(0x40021504)     // countertop mode up/upDown
#define COUNTERTOP_PWM_A         IOREG32(0x40021508)     //   countertop = pwm clock / pwm freq
#define PRESCALER_PWM_A          IOREG32(0x4002150C)     // divisor for PWM clock (2 power n)
#define SEQ0PTR_PWM_A            IOREG32(0x40021520)     // address where the sequence is stored
#define SEQ0CNT_PWM_A            IOREG32(0x40021524)     // number of values in the sequence
#define SEQ0REFRESH_PWM_A        IOREG32(0x40021528)     // additional pwm cycles between two
                                                    //   values of the sequence
#define SEQ0ENDDELAY_PWM_A       IOREG32(0x4002152C)     // additional pwm cycles at the end
#define PSEL_PWM_A               IOREG32(0x40021560)     // pin select
#define DECODER_PWM_A            IOREG32(0x40021510)     // to set indivudial duty cycles of
                                                    //   all 4 PWM channels
                                                    // we will use common configuration
                                                    // to set all 4 PWM to same duty cycle


#define STOP_PWM_B       		 IOREG32(0x40022004)     // stop pwm
#define SEQ0START_PWM_B  		 IOREG32(0x40022008)     // start sequence 0
#define EVENTS_STOPPED_PWM_B     IOREG32(0x40022104)     // event to indicate PWM stopped
#define EVENTS_SEQ0END_PWM_B     IOREG32(0x40022110)     // event to indicate sequnce 0 ended
#define EVENTS_PWMEND_PWM_B      IOREG32(0x40022118)     // event to indicate pwm period ended
#define ENABLE_PWM_B             IOREG32(0x40022500)     // enable / disable pwm
#define MODE_PWM_B               IOREG32(0x40022504)
#define COUNTERTOP_PWM_B         IOREG32(0x40022508)     //   countertop = pwm clock / pwm freq
#define PRESCALER_PWM_B          IOREG32(0x4002250C)     // divisor for PWM clock (2 power n)

#define SEQ0PTR_PWM_B           IOREG32(0x40022520)     // address where the sequence is stored
#define SEQ0CNT_PWM_B           IOREG32(0x40022524)     // number of values in the sequence
#define SEQ0REFRESH_PWM_B       IOREG32(0x40022528)     // additional pwm cycles between two
                                                    //   values of the sequence
#define SEQ0ENDDELAY_PWM_B      IOREG32(0x4002252C)     // additional pwm cycles at the end
#define PSEL_PWM_B              IOREG32(0x40022560)     // pin select
#define DECODER_PWM_B           IOREG32(0x40022510)

#define   PRESCALER_DIV32   5                      // 0.5MHz clock
#define PWM_CLK 500000

#define VELOCITY_STEP 25
static uint16_t s_sequenceA[2], s_sequenceB[2];

/* APIs */
void servo_init(void){
    PRESCALER_PWM_A = PRESCALER_DIV32;  // 1MHz clock
    PSEL_PWM_A = A_PIN;   //Connect the port to the pin 
    ENABLE_PWM_A = 1;
    MODE_PWM_A = 0;     // only UP mode

    PRESCALER_PWM_B = PRESCALER_DIV32;
    PSEL_PWM_B = B_PIN;   // /* Connect the port to the pin */
    ENABLE_PWM_B = 1;
    MODE_PWM_B = 0;     // only UP mode
}

/* Generate a PWM wave of the specified duty cycle for the specified duration
 *  duty cycle in integer percentage, for example, 50.
 */
void servo_out(int dirA, int vA, int dirB, int vB)
{
    int freqA, freqB;
    int dutyA, dutyB;
    int duration_ms = 5000;

    freqA = 250;
    freqB = 250;

    dutyA = 50 - dirA*vA*VELOCITY_STEP;
    dutyB = 50 - dirB*vB*VELOCITY_STEP;

    int n_pwm_cyclesA = (freqA * duration_ms) / 1000;
    int n_pwm_cyclesB = (freqB * duration_ms) / 1000;
    uint16_t countertopA, countertopB;

    /* Set PWM counter for the required frequency. */
    countertopA = PWM_CLK / freqA;
    COUNTERTOP_PWM_A = countertopA;

    countertopB =  PWM_CLK / freqB;
    COUNTERTOP_PWM_B = countertopB;

    /* To generate n PWM cycles, we create a sequence of two identical values
     * with (n-2) cycles gap in-between.
     */
    SEQ0PTR_PWM_A = (uint32_t) s_sequenceA;
    SEQ0CNT_PWM_A = 2;
    SEQ0REFRESH_PWM_A = n_pwm_cyclesA - 2;
    SEQ0PTR_PWM_B = (uint32_t) s_sequenceB;
    SEQ0CNT_PWM_B = 2;
    SEQ0REFRESH_PWM_B = n_pwm_cyclesB - 2;

    /* Set count values for specified duty cycle */
    s_sequenceA[0] = (countertopA * dutyA) / 100;
    s_sequenceA[1] = (countertopA * dutyA) / 100;
    s_sequenceB[0] = (countertopB * dutyB) / 100;
    s_sequenceB[1] = (countertopB * dutyB) / 100;

    /* Start sequence 0 */
    SEQ0START_PWM_B = 1;

    /* Start sequence 0 */
    SEQ0START_PWM_A = 1;

    // /* Wait for the sequence to complete */
    // while (EVENTS_SEQ0END_PWM_A == 0 || EVENTS_SEQ0END_PWM_B == 0)
    //     ;
    // EVENTS_SEQ0END_PWM_A = 0;
    // EVENTS_SEQ0END_PWM_B = 0;

    // /* Stop PWM */
    // STOP_PWM_A = 1;
    // STOP_PWM_B = 1;
    // // while (EVENTS_STOPPED_PWM_A == 0)
    // while (EVENTS_STOPPED_PWM_A == 0 && EVENTS_SEQ0END_PWM_B == 0)
    //     ;
    // EVENTS_STOPPED_PWM_A = 0;
    // EVENTS_STOPPED_PWM_B = 0;

}