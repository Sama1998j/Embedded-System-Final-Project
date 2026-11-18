#include <xc.h>
#include "motor.h"

// 72 MHz / 10 kHz = 7200 counts per PWM period
#define PWM_PERIOD 7200

// Runtime-configurable saturation limit, set via $PCCFG,MAXSPEED,x*
static int max_speed_percent = 100;

static void oc_config(void)
{
    // OC1 = RD1 (left backward)
    OC1CON1 = 0;
    OC1CON2 = 0;
    OC1CON1bits.OCTSEL = 0b111;   // peripheral clock as source (TPWM = Tcy)
    OC1CON2bits.SYNCSEL = 0x1F;   // self-sync. timer resets at OCxRS
    OC1R = 0;                     // duty
    OC1RS = PWM_PERIOD - 1;       // period = 10 kHz
    OC1CON1bits.OCM = 0b110;      // edge aligned PWM

    // OC2 = RD2 (left forward)
    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2R = 0;
    OC2RS = PWM_PERIOD - 1;
    OC2CON1bits.OCM = 0b110;

    // OC3 = RD3 (right backward)
    OC3CON1 = 0;
    OC3CON2 = 0;
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3R = 0;
    OC3RS = PWM_PERIOD - 1;
    OC3CON1bits.OCM = 0b110;

    // OC4 = RD4 (right forward)
    OC4CON1 = 0;
    OC4CON2 = 0;
    OC4CON1bits.OCTSEL = 0b111;
    OC4CON2bits.SYNCSEL = 0x1F;
    OC4R = 0;
    OC4RS = PWM_PERIOD - 1;
    OC4CON1bits.OCM = 0b110;
}

void motor_set_max_speed(int percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    max_speed_percent = percent;
}

void motor_init(void)
{
    // RD1-RD4 as digital outputs.
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD4 = 0;

    // Map the four Output Compare outputs onto RP65-RP68 (RD1-RD4).
    RPOR0bits.RP65R = 0b010000;   // OC1 = RD1
    RPOR1bits.RP66R = 0b010001;   // OC2 = RD2
    RPOR1bits.RP67R = 0b010010;   // OC3 = RD3
    RPOR2bits.RP68R = 0b010011;   // OC4 = RD4

    oc_config();
}

// Convert PWM percentage to timer counts
static unsigned int pwm_count(int percent)
{
    // Use long because 100 * 7200 exceeds a 16-bit int
    long count = (long)percent * PWM_PERIOD;
    count = count / 100;

    return (unsigned int)count;
}

void motor_set(int left_pwm, int right_pwm)
{
    // saturate to +/- the configured max speed
    if (left_pwm > max_speed_percent)   left_pwm = max_speed_percent;
    if (left_pwm < -max_speed_percent)  left_pwm = -max_speed_percent;
    if (right_pwm > max_speed_percent)  right_pwm = max_speed_percent;
    if (right_pwm < -max_speed_percent) right_pwm = -max_speed_percent;
    
    /* Actuation per the specification table:
     *   left  > 0 -> RD2, left  < 0 -> RD1
     *   right > 0 -> RD4, right < 0 -> RD3
     */
    if (left_pwm >= 0)
    {
        OC2R = pwm_count(left_pwm);    // RD2 = left forward
        OC1R = 0;
    }
    else
    {
        OC1R = pwm_count(-left_pwm);   // RD1 = left backward
        OC2R = 0;
    }

    if (right_pwm >= 0)
    {
        OC4R = pwm_count(right_pwm);   // RD4 = right forward
        OC3R = 0;
    }
    else
    {
        OC3R = pwm_count(-right_pwm);  // RD3 = right backward
        OC4R = 0;
    }
}
