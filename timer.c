#include <xc.h>
#include "timer.h"

// Timer ticks per millisecond with Fcy = 72 MHz and 1:256 prescaler
#define TICKS_PER_MS 281

void tmr_setup_period(int timer, int ms)
{
    if (timer == TIMER1)
    {
        // Stop Timer1 before configuration
        T1CONbits.TON = 0;

        // Reset Timer1 counter
        TMR1 = 0;

        // Clear Timer1 interrupt flag
        IFS0bits.T1IF = 0;

        // Set Timer1 period register
        PR1 = TICKS_PER_MS * ms;
        
        T1CONbits.TCKPS = 3;

        // Use internal clock
        T1CONbits.TCS = 0;

        // Disable gated timer mode
        T1CONbits.TGATE = 0;

        // Start Timer1
        T1CONbits.TON = 1;
    }

    if (timer == TIMER2)
    {
        T2CONbits.TON = 0;
        TMR2 = 0;
        IFS0bits.T2IF = 0;
        PR2 = TICKS_PER_MS * ms;
        T2CONbits.TCKPS = 3;
        T2CONbits.TCS = 0;
        T2CONbits.TGATE = 0;
        T2CONbits.TON = 1;
    }
}

// Wait for the timer period; return 1 if the deadline was missed
int tmr_wait_period(int timer)
{
    int ret = 0;

    if (timer == TIMER1)
    {
        if (IFS0bits.T1IF == 1)
        {
            IFS0bits.T1IF = 0;
            return 1;           // deadline already missed, do not wait again
        }

        while (IFS0bits.T1IF == 0);
        IFS0bits.T1IF = 0;
    }

    if (timer == TIMER2)
    {
        if (IFS0bits.T2IF == 1)
        {
            IFS0bits.T2IF = 0;
            return 1;
        }

        while (IFS0bits.T2IF == 0);
        IFS0bits.T2IF = 0;
    }

    return ret;
}

void tmr_wait_ms(int timer, int ms)
{
    if (timer == TIMER1)
    {
        tmr_setup_period(TIMER1, ms);
        while (IFS0bits.T1IF == 0);
        IFS0bits.T1IF = 0;
        T1CONbits.TON = 0;
    }

    if (timer == TIMER2)
    {
        tmr_setup_period(TIMER2, ms);
        while (IFS0bits.T2IF == 0);
        IFS0bits.T2IF = 0;
        T2CONbits.TON = 0;
    }
}
