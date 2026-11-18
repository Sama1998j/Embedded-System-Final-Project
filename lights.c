#include <xc.h>
#include "lights.h"

void lights_init(void)
{
    TRISBbits.TRISB8 = 0;   // RB8 left side lights
    TRISFbits.TRISF1 = 0;   // RF1 right side lights
    TRISGbits.TRISG1 = 0;   // RG1 low-intensity lights
    TRISFbits.TRISF0 = 0;   // RF0 brake lights

    LATBbits.LATB8 = 0;
    LATFbits.LATF1 = 0;
    LATGbits.LATG1 = 0;
    LATFbits.LATF0 = 0;
}

void lights_left(int on)
{
    LATBbits.LATB8 = on;
}

void lights_right(int on)
{
    LATFbits.LATF1 = on;
}

void lights_low(int on)
{
    LATGbits.LATG1 = on;
}
// RF0 controls the brake lights
void lights_brake(int on)
{
    LATFbits.LATF0 = on;
}
