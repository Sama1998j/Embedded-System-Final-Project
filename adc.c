#include <xc.h>
#include "adc.h"

#define BATTERY_CHANNEL 11      // AN11 (RB11)
#define IR_CHANNEL      14      // AN14 (RB14)
#define VREF            3.3
#define ADC_MAX        4095.0   // 12-bit ADC
#define BATTERY_RATIO   3.0     

// IR sensor distance limits
#define DIST_MIN_CM     ADC_DIST_MIN_CM
#define DIST_MAX_CM     ADC_DIST_MAX_CM

void adc_init(void)
{
    // AN11 / RB11 (battery) and AN14 / RB14 (IR) as analog inputs
    ANSELBbits.ANSB11 = 1;
    TRISBbits.TRISB11 = 1;
    ANSELBbits.ANSB14 = 1;
    TRISBbits.TRISB14 = 1;

    // RB9 as digital output to enable the IR sensor
    ANSELBbits.ANSB9 = 0;
    TRISBbits.TRISB9 = 0;
    LATBbits.LATB9 = 1;


    AD1CON1bits.ADON = 0;
    AD1CON1bits.AD12B = 1;   // 12-bit
    AD1CON1bits.FORM = 0;    // integer output
    AD1CON1bits.SSRC = 7;    // auto-convert after the sample time set by SAMC
    AD1CON1bits.ASAM = 1;    // auto-sample

    AD1CON2bits.VCFG = 0;    // AVdd / AVss reference
    AD1CON2bits.CHPS = 0;    // CH0 only

    // Enable scan mode for AN11 and AN14 
    AD1CON2bits.CSCNA = 1;   // scan inputs listed in AD1CSSL
    AD1CON2bits.SMPI = 1;    // two scanned channels

    AD1CON3bits.ADRC = 0;    // system clock
    AD1CON3bits.ADCS = 10;   // Tad
    AD1CON3bits.SAMC = 31;   // maximum sample time

    // Select AN11 and AN14 for scanning
    AD1CSSL = 0;
    AD1CSSLbits.CSS11 = 1;   // AN11 battery
    AD1CSSLbits.CSS14 = 1;   // AN14 IR

    AD1CHS0bits.CH0NA = 0;   // negative input is VR- (GND)

    AD1CON1bits.ADON = 1;
}

// Read the latest ADC results
static void adc_scan(unsigned int *battery_raw, unsigned int *ir_raw)
{
    *battery_raw = ADC1BUF0;    // AN11
    *ir_raw      = ADC1BUF1;    // AN14
}

// Latest ADC results
static unsigned int battery_raw_cached = 0;
static unsigned int ir_raw_cached       = 0;

// Update cached ADC values
void adc_update(void)
{
    adc_scan(&battery_raw_cached, &ir_raw_cached);
}

double adc_read_battery(void)
{
    double pin_voltage = ((double)battery_raw_cached / ADC_MAX) * VREF;
    return pin_voltage * BATTERY_RATIO;
}

int adc_read_distance(void)
{
    double v = ((double)ir_raw_cached / ADC_MAX) * VREF;
    double d_m = 2.34 - 4.74 * v + 4.06 * v * v - 1.60 * v * v * v + 0.24 * v * v * v * v;
    int cm = (int)(d_m * 100.0);

    if (cm < DIST_MIN_CM) cm = DIST_MIN_CM;
    if (cm > DIST_MAX_CM) cm = DIST_MAX_CM;

    return cm;
}