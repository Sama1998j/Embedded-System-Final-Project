#include <stdio.h>
#include <math.h>
#include "xc.h"
#include "timer.h"
#include "uart.h"
#include "parser.h"
#include "adc.h"
#include "spi.h"
#include "imu.h"
#include "motor.h"
#include "lights.h"
#include "fsm.h"

// 500 Hz control loop: one tick every 2 ms
#define LOOP_PERIOD_MS 2

/* Magnetometer hard-iron offsets, obtained as (min+max)/2 per axis
 * over a full rotation. */
#define MAG_OFFSET_X 152
#define MAG_OFFSET_Y -238

// 10 ticks at 2 ms = 20 ms of settling before a button level is accepted
#define DEBOUNCE_TICKS 10

/* Moving-average window for the IMU sensors (~0.4 s at 10 Hz).
 * Power of two so the division reduces to a shift. */
#define IMU_FILTER_N 4

// Stagger UART reports within the 100 ms period
#define REPORT_PERIOD    50

// UART report offsets
#define PHASE_DIST        0    
#define PHASE_ANGLE      12   
#define PHASE_GYRO       25    
#define PHASE_BATT       37    

// Enable/disable flags for periodic UART reports, set via $PCLOG,...*
int log_batt_enable  = 1;
int log_dist_enable  = 1;
int log_angle_enable = 1;
int log_gyro_enable  = 1;

int main(void) {

    // All pins digital default
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA0 = 0;

    // Beam headlights (RA7): no specified behaviour, driven low.
    TRISAbits.TRISA7 = 0;
    LATAbits.LATA7 = 0;
    // Counter for periodic UART scheduling
    unsigned int tick = 0;

    int ld1_counter = 0;
    int dist_sum = 0;
    int dist_samples = 0;
    
    // sensor values sent to the state machine
    int current_distance = 150;
    double current_yaw = 0.0;

    // Moving-average buffers for IMU data
    int accel_hist_x[IMU_FILTER_N] = {0};
    int accel_hist_y[IMU_FILTER_N] = {0};
    int accel_hist_z[IMU_FILTER_N] = {0};
    long accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;

    int mag_hist_x[IMU_FILTER_N] = {0};
    int mag_hist_y[IMU_FILTER_N] = {0};
    int mag_hist_z[IMU_FILTER_N] = {0};
    long mag_sum_x = 0, mag_sum_y = 0, mag_sum_z = 0;

    int gyro_hist_x[IMU_FILTER_N] = {0};
    int gyro_hist_y[IMU_FILTER_N] = {0};
    int gyro_hist_z[IMU_FILTER_N] = {0};
    long gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    int imu_hist_i = 0;

    // Debounce state for RE8 and RE9
    int e8_stable = 1;
    int e8_raw_prev = 1;
    int e8_count = 0;

    int e9_stable = 1;
    int e9_raw_prev = 1;
    int e9_count = 0;

    char buffer[64];

    pc_reference reference[1] = {{0, 0}};

    // RE8 (state toggle) and RE9 ($MBUF) as inputs
    TRISEbits.TRISE8 = 1;
    TRISEbits.TRISE9 = 1;

    // INT1 on RE8 (RPI88), falling edge, for the immediate emergency stop.
    RPINR0bits.INT1R = 88;
    INTCON2bits.INT1EP = 1;   // 1 = negative/falling edge
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;

    uart1_init();
    adc_init();
    setup_spi();
    imu_init();
    motor_init();
    lights_init();
    fsm_init();

    // Start the 500 Hz timer loop
    tmr_setup_period(TIMER1, LOOP_PERIOD_MS);

    while (1)
    {
        parser_process(reference);

        // Update ADC values once per control-loop tick
        adc_update();
         
        // Debounce RE8 for the Halted/Moving toggle
        int e8_raw = PORTEbits.RE8;

        if (e8_raw == e8_raw_prev)
        {
            if (e8_count < DEBOUNCE_TICKS)
            {
                e8_count++;
            }
        }
        else
        {
            e8_count = 0;
        }
        e8_raw_prev = e8_raw;

        if (e8_count >= DEBOUNCE_TICKS && e8_raw != e8_stable)
        {
            e8_stable = e8_raw;

            if (e8_stable == 0)   // falling edge = press
            {
                fsm_request_toggle();
            }
        }
        // RE9 debounced press: reports TX/RX buffer occupancy via $MBUF.
        int e9_raw = PORTEbits.RE9;

        if (e9_raw == e9_raw_prev)
        {
            if (e9_count < DEBOUNCE_TICKS)
            {
                e9_count++;
            }
        }
        else
        {
            e9_count = 0;
        }
        e9_raw_prev = e9_raw;

        if (e9_count >= DEBOUNCE_TICKS && e9_raw != e9_stable)
        {
            e9_stable = e9_raw;

            if (e9_stable == 0)   // falling edge = press
            {
                sprintf(buffer, "$MBUF,%d,%d*", uart1_tx_count(), uart1_rx_count());
                uart1_send_string(buffer);
            }
        }


        // LED A0 blinks at 1 Hz to show the main loop is alive
        ld1_counter++;
        if (ld1_counter >= 250)
        {
            ld1_counter = 0;
            LATAbits.LATA0 = !LATAbits.LATA0;
        }

        int phase = tick % REPORT_PERIOD;

        // Read IR at 500 Hz; report distance at 10 Hz
        current_distance = adc_read_distance();
        dist_sum += current_distance;
        dist_samples++;

        // $MDIST at 10 Hz
        if (phase == PHASE_DIST)
        {
            int dist_avg = (dist_samples > 0) ? (dist_sum / dist_samples) : current_distance;
            dist_sum = 0;
            dist_samples = 0;
            if (log_dist_enable)
            {
                sprintf(buffer, "$MDIST,%d*", dist_avg);
                uart1_send_string(buffer);
            }
        }

        // Read and filter IMU data at PHASE_ANGLE
        static double gx_c = 0, gy_c = 0, gz_c = 0;

                if (phase == PHASE_ANGLE)
        {
            // Update moving-average filters
            AxesRaw_t accel = accel_read_xyz();
            AxesRaw_t mag   = mag_read_xyz();
            AxesRaw_t gyro  = gyro_read_xyz();

            accel_sum_x -= accel_hist_x[imu_hist_i];
            accel_sum_y -= accel_hist_y[imu_hist_i];
            accel_sum_z -= accel_hist_z[imu_hist_i];
            accel_hist_x[imu_hist_i] = accel.x;
            accel_hist_y[imu_hist_i] = accel.y;
            accel_hist_z[imu_hist_i] = accel.z;
            accel_sum_x += accel.x;
            accel_sum_y += accel.y;
            accel_sum_z += accel.z;

            mag_sum_x -= mag_hist_x[imu_hist_i];
            mag_sum_y -= mag_hist_y[imu_hist_i];
            mag_sum_z -= mag_hist_z[imu_hist_i];
            mag_hist_x[imu_hist_i] = mag.x;
            mag_hist_y[imu_hist_i] = mag.y;
            mag_hist_z[imu_hist_i] = mag.z;
            mag_sum_x += mag.x;
            mag_sum_y += mag.y;
            mag_sum_z += mag.z;

            gyro_sum_x -= gyro_hist_x[imu_hist_i];
            gyro_sum_y -= gyro_hist_y[imu_hist_i];
            gyro_sum_z -= gyro_hist_z[imu_hist_i];
            gyro_hist_x[imu_hist_i] = gyro.x;
            gyro_hist_y[imu_hist_i] = gyro.y;
            gyro_hist_z[imu_hist_i] = gyro.z;
            gyro_sum_x += gyro.x;
            gyro_sum_y += gyro.y;
            gyro_sum_z += gyro.z;

            imu_hist_i++;
            if (imu_hist_i >= IMU_FILTER_N)
            {
                imu_hist_i = 0;
            }

            // Filtered accelerometer, used for roll/pitch
            double ax = (double)accel_sum_x / IMU_FILTER_N;
            double ay = (double)accel_sum_y / IMU_FILTER_N;
            double az = (double)accel_sum_z / IMU_FILTER_N;

            // Filtered magnetometer, used for yaw
            double mx = (double)mag_sum_x / IMU_FILTER_N;
            double my = (double)mag_sum_y / IMU_FILTER_N;

            // Filtered gyro, cached for the $MGYRO report at PHASE_GYRO.
            // BMG160 default range: 16.4 LSB per deg/s
            gx_c = ((double)gyro_sum_x / IMU_FILTER_N) / 16.4;
            gy_c = ((double)gyro_sum_y / IMU_FILTER_N) / 16.4;
            gz_c = ((double)gyro_sum_z / IMU_FILTER_N) / 16.4;

            double roll  = atan2(-ay, az) * 180.0 / M_PI;
            double pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
            double yaw   = atan2(my - (double)MAG_OFFSET_Y,
                                 mx - (double)MAG_OFFSET_X) * 180.0 / M_PI;

            current_yaw = yaw;

            if (log_angle_enable)
            {
                sprintf(buffer, "$MANGLE,%.2f,%.2f,%.2f*", roll, pitch, yaw);
                uart1_send_string(buffer);
            }
        }

        // $MGYRO report at PHASE_GYRO, using values cached at PHASE_ANGLE.
        if (phase == PHASE_GYRO)
        {
            if (log_gyro_enable)
            {
                sprintf(buffer, "$MGYRO,%.2f,%.2f,%.2f*", gx_c, gy_c, gz_c);
                uart1_send_string(buffer);
            }
        }

        // $MBATT report at 1 Hz (every tenth window), at PHASE_BATT.
        if (phase == PHASE_BATT && (tick / REPORT_PERIOD) % 10 == 0)
        {
            if (log_batt_enable)
            {
                sprintf(buffer, "$MBATT,%.2f*", adc_read_battery());
                uart1_send_string(buffer);
            }
        }

        tick++;
        

        // Run the FSM last, on this tick's freshly read sensor values
        fsm_update(reference, current_distance, current_yaw);

        tmr_wait_period(TIMER1);
    }

    return 0;
}
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(void)
{
    IFS1bits.INT1IF = 0;
    // Only the immediate emergency stop runs in the interrupt (Obstacle state).
    // The Halted<->Moving toggle is handled by debounced polling in the loop.
    fsm_emergency_from_isr();
}
