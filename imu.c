#include <xc.h>
#include "imu.h"
#include "spi.h"
#include "timer.h"

// SPI Defines
#define MAG_CS LATDbits.LATD6    
#define MAG_CS_DIR TRISDbits.TRISD6

#define ACCEL_CS LATBbits.LATB3  
#define ACCEL_CS_DIR TRISBbits.TRISB3

#define GYRO_CS LATBbits.LATB4   
#define GYRO_CS_DIR TRISBbits.TRISB4


void imu_init(void) {
    ACCEL_CS_DIR = 0;
    GYRO_CS_DIR = 0;
    MAG_CS_DIR = 0;
    ACCEL_CS = 1;
    GYRO_CS = 1;
    MAG_CS = 1;

    // Magnetometer
    mag_write_reg(0x4B, 0x01); // set power bit - Sleep
    tmr_wait_ms(TIMER2, 3);
    mag_write_reg(0x4C, 0x00); // OpMode normal - Active
    tmr_wait_ms(TIMER2, 3);
}

void accel_write_reg(unsigned char addr, unsigned char value) {
    ACCEL_CS = 0;
    spi_write(addr & 0x7F); 
    spi_write(value);
    ACCEL_CS = 1;
}

AxesRaw_t accel_read_xyz(void) {
    AxesRaw_t accel;
    unsigned int lsb, msb;

    ACCEL_CS = 0;
    
    // Send the start address (0x02 for ACC) with the Read bit (0x80)
    spi_write(0x02 | 0x80); 
    
    // Convert 14-bit signed accelerometer values

    // Read x-axis (14-bit)
    lsb = spi_write(0x00);
    msb = spi_write(0x00);
    accel.x = (int)(((msb << 8) | (lsb & 0x00F0))) >> 4;

    // Read y-axis (14-bit)
    lsb = spi_write(0x00);
    msb = spi_write(0x00);
    accel.y = (int)(((msb << 8) | (lsb & 0x00F0))) >> 4;

    // Read z-axis (14-bit)
    lsb = spi_write(0x00);
    msb = spi_write(0x00);
    accel.z = (int)(((msb << 8) | (lsb & 0x00F0))) >> 4;

    ACCEL_CS = 1;

    return accel;
}

void mag_write_reg(unsigned char addr, unsigned char value) {
    MAG_CS = 0;
    spi_write(addr & 0x7F);
    spi_write(value);
    MAG_CS = 1;
}

AxesRaw_t mag_read_xyz(void) {
    AxesRaw_t mag;
    unsigned int lsb, msb;

    MAG_CS = 0;

    // Send the start address (0x42 for MAG X) with the Read bit (0x80)
    spi_write(0x42 | 0x80);

    // Convert signed magnetometer values

    // Read x-axis (13-bit)
    lsb = spi_write(0x00);
    msb = spi_write(0x00);
    mag.x = (int)(((msb << 8) | (lsb & 0x00F8))) >> 3;

    // Read y-axis (13-bit)
    lsb = spi_write(0x00);
    msb = spi_write(0x00);
    mag.y = (int)(((msb << 8) | (lsb & 0x00F8))) >> 3;

    // Read z-axis (15-bit)
    lsb = spi_write(0x00);
    msb = spi_write(0x00);
    mag.z = (int)(((msb << 8) | (lsb & 0x00FE))) >> 1;

    MAG_CS = 1;

    return mag;
}
void gyro_write_reg(unsigned char addr, unsigned char value) {
    GYRO_CS = 0;
    spi_write(addr & 0x7F);
    spi_write(value);
    GYRO_CS = 1;
}

AxesRaw_t gyro_read_xyz(void) {
    AxesRaw_t gyro;
    unsigned int lsb, msb;

    GYRO_CS = 0;

    // Send the start address (0x02 for RATE_X_LSB) with the Read bit (0x80)
    spi_write(0x02 | 0x80);

    // Read x-axis (full 16-bit, no shifting needed)
    lsb = spi_write(0x00);
    msb = spi_write(0x00);
    gyro.x = (int)((msb << 8) | lsb);

    // Read y-axis
    lsb = spi_write(0x00);
    msb = spi_write(0x00);
    gyro.y = (int)((msb << 8) | lsb);

    // Read z-axis
    lsb = spi_write(0x00);
    msb = spi_write(0x00);
    gyro.z = (int)((msb << 8) | lsb);

    GYRO_CS = 1;

    return gyro;
}