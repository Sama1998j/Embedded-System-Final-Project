#ifndef IMU_H
#define IMU_H

// Three-axis sensor reading.
typedef struct {
    int x;
    int y;
    int z;
} AxesRaw_t;

void imu_init(void);
void accel_write_reg(unsigned char addr, unsigned char value);
AxesRaw_t accel_read_xyz(void);
void mag_write_reg(unsigned char addr, unsigned char value);
AxesRaw_t mag_read_xyz(void);
void gyro_write_reg(unsigned char addr, unsigned char value);
AxesRaw_t gyro_read_xyz(void);

#endif

