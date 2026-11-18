#ifndef MOTOR_H
#define MOTOR_H

void motor_init(void);
void motor_set(int left_pwm, int right_pwm);
void motor_set_max_speed(int percent);

#endif
