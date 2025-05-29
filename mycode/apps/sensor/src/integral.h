#ifndef INTEGRAL_H
#define INTEGRAL_H
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>

#define STATE_SIZE 4
#define MEASUREMENT_SIZE 2
#define CONTROL_SIZE 2

typedef struct {
    float state_vector[STATE_SIZE];                   // x, y, vx, vy
    float error_covariance[STATE_SIZE][STATE_SIZE];   // P
    float transition[STATE_SIZE][STATE_SIZE];         // A
    float observation[MEASUREMENT_SIZE][STATE_SIZE];  // H
    float control_matrix[STATE_SIZE][CONTROL_SIZE];   // B
    float process_noise[STATE_SIZE][STATE_SIZE];      // Q
    float measurement_noise[MEASUREMENT_SIZE][MEASUREMENT_SIZE]; // R
} KalmanFilter;

extern int integral();
extern void kalman_init(KalmanFilter *kf, float dt); 
extern void kalman_update(KalmanFilter *kf, float *measurement, float *accel);
extern void print_float(const char *label, float f);