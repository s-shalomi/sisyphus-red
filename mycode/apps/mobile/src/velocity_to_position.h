#ifndef VELOCITY_TO_POSITION_H
#define VELOCITY_TO_POSITION_H
#endif

#include <string.h>
#include <math.h>
#include <zephyr/sys/printk.h>

#define STATE_SIZE 2
#define CONTROL_SIZE 2
#define MEASUREMENT_SIZE 2

typedef struct {
    float state_vector[STATE_SIZE];                  // [x, y]
    float error_covariance[STATE_SIZE][STATE_SIZE];  // P
    float transition[STATE_SIZE][STATE_SIZE];        // A
    float control_matrix[STATE_SIZE][CONTROL_SIZE];  // B
    float process_noise[STATE_SIZE][STATE_SIZE];     // Q
    float measurement_noise[MEASUREMENT_SIZE][MEASUREMENT_SIZE]; // R
} KalmanFilter2D;

extern void kalman2d_init(KalmanFilter2D *kf, float dt);
extern void reset_kalman_filter(KalmanFilter2D *kf);
