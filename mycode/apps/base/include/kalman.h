#ifndef KALMAN_H
#define KALMAN_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <math.h>
#include <stdio.h>

#define STATE_SIZE 4
#define NUM_MEAS 2

typedef struct {
    float transition[STATE_SIZE][STATE_SIZE]; // State transition matrix
    float measurement[NUM_MEAS][STATE_SIZE]; // Measurement matrix
    float state_vector[STATE_SIZE];  // estimated state vector
    float covariance[STATE_SIZE][STATE_SIZE]; // State covariance matrix
    float process_noise[STATE_SIZE][STATE_SIZE]; // Process noise covariance
    float measurement_noise[NUM_MEAS][NUM_MEAS]; // Measurement noise covariance

    float state_estimate[STATE_SIZE]; // Estimated state vector
    float covariance_estimate[STATE_SIZE][STATE_SIZE]; // Estimated covariance matrix
    float meas_error[NUM_MEAS]; // Measurement error
    float cov_error[NUM_MEAS][NUM_MEAS]; // Measurement covariance
    float kalman_gain[STATE_SIZE][NUM_MEAS]; // Kalman gain
} KalmanFilter;

extern void kalman_init(KalmanFilter *kf, float *x_init, float *cov_init, float meas_err, float proc_err, float dt);
extern void kalman_update(KalmanFilter *kf, float *obs);

#endif