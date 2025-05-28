#include "kalman.h"

// adapted from https://www.geeksforgeeks.org/c-matrix-multiplication/
void matrix_multiply(float *result, float *mat1, float *mat2, int r1, int c1, int c2) {
    for (int i = 0; i < r1; i++) {
        for (int j = 0; j < c2; j++) {
            result[i * c2 + j] = 0.0; // Initialize result element
            for (int k = 0; k < c1; k++) {
                result[i * c2 + j] += mat1[i * c1 + k] * mat2[k * c2 + j];
            }
        }
    }
}

// https://www.geeksforgeeks.org/c-transpose-matrix/
void matrix_transpose(float *result, float *mat, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result[j * rows + i] = mat[i * cols + j]; // Swaps row/column indices
        }
    }
}

void matrix_add(float *result, float *mat1, float *mat2, int rows, int cols) {
    for (int i = 0; i < rows * cols; i++) {
        result[i] = mat1[i] + mat2[i];
    }
}

void matrix_inverse_2x2(float *result, float *mat) {
    float det = mat[0] * mat[3] - mat[1] * mat[2]; // Calculate determinant
    if (det == 0) {
        printk("Determinant is zero, cannot invert 2x2 matrix.\n");
        return;
    }
    float inv_det = 1.0 / det;
    // Apply 2x2 inverse formula
    result[0] = mat[3] * inv_det;
    result[1] = -mat[1] * inv_det;
    result[2] = -mat[2] * inv_det;
    result[3] = mat[0] * inv_det;
}


void kalman_init(KalmanFilter *kf, float *x_init, float *cov_init, float meas_err, float proc_err, float dt) {
    // initialize state transition matrix
    kf->transition[0][0] = 1.0; kf->transition[0][1] = 0.0; kf->transition[0][2] = dt;  kf->transition[0][3] = 0.0;
    kf->transition[1][0] = 0.0; kf->transition[1][1] = 1.0; kf->transition[1][2] = 0.0; kf->transition[1][3] = dt;
    kf->transition[2][0] = 0.0; kf->transition[2][1] = 0.0; kf->transition[2][2] = 1.0; kf->transition[2][3] = 0.0;
    kf->transition[3][0] = 0.0; kf->transition[3][1] = 0.0; kf->transition[3][2] = 0.0; kf->transition[3][3] = 1.0;

    // initialize measurement matrix
    kf->measurement[0][0] = 1.0; kf->measurement[0][1] = 0.0; kf->measurement[0][2] = 0.0; kf->measurement[0][3] = 0.0;
    kf->measurement[1][0] = 0.0; kf->measurement[1][1] = 1.0; kf->measurement[1][2] = 0.0; kf->measurement[1][3] = 0.0;

    // initialize state vector and covariance matrix
    for (int i = 0; i < STATE_SIZE; i++) {
        kf->state_vector[i] = x_init[i];
        for (int j = 0; j < STATE_SIZE; j++) {
            kf->covariance[i][j] = cov_init[i * STATE_SIZE + j];
        }
    }

    // initialize process noise covariance
    // identify matrix
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            kf->process_noise[i][j] = (i == j) ? proc_err : 0.0f;
        }
    }

    // initialise measurement noise covariance
    // identify matrix
    for (int i = 0; i < NUM_MEAS; i++) {
        for (int j = 0; j < NUM_MEAS; j++) {
            kf->measurement_noise[i][j] = (i == j) ? meas_err : 0.0;
        }
    }
}

void kalman_update(KalmanFilter *kf, float *obs) {
    float transition_transpose[STATE_SIZE][STATE_SIZE];
    float temp1[STATE_SIZE][STATE_SIZE];
    float temp2[STATE_SIZE][STATE_SIZE];

    // make prediction
    matrix_multiply((float*)kf->state_estimate, (float*)kf->transition, kf->state_vector, STATE_SIZE, STATE_SIZE, 1);

    matrix_transpose((float*)transition_transpose, (float*)kf->transition, STATE_SIZE, STATE_SIZE);
    matrix_multiply((float*)temp1, (float*)kf->transition, (float*)kf->covariance, STATE_SIZE, STATE_SIZE, STATE_SIZE);
    matrix_multiply((float*)kf->covariance_estimate, (float*)temp1, (float*)transition_transpose, STATE_SIZE, STATE_SIZE, STATE_SIZE);
    matrix_add((float*)kf->covariance_estimate, (float*)kf->covariance_estimate, (float*)kf->process_noise, STATE_SIZE, STATE_SIZE);

    // update estimate
    float predicted_obs[NUM_MEAS];
    matrix_multiply((float*)predicted_obs, (float*)kf->measurement, kf->state_estimate, NUM_MEAS, STATE_SIZE, 1);
    for (int i = 0; i < NUM_MEAS; i++) {
        kf->meas_error[i] = obs[i] - predicted_obs[i];
    }

    matrix_transpose((float*)transition_transpose, (float*)kf->measurement, NUM_MEAS, STATE_SIZE);
    matrix_multiply((float*)temp1, (float*)kf->measurement, (float*)kf->covariance_estimate, NUM_MEAS, STATE_SIZE, STATE_SIZE);
    matrix_multiply((float*)kf->cov_error, (float*)temp1, (float*)transition_transpose, NUM_MEAS, STATE_SIZE, NUM_MEAS);
    matrix_add((float*)kf->cov_error, (float*)kf->cov_error, (float*)kf->measurement_noise, NUM_MEAS, NUM_MEAS);

    float error_cov_inv[NUM_MEAS][NUM_MEAS];
    matrix_inverse_2x2((float*)error_cov_inv, (float*)kf->cov_error);
    matrix_multiply((float*)temp1, (float*)kf->covariance_estimate, (float*)transition_transpose, STATE_SIZE, STATE_SIZE, NUM_MEAS);
    matrix_multiply((float*)kf->kalman_gain, (float*)temp1, (float*)error_cov_inv, STATE_SIZE, NUM_MEAS, NUM_MEAS);

    float kalman_gain_correction[STATE_SIZE];
    matrix_multiply((float*)kalman_gain_correction, (float*)kf->kalman_gain, (float*)kf->meas_error, STATE_SIZE, NUM_MEAS, 1);
    matrix_add((float*)kf->state_vector, (float*)kf->state_estimate, kalman_gain_correction, STATE_SIZE, 1);

    // update covariance
    float gain_obs[STATE_SIZE][STATE_SIZE];
    float id_minus_gainobs[STATE_SIZE][STATE_SIZE];
    matrix_multiply((float*)gain_obs, (float*)kf->kalman_gain, (float*)kf->measurement, STATE_SIZE, NUM_MEAS, STATE_SIZE);
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            id_minus_gainobs[i][j] = (i == j) ? 1.0f - gain_obs[i][j] : -gain_obs[i][j];
        }
    }
    matrix_multiply((float*)kf->covariance, (float*)id_minus_gainobs, (float*)kf->covariance_estimate, STATE_SIZE, STATE_SIZE, STATE_SIZE);

}