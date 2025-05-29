#include "integral.h"

// Helper matrix functions
void matrix_multiply(float *result, float *a, float *b, int m, int n, int p) {
    for (int i = 0; i < m; i++)
        for (int j = 0; j < p; j++) {
            result[i * p + j] = 0.0f;
            for (int k = 0; k < n; k++)
                result[i * p + j] += a[i * n + k] * b[k * p + j];
        }
}

void matrix_add(float *result, float *a, float *b, int m, int n) {
    for (int i = 0; i < m * n; i++)
        result[i] = a[i] + b[i];
}

void matrix_subtract(float *result, float *a, float *b, int m, int n) {
    for (int i = 0; i < m * n; i++)
        result[i] = a[i] - b[i];
}

void matrix_transpose(float *result, float *a, int m, int n) {
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            result[j * m + i] = a[i * n + j];
}

void matrix_copy(float *dst, float *src, int m, int n) {
    memcpy(dst, src, sizeof(float) * m * n);
}

void matrix_identity(float *matrix, int size) {
    memset(matrix, 0, sizeof(float) * size * size);
    for (int i = 0; i < size; i++)
        matrix[i * size + i] = 1.0f;
}

// Inversion for 2x2 matrix only
void matrix_inverse_2x2(float *result, float *matrix) {
    float det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
    if (det != 0.0f) {
        float inv_det = 1.0f / det;
        result[0] =  matrix[3] * inv_det;
        result[1] = -matrix[1] * inv_det;
        result[2] = -matrix[2] * inv_det;
        result[3] =  matrix[0] * inv_det;
    }
}

// Initialize Kalman Filter
void kalman_init(KalmanFilter *kf, float dt) {
    memset(kf, 0, sizeof(KalmanFilter));

    // Transition matrix A
    kf->transition[0][0] = 1.0f; kf->transition[0][2] = dt;
    kf->transition[1][1] = 1.0f; kf->transition[1][3] = dt;
    kf->transition[2][2] = 1.0f;
    kf->transition[3][3] = 1.0f;

    // Control matrix B
    kf->control_matrix[0][0] = 0.5f * dt * dt;
    kf->control_matrix[1][1] = 0.5f * dt * dt;
    kf->control_matrix[2][0] = dt;
    kf->control_matrix[3][1] = dt;

    // Observation matrix H (measuring position only)
    kf->observation[0][0] = 1.0f;
    kf->observation[1][1] = 1.0f;

    // Initial error covariance (P)
    for (int i = 0; i < STATE_SIZE; i++)
        kf->error_covariance[i][i] = 1.0f;

    // Process noise (Q)
    for (int i = 0; i < STATE_SIZE; i++)
        kf->process_noise[i][i] = 0.01f;

    // Measurement noise (R)
    for (int i = 0; i < MEASUREMENT_SIZE; i++)
        kf->measurement_noise[i][i] = 1.0f;
}

// Kalman Filter update
void kalman_update(KalmanFilter *kf, float *measurement, float *accel) {
    float temp1[STATE_SIZE * 1];
    float temp2[STATE_SIZE * 1];
    float temp3[STATE_SIZE * STATE_SIZE];
    float temp4[MEASUREMENT_SIZE * 1];
    float temp5[MEASUREMENT_SIZE * 1];
    float Ht[STATE_SIZE * MEASUREMENT_SIZE];
    float S[MEASUREMENT_SIZE * MEASUREMENT_SIZE];
    float K[STATE_SIZE * MEASUREMENT_SIZE];
    float I[STATE_SIZE * STATE_SIZE];

    // Predict step: x' = A * x + B * u
    matrix_multiply(temp1, (float *)kf->transition, kf->state_vector, STATE_SIZE, STATE_SIZE, 1); // A*x
    matrix_multiply(temp2, (float *)kf->control_matrix, accel, STATE_SIZE, CONTROL_SIZE, 1);       // B*u
    matrix_add(kf->state_vector, temp1, temp2, STATE_SIZE, 1);                                   // x' = A*x + B*u

    // Predict error covariance: P' = A * P * A^T + Q
    matrix_multiply(temp3, (float *)kf->transition, (float *)kf->error_covariance, STATE_SIZE, STATE_SIZE, STATE_SIZE);
    float At[STATE_SIZE * STATE_SIZE];
    matrix_transpose(At, (float *)kf->transition, STATE_SIZE, STATE_SIZE);
    float P_pred[STATE_SIZE * STATE_SIZE];
    matrix_multiply(P_pred, temp3, At, STATE_SIZE, STATE_SIZE, STATE_SIZE);
    matrix_add((float *)kf->error_covariance, P_pred, (float *)kf->process_noise, STATE_SIZE, STATE_SIZE);

    // Update step
    float H[MEASUREMENT_SIZE * STATE_SIZE];
    memcpy(H, kf->observation, sizeof(float) * MEASUREMENT_SIZE * STATE_SIZE);
    matrix_transpose(Ht, H, MEASUREMENT_SIZE, STATE_SIZE);

    float PHt[STATE_SIZE * MEASUREMENT_SIZE];
    matrix_multiply(PHt, (float *)kf->error_covariance, Ht, STATE_SIZE, STATE_SIZE, MEASUREMENT_SIZE);
    float HPHt[MEASUREMENT_SIZE * MEASUREMENT_SIZE];
    matrix_multiply(S, H, PHt, MEASUREMENT_SIZE, STATE_SIZE, MEASUREMENT_SIZE);
    matrix_add(S, S, (float *)kf->measurement_noise, MEASUREMENT_SIZE, MEASUREMENT_SIZE);

    float S_inv[MEASUREMENT_SIZE * MEASUREMENT_SIZE];
    matrix_inverse_2x2(S_inv, S);
    matrix_multiply(K, PHt, S_inv, STATE_SIZE, MEASUREMENT_SIZE, MEASUREMENT_SIZE); // Kalman gain

    matrix_multiply(temp4, H, (float *)kf->state_vector, MEASUREMENT_SIZE, STATE_SIZE, 1);
    matrix_subtract(temp5, measurement, temp4, MEASUREMENT_SIZE, 1); // y = z - Hx

    matrix_multiply(temp1, K, temp5, STATE_SIZE, MEASUREMENT_SIZE, 1);
    matrix_add((float *)kf->state_vector, (float *)kf->state_vector, temp1, STATE_SIZE, 1);

    float KH[STATE_SIZE * STATE_SIZE];
    matrix_multiply(KH, K, H, STATE_SIZE, MEASUREMENT_SIZE, STATE_SIZE);
    matrix_identity(I, STATE_SIZE);
    matrix_subtract((float *)temp3, I, KH, STATE_SIZE, STATE_SIZE);
    matrix_multiply((float *)kf->error_covariance, temp3, (float *)kf->error_covariance, STATE_SIZE, STATE_SIZE, STATE_SIZE);
}

void print_float(const char *label, float f) {
    int whole = (int)f;
    int frac = (int)((f - whole) * 1000); // 3 decimal places
    if (frac < 0) frac = -frac; // Ensure positive fraction

    printk("%s%d.%03d", label, whole, frac);
}

int integral() {
    KalmanFilter kf;
    float dt = 1.0f;  // 1 second timestep
    kalman_init(&kf, dt);

    float measurement[2];
    float acceleration[2];

    for (int i = 0; i < 20; i++) {
        acceleration[0] = i + 1;  // ax = 1, 2, ..., 20
        acceleration[1] = i + 1;  // ay = 1, 2, ..., 20

        // Set measurement equal to predicted position (or set to NULL if skipping update)
        measurement[0] = kf.state_vector[0];
        measurement[1] = kf.state_vector[1];

        kalman_update(&kf, measurement, acceleration);

        printf("step %d, acceleration: [", i + 1);
        print_float("", acceleration[0]);
        printf(", ");
        print_float("", acceleration[1]);
        printf("]\n");
        printf("State Estimate: [");
        print_float("", kf.state_vector[0]);
        printf(", ");
        print_float("", kf.state_vector[1]);
        printf(", ");
        print_float("", kf.state_vector[2]);
        printf(", ");
        print_float("", kf.state_vector[3]);
        printf("]\n");
    }
    return 0;
}