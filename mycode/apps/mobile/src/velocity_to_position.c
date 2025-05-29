
#include "velocity_to_position.h"

#define PROCESS_NOISE 0.09f
#define ERROR_COVARIANCE 1.0f
#define MEASUREMENT_NOISE 1.0f

// Initialization
void kalman2d_init(KalmanFilter2D *kf, float dt) {
    memset(kf, 0, sizeof(KalmanFilter2D));
    // State transition (A)
    kf->transition[0][0] = 1.0f;
    kf->transition[1][1] = 1.0f;

    // Control (B)
    kf->control_matrix[0][0] = dt;
    kf->control_matrix[1][1] = dt;

    // Error covariance (P)
    for (int i = 0; i < STATE_SIZE; i++)
        kf->error_covariance[i][i] = ERROR_COVARIANCE;

    // Process noise (Q)
    for (int i = 0; i < STATE_SIZE; i++)
        kf->process_noise[i][i] = PROCESS_NOISE;

    // Measurement noise (R)
    for (int i = 0; i < MEASUREMENT_SIZE; i++)
        kf->measurement_noise[i][i] = MEASUREMENT_NOISE;
}

// Only prediction step (position update from velocity)
void kalman2d_predict(KalmanFilter2D *kf, float *vel) {
    float temp[STATE_SIZE] = {0};
    // x' = A * x + B * u
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j)
            temp[i] += kf->transition[i][j] * kf->state_vector[j];
        for (int j = 0; j < CONTROL_SIZE; ++j)
            temp[i] += kf->control_matrix[i][j] * vel[j];
    }
    memcpy(kf->state_vector, temp, sizeof(float) * STATE_SIZE);
}

void print_floats(const char *label, float f) {
    int whole = (int)f;
    int frac = (int)((f - whole) * 1000); // 3 decimal places
    if (frac < 0) frac = -frac; // Ensure positive fraction

    printk("%s%d.%03d", label, whole, frac);
}

void reset_kalman_filter(KalmanFilter2D *kf)
{
    for (int i = 0; i < STATE_SIZE; ++i)
        kf->state_vector[i] = 0.0f;
    for (int i = 0; i < STATE_SIZE; ++i)
        for (int j = 0; j < STATE_SIZE; ++j)
            kf->error_covariance[i][j] = (i == j) ? 1.0f : 0.0f;
}


void print_state(const KalmanFilter2D *kf) 
{
    print_floats("", kf->state_vector[0]);
    printk(", ");
    print_floats("", kf->state_vector[1]);
    printk("\n");
}
