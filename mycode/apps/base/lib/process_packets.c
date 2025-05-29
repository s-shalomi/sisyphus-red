#include "recieve.h"
#include "process_packets.h"
#include "kalman.h"
#include <string.h>
#include <stdlib.h>

struct k_fifo car_info_queue;
struct k_fifo pathfinding_queue;

int found_obstacle = 0;

float kalman_init_x = -1.0;
float kalman_init_y = -1.0;

void print_float(const char *label, float f) {
    int whole = (int)f;
    int frac = (int)((f - whole) * 1000); // 3 decimal places
    if (frac < 0) frac = -frac; // Ensure positive fraction

    printk("%s%d.%03d", label, whole, frac);
}

void processing(void) {
    k_fifo_init(&car_info_queue);
    k_fifo_init(&pathfinding_queue);
    printk("processing\n");

    while (kalman_init_x < 0 || kalman_init_y < 0) {
        // printk("invalid coordinates\n");
        k_sleep(K_MSEC(200));
    }

    // Initialize Kalman filter
    /////////////////////////////////////////////////////
    float proc_err = 0.1;
    float meas_err = 1.0;
    float init_err = 0.1;
    float dt = 0.5; // time step in seconds


    float state_init[4] = {kalman_init_x, kalman_init_y, 0.0, 0.0}; // initial state vector
    float cov_init[STATE_SIZE * STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            cov_init[i * STATE_SIZE + j] = (i == j) ? init_err : 0.0f; // diagonal elements are initial error
        }
    }

    // KalmanFilter kf_mouse;
    // KalmanFilter kf_accel;
    // kalman_init(&kf_mouse, state_init, cov_init, meas_err, proc_err, dt);
    // kalman_init(&kf_accel, state_init, cov_init, meas_err, proc_err, dt);

    while (1) {
        struct data_item_t *rx_data = k_fifo_get(&packets_queue, K_FOREVER);
        printk("processing\n");
        // process data

        // float mouse_data[NUM_MEAS] = {rx_data->data.mouse_x, rx_data->data.mouse_y}; // simulated observation data
        // float accel_data[NUM_MEAS] = {rx_data->data.displacement_x, rx_data->data.displacement_x}; // simulated observation data
        // kalman_update(&kf_mouse, mouse_data);
        // float x_est_mouse = kf_mouse.state_estimate[0];
        // float y_est_mouse = kf_mouse.state_estimate[1];
        // kalman_update(&kf_accel, accel_data);
        // float x_est_accel = kf_accel.state_estimate[0];
        // float y_est_accel = kf_accel.state_estimate[1];

        // // Combine estimates from both Kalman filters
        // float x_est_combined = (x_est_mouse + x_est_accel) / 2.0f;
        // float y_est_combined = (y_est_mouse + y_est_accel) / 2.0f;

        // // Update the Kalman filter state estimate
        // kf_mouse.state_estimate[0] = x_est_combined;
        // kf_mouse.state_estimate[1] = y_est_combined;
        // kf_accel.state_estimate[0] = x_est_combined;
        // kf_accel.state_estimate[1] = y_est_combined;

        // printk("Kalman Filter Combined State Estimate: [");
        // print_float("", x_est_combined);
        // printk(", ");
        // print_float("", y_est_combined);
        // printk("]\n");

        printk("Mouse! X: ");
		print_float("", -rx_data->data.mouse_x);
		printk(", Mouse Y: ");
		print_float("", -rx_data->data.mouse_y);
        printk("\n");


        float x_est_combined = -rx_data->data.mouse_x * 99.9 + rx_data->data.displacement_x * 0.01; // weighted average
        float y_est_combined = -rx_data->data.mouse_y * 99.9 + rx_data->data.displacement_y * 0.01; // weighted average 



        

        struct car_info processed_data;
        memset(&processed_data, 0, sizeof(processed_data));
        processed_data.car_x = x_est_combined;
        processed_data.car_y = y_est_combined;
        // processed_data.car_x = 0;
        // processed_data.car_y = 1;
        
        printk("Car position:");
        print_float("", processed_data.car_x);
        printk(", ");
        print_float("", processed_data.car_y);
        printk("\n");
        int obstacle_angle = rx_data->data.obstacle_dir;
        // int obstacle_angle = 90;

        if (rx_data->data.obstacle_detected) {
            processed_data.obstacle_detected = 1;
            processed_data.obstacle_dist = rx_data->data.obstacle_dist; // to do /////////////////////
            processed_data.obstacle_direction = NORTH;

            if (obstacle_angle == 0) {
                processed_data.obstacle_direction = WEST;
            } else if (obstacle_angle == 45 || obstacle_angle == 30 || obstacle_angle == 15) {
                processed_data.obstacle_direction = NORTHWEST;
            } else if (obstacle_angle == 90) {
                processed_data.obstacle_direction = NORTH;
            } else if (obstacle_angle == 105 || obstacle_angle == 120 || obstacle_angle == 135) {
                processed_data.obstacle_direction = NORTHEAST;
            } else if (obstacle_angle == 180) {
                processed_data.obstacle_direction = EAST;
            } else {
                processed_data.obstacle_direction = -1; // invalid direction
            }
        } else {
            processed_data.obstacle_detected = 0;
            processed_data.obstacle_dist = -1; // no obstacle
            processed_data.obstacle_direction = -1; // no obstacle
        }

        // printk("Obstacle detected %d obstacle dir %d obstacle dist %d\n",
        //        processed_data.obstacle_detected,
        //        processed_data.obstacle_direction,
        //        processed_data.obstacle_dist);

        struct car_info_data_item_t tx_data_obstacle = {.data = processed_data};

        size_t size_processed = sizeof(struct car_info_data_item_t);
        char *mem_ptr_processed = k_malloc(size_processed); // create pointer to memory location
        __ASSERT_NO_MSG(mem_ptr_processed != 0); // ensure mem_ptr is valid

        memcpy(mem_ptr_processed, &tx_data_obstacle, size_processed);

        k_fifo_put(&pathfinding_queue, mem_ptr_processed);

        k_free(rx_data); // fifo gets pointer to memory location of tx_data
        k_sleep(K_MSEC(200));
    }
}