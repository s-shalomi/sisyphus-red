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
        printk("invalid coordinates\n");
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

    KalmanFilter kf;
    kalman_init(&kf, state_init, cov_init, meas_err, proc_err, dt);

    while (1) {
        printk("processing\n");
		// struct data_item_t *rx_data = k_fifo_get(&packets_queue, K_FOREVER);
        // process data

        float obs[NUM_MEAS] = {kalman_init_x, kalman_init_y}; // simulated observation data
        kalman_update(&kf, obs);
        
        printk("Kalman Filter State Estimate: [");
        print_float("", kf.state_estimate[0]);
        printk(", ");
        print_float("", kf.state_estimate[1]);
        printk(", ");
        print_float("", kf.state_estimate[2]);
        printk(", ");
        print_float("", kf.state_estimate[3]);
        printk("]\n");


        


        // send processed data in queue
		// k_free(rx_data); // fifo gets pointer to memory location of tx_data
        



        // struct car_info processed_data;
        // memset(&processed_data, 0, sizeof(processed_data));

        // struct car_info_data_item_t tx_data = {.data = processed_data};

        // size_t size = sizeof(struct car_info_data_item_t);
        // char *mem_ptr = k_malloc(size); // create pointer to memory location
        // __ASSERT_NO_MSG(mem_ptr != 0); // ensure mem_ptr is valid

        // memcpy(mem_ptr, &tx_data, size);

        // k_fifo_put(&car_info_queue, mem_ptr);

        if (found_obstacle % 30 == 0) {
            // printk("sending to redraw\n");
            
            // if obstacle detected, send info to pathfinding
            struct car_info obstacle_data;
            memset(&obstacle_data, 0, sizeof(obstacle_data));
            obstacle_data.obstacle_detected = 1;
            obstacle_data.obstacle_x = 2;
            obstacle_data.obstacle_y = 3;
            obstacle_data.car_x = 1;
            obstacle_data.car_y = 2;

    
            struct car_info_data_item_t tx_data_obstacle = {.data = obstacle_data};
    
            size_t size_obstacle = sizeof(struct car_info_data_item_t);
            char *mem_ptr_obstacle = k_malloc(size_obstacle); // create pointer to memory location
            __ASSERT_NO_MSG(mem_ptr_obstacle != 0); // ensure mem_ptr is valid
    
            memcpy(mem_ptr_obstacle, &tx_data_obstacle, size_obstacle);
    
            k_fifo_put(&pathfinding_queue, mem_ptr_obstacle);
        }
        found_obstacle++;




        k_sleep(K_MSEC(200));
    }
}