#ifndef PROCESS_PACKETS_H
#define PROCESS_PACKETS_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>

extern struct k_fifo car_info_queue;
extern struct k_fifo pathfinding_queue;

struct car_info {
    int obstacle_detected;
    int obstacle_x;
    int obstacle_y;
    int car_x;
    int car_y;
};

extern void processing(void);

struct car_info_data_item_t {
    void *fifo_reserved; /* 1st word reserved for use by FIFO */
    struct car_info data;
};

extern float kalman_init_x;
extern float kalman_init_y;

#endif