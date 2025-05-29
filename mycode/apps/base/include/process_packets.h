#ifndef PROCESS_PACKETS_H
#define PROCESS_PACKETS_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define WEST 0
#define NORTHWEST 1
#define NORTH 2
#define NORTHEAST 3
#define EAST 4

extern struct k_fifo car_info_queue;
extern struct k_fifo pathfinding_queue;

struct car_info {
    int obstacle_detected;
    int obstacle_dist;
    int obstacle_direction; // 0 = west, 1 = northwest, 2 = north, 3 = northeast, 4 = east
    float car_x;
    float car_y;
};

extern void processing(void);

struct car_info_data_item_t {
    void *fifo_reserved; /* 1st word reserved for use by FIFO */
    struct car_info data;
};

extern float kalman_init_x;
extern float kalman_init_y;

#endif