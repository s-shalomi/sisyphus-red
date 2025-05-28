#include "recieve.h"
#include "process_packets.h"
#include <string.h>

struct k_fifo car_info_queue;
struct k_fifo pathfinding_queue;

int found_obstacle = 0;

void processing(void) {
    k_fifo_init(&car_info_queue);
    k_fifo_init(&pathfinding_queue);
    printk("processing\n");

    while (1) {
        printk("processing\n");
		// struct data_item_t *rx_data = k_fifo_get(&packets_queue, K_FOREVER);
        // process data

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