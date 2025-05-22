#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "process_packets.h"
#include "pathfinding.h"

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7


void send_json(void) {
    // get info from car_info queue
    // if obstacle detected, draw new path
    int count = 0;
    while (1) {
        printk("sending json %d \n", count);
        count++;
        k_sleep(K_MSEC(200));
    }
}


K_THREAD_DEFINE(process_id, STACKSIZE, processing, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(json_id, STACKSIZE, send_json, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(pathfinding_id, STACKSIZE, draw_map, NULL, NULL, NULL, PRIORITY + 2, 0, 0); // lower priority
