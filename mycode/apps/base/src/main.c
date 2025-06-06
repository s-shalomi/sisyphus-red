#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include "process_packets.h"
#include "pathfinding.h"
#include "recieve.h"

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7


void send_json(void) {
    // get info from car_info queue
    // if obstacle detected, draw new path
    int count = 0;
    while (1) {
        // printk("json %d \n", count);
        count++;
        k_sleep(K_MSEC(200));
    }
}


K_THREAD_DEFINE(process_id, STACKSIZE * 5, processing, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(json_id, STACKSIZE, send_json, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(pathfinding_id, STACKSIZE * 4, draw_map, NULL, NULL, NULL, 7, 0, 0); // lower priority
K_THREAD_DEFINE(recieve_id, STACKSIZE * 5, recieve_packets, NULL, NULL, NULL, 4, 0, 0);

// route (x,y) (x,y) - defines the start and end point for path
static int cmd_get_map(const struct shell *sh, size_t argc, char **argv) {
    if (argc != 3) {
        shell_error(sh, "Invalid number of arguments");
        return 0;
    }

    char* start_str = argv[1];
    char* end_str = argv[2];

    int start_x, start_y, end_x, end_y;

    if (sscanf(start_str, "(%d,%d)", &start_x, &start_y) != 2 ||
        sscanf(end_str, "(%d,%d)", &end_x, &end_y) != 2) {
        shell_error(sh, "Invalid format for start or end point. Use (x,y)");
        return 0;
    }

    if (start_x < 0 || start_x >= ROWS || start_y < 0 || start_y >= COLUMNS ||
        end_x < 0 || end_x >= ROWS || end_y < 0 || end_y >= COLUMNS) {
        shell_error(sh, "Start or end point out of bounds");
        return 0;
    }

    start.x = start_x;
    start.y = start_y;
    end.x = end_x;
    end.y = end_y;
    kalman_init_x = start_x;
    kalman_init_y = start_y;
    printk("Start point set to (%d, %d)\n", start.x, start.y);
    printk("End point set to (%d, %d)\n", end.x, end.y);
    
    coords_given = 1;
    return 1;
}

// mode auto or mode man
static int cmd_get_mode(const struct shell *sh, size_t argc, char **argv) {
    if (argc != 2) {
        shell_error(sh, "Invalid number of arguments");
        return 0;
    }

    if (!strcmp(argv[1], "auto")) {
        mode = AUTO;
        printk("Auto mode enabled\n");
    } else if (!strcmp(argv[1], "man")) {
        mode = MANUAL;
        printk("Manual mode enabled\n");
    } else {
        shell_error(sh, "Invalid mode. Use 'auto' or 'man'");
        return 0;
    }
    return 1;
}

SHELL_CMD_ARG_REGISTER(route, NULL, "Defines the start and end point for path", cmd_get_map, 3, 3);
SHELL_CMD_ARG_REGISTER(mode, NULL, "Defines if car position is from input for rc car", cmd_get_mode, 2, 2);
