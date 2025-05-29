#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <limits.h>
#include "pqueue.h"
#include "pathfinding.h"
#include "process_packets.h"

// adapted from https://www.geeksforgeeks.org/a-search-algorithm/


const int dx[NUM_NEIGHBORS] = {-1, -1, -1, 0, 0, 1, 1, 1};
const int dy[NUM_NEIGHBORS] = {0, 1, -1, 1, -1, 0, 1, -1};

struct point start = {-1, -1};
struct point end = {-1, -1}; 
int coords_given = 0;
int mode = MANUAL;
int prev_mode = MANUAL;

// cell is valid if it is within the bounds of the map and not an obstacle
bool is_valid_point(struct point pos, int map[ROWS][COLUMNS]) {
    return (pos.x >= 0 && pos.x < ROWS && pos.y >= 0 && pos.y < COLUMNS && map[pos.x][pos.y] == 1);
}

bool at_end(struct point pos, struct point end) {
    // check if the current position is the end position
    return (pos.x == end.x && pos.y == end.y);
}

double get_heuristic(struct point pos, struct point end) {
    // Euclidean distance - move in any direction to goal
    return sqrt(pow(pos.x - end.x, 2) + pow(pos.y - end.y, 2));
}

void trace_path(struct node* end_node, struct point end, int map[ROWS][COLUMNS]) {
    int size = 0; // '(x, y) >' - start with end pos
    char path[1024];

    int offset = 0;

    // from end node to start node
    struct node* current = end_node;
    while (current != NULL) {
        offset += snprintf(path + offset, sizeof(path) - offset, "(%d, %d) > ", current->x, current->y);
        size += strlen(path);
        // map[current->x][current->y] = 2; // mark path with 2
        current = current->parent;

    }
    path[size] = '\0';

    int i = 0;
    printk("Path: {");
    while (path[i] != '\0') {
        printk("%c", path[i]);
        i++;
    }
    printk("}\n");

    // print obstacle locations
    char obstacles[1024];
    offset = 0;
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLUMNS; j++) {
            if (map[i][j] == 0) { // obstacle
                offset += snprintf(obstacles + offset, sizeof(obstacles) - offset, "(%d, %d) ", i, j);
            }
        }
    }
    obstacles[offset] = '\0';
    printk("Obstacles: {");
    i = 0;
    while (obstacles[i] != '\0') {
        printk("%c", obstacles[i]);
        i++;
    }
    printk("}\n");

}

void free_discovered_info(struct node* discovered_info[ROWS][COLUMNS]) {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLUMNS; j++) {
            k_free(discovered_info[i][j]);
        }
    }
}

void a_star_search(int map[ROWS][COLUMNS], struct point start, struct point end) {
    // check start and end positions are valid
    if (!is_valid_point(start, map) || !is_valid_point(end, map)) {
        printk("Invalid start or end position\n");
        return;
    }

    // check if at end position
    if (at_end(start, end)) {
        printk("Already at end position\n");
        return;
    }
    
    PQueue* discovered_queue = init_pqueue();

    // initialise visited and discovered nodes
    struct node* discovered_info[ROWS][COLUMNS];
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLUMNS; j++) {
            // init nodes
            struct node* node = k_malloc(sizeof(struct node));
            if (i == start.x && j == start.y) {
                node->x = start.x;
                node->y = start.y;
                node->path_weight = 0;
                node->heuristic_weight = get_heuristic(start, end);
                node->final_weight = node->path_weight + node->heuristic_weight;
                node->parent = NULL;
                node->discovered = true;
                node->visited = false;
                insert_pqueue(discovered_queue, node->final_weight, node);
            } else {
                node->x = i;
                node->y = j;
                node->path_weight = INF;
                node->heuristic_weight = INF;
                node->final_weight = INF;
                node->parent = NULL;
                node->visited = false;
                node->discovered = false;
            }
            discovered_info[i][j] = node;
        }
    }
    while (get_pqueue_size(discovered_queue) > 0) {
        Entry entry = remove_pqueue_min(discovered_queue);
        struct node* curr_node = entry.data;
        curr_node->discovered = false;


        struct point pos = {curr_node->x, curr_node->y};
        if (at_end(pos, end)) {
            trace_path(curr_node, end, map); 
            free_pqueue(discovered_queue);
            free_discovered_info(discovered_info);
            return;
        }

        // get neighbours
        for (int i = 0; i < NUM_NEIGHBORS; i++) {
            int nx = curr_node->x + dx[i];
            int ny = curr_node->y + dy[i];
            struct point neighbour_pos = {nx, ny};

            if (!is_valid_point(neighbour_pos, map)) {
                continue; // skip blocked points and out of bounds
            }

            
            struct node* neighbour = discovered_info[nx][ny];
            
            if (neighbour == NULL) {
                printk("Error\n");
            }

            if (neighbour->visited) {
                continue; // skip visited points
            }

            float path_weight = curr_node->path_weight + ((dx[i] == 0 || dy[i] == 0) ? 1.0 : 1.414); // straight:1, diagonal: sqrt(2)
            float heuristic_weight = get_heuristic(neighbour_pos, end);
            float final_weight = path_weight + heuristic_weight;
            
            if (final_weight < neighbour->final_weight) {
                neighbour->parent = curr_node;
                neighbour->path_weight = path_weight;
                neighbour-> heuristic_weight = heuristic_weight;
                neighbour-> final_weight = final_weight;

                if (!neighbour->discovered) {
                    insert_pqueue(discovered_queue, neighbour->final_weight, neighbour);
                    discovered_info[nx][ny] = neighbour;
                    neighbour->discovered = true;
                }
            }
        }
        curr_node->visited = true;
    }

    printk("No path found\n");
    free_pqueue(discovered_queue);
    free_discovered_info(discovered_info);
}


void draw_map(void) {

    int initial_map[ROWS][COLUMNS] = {
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1}
    };

    struct point grid_start;
    struct point grid_end;
    grid_start.x = start.x / CELL_WIDTH;
    grid_start.y = start.y / CELL_HEIGHT;
    grid_end.x = end.x / CELL_WIDTH;
    grid_end.y = end.y / CELL_HEIGHT;

    if (coords_given) {
        a_star_search(initial_map, grid_start, grid_end);
    } else {
        printk("Error, start and end points not selected\n");
    }
    

    while (1) {
        if (mode != prev_mode) {
            printk("Mode changed\n");
            
            if (mode == MANUAL) {
                printk("Manual mode\n");
                coords_given = 0;
            } 
            prev_mode = mode;
        }
        printk("drawing\n");
        struct car_info_data_item_t *rx_data = k_fifo_get(&pathfinding_queue, K_FOREVER);
        initial_map[rx_data->data.obstacle_x][rx_data->data.obstacle_y] = 0;
        struct point car_pos;
        
        if (mode == AUTO) {
            car_pos.x = rx_data->data.car_x / CELL_WIDTH;
            car_pos.y = rx_data->data.car_y / CELL_HEIGHT;
        } else if (mode == MANUAL) {
            if (!coords_given) {
                printk("Enter position");
		        k_free(rx_data); // fifo gets pointer to memory location of tx_data
                k_sleep(K_MSEC(200));
                continue;
            }
            car_pos.x = start.x / CELL_WIDTH;
            car_pos.y = start.y / CELL_HEIGHT;
        }

        // set obstacles

        if (coords_given) {
            grid_end.x = end.x / CELL_WIDTH;
            grid_end.y = end.y / CELL_HEIGHT;
            printk("pos %d %d %d %d\n", car_pos.x, car_pos.y, grid_end.x, grid_end.y);
            a_star_search(initial_map, car_pos, grid_end);
        } else {
            printk("Error, start and end points not selected\n");
        }
		k_free(rx_data); // fifo gets pointer to memory location of tx_data

        if (mode == MANUAL) {
            coords_given = 0;
        }
        k_sleep(K_MSEC(200));
    }
}
