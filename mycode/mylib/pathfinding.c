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

int initial_map[ROWS][COLUMNS] = {
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1}
};

struct point start = {0, 1};
struct point end = {5, 6};

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
    // int trace_size = 10;
    int size = 0; // '(x, y) >' - start with end pos
    // char* path = (char*)k_malloc(sizeof(char) * size);
    char path[1024];

    int offset = 0;

    // from end node to start node
    struct node* current = end_node;
    printf("Tracing path:\n");
    while (current != NULL) {
        offset += snprintf(path + offset, sizeof(path) - offset, "(%d, %d) > ", current->x, current->y);
        size += strlen(path);
        // size += trace_size; // increase size for each node
        // path = (char*)realloc(path, sizeof(char) * size);
        // map[current->x][current->y] = 2; // mark path with 2
        current = current->parent;

    }
    path[size] = '\0';
    
    int i = 0;
    while (path[i] != '\0') {
        printf("%c", path[i]);
        i++;
    }
    printf("\n");

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
        printf("Invalid start or end position\n");
        return;
    }

    // check if at end position
    if (at_end(start, end)) {
        printf("Already at end position\n");
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
                printf("Error\n");
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

    printf("No path found\n");
    free_pqueue(discovered_queue);
    free_discovered_info(discovered_info);
}


void draw_map(void) {
    printk("init draw\n");
    printk("Map state with start %d %d and end %d %d\n", start.x, start.y, end.x, end.y);
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLUMNS; j++) {
            if (i == start.x && j == start.y) {
                printk(" S ");
            } else if (i == end.x && j == end.y) {
                printk(" E ");
            } else {
                printk(" %d ", initial_map[i][j]);
            }
        }
        printk("\n");
    }
    a_star_search(initial_map, start, end);
    while (1) {
        struct car_info_data_item_t *rx_data = k_fifo_get(&pathfinding_queue, K_FOREVER);
        initial_map[rx_data->data.obstacle_x][rx_data->data.obstacle_y] = 0;
        struct point car_pos = {.x = rx_data->data.car_x, .y = rx_data->data.car_y};

        printk("Map state with start %d %d and end %d %d\n", car_pos.x, car_pos.y, end.x, end.y);
        for (int i = 0; i < ROWS; i++) {
            for (int j = 0; j < COLUMNS; j++) {
                if (i == car_pos.x && j == car_pos.y) {
                    printk(" S ");
                } else if (i == end.x && j == end.y) {
                    printk(" E ");
                } else {
                    printk(" %d ", initial_map[i][j]);
                }
            }
            printk("\n");
        }
        a_star_search(initial_map, car_pos, end);
		k_free(rx_data); // fifo gets pointer to memory location of tx_data

        k_sleep(K_MSEC(200));
    }
}
