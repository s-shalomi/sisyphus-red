#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <limits.h>
#include "pqueue.h"

// adapted from https://www.geeksforgeeks.org/a-search-algorithm/

#define ROWS 6
#define COLUMNS 8

#define NUM_NEIGHBORS 8
const int dx[NUM_NEIGHBORS] = {-1, -1, -1, 0, 0, 1, 1, 1};
const int dy[NUM_NEIGHBORS] = {0, 1, -1, 1, -1, 0, 1, -1};

struct point {
    int x;
    int y;
};

struct node {
    int x;
    int y;
    float final_weight;
    float path_weight;
    float heuristic_weight;
    struct node* parent;
    bool discovered;
    bool visited;
};

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
    int trace_size = 10;
    int size = trace_size; // '(x, y) >' - start with end pos
    char* path = (char*)malloc(sizeof(char) * size);

    // from end node to start node
    struct node* current = end_node;
    printf("Tracing path:\n");
    while (current != NULL) {
        sprintf(path + strlen(path), "(%d, %d) > ", current->x, current->y);
        size += trace_size; // increase size for each node
        path = (char*)realloc(path, sizeof(char) * size);
        printf("(%d, %d) > ", current->x, current->y);
        map[current->x][current->y] = 2; // mark path with 2
        current = current->parent;

    }
    printf("\n");

    map[end.x][end.y] = 3; // mark end with 3
    map[start.x][start.y] = 4; // mark start with 4

    printf("New map:\n");
    // print the path on the map
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLUMNS; j++) {
            if (map[i][j] == 1) {
                printf(" %d ", map[i][j]);
            } else if (map[i][j] == 3) {
                printf(" E ");
            } else if (map[i][j] == 4) {
                printf(" S ");
            } else {
                printf(" o ");
            }
        }
        printf("\n");
    }
}

void free_discovered_info(struct node* discovered_info[ROWS][COLUMNS]) {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLUMNS; j++) {
            free(discovered_info[i][j]);
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
            struct node* node = malloc(sizeof(struct node));
            if (i == start.x && j == start.y) {
                node->x = start.x;
                node->y = start.y;
                node->path_weight = 0;
                node->heuristic_weight = get_heuristic(start, end);
                node->final_weight = node->path_weight + node->heuristic_weight;
                node->parent = NULL;
                node->discovered = true;
                insert_pqueue(discovered_queue, node->final_weight, node);
            } else {
                node->x = i;
                node->y = j;
                node->path_weight = INFINITY;
                node->heuristic_weight = INFINITY;
                node->final_weight = INFINITY;
                node->parent = NULL;
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
    return;
}

int main(void) {
    a_star_search(initial_map, start, end);
    return 0;
}