#ifndef PATHFINDING_H
#define PATHFINDING_H

#define ROWS 6
#define COLUMNS 8
#define CELL_WIDTH 10
#define CELL_HEIGHT 10

#define NUM_NEIGHBORS 8

#define INF 999999999999

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

extern struct point start;
extern struct point end;
extern int coords_given;

extern void draw_map(void);


#endif