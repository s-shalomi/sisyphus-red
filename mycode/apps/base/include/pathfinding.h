#ifndef PATHFINDING_H
#define PATHFINDING_H

#define ROWS 6
#define COLUMNS 8
#define CELL_WIDTH 255 // mm
#define CELL_HEIGHT 255 // mm

#define M_PI 3.14159265358979323846f

#define NUM_NEIGHBORS 4

#define INF 999999999999

#define MANUAL 0
#define AUTO 1

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
extern int mode;

extern void draw_map(void);


#endif