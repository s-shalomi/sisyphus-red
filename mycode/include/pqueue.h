#ifndef PQUEUE_H
#define PQUEUE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


typedef struct Entry {
    int priority;
    void* data;
} Entry;

typedef struct PQueue {
    Entry* entries;
    int num_items;
    int capacity;
} PQueue;

extern PQueue* init_pqueue();
extern void free_pqueue(PQueue* pqueue);
extern void resize_pqueue(PQueue* pqueue);
int get_parent(int index);
int get_left_child(int index);
int get_right_child(int index);
void swap_entries(Entry* a, Entry* b);
extern void insert_pqueue(PQueue* pqueue, int priority, void* data);
extern Entry remove_pqueue_min(PQueue* pqueue);
extern int get_pqueue_size(PQueue* pqueue);

#endif