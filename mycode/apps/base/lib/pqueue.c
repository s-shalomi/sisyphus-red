#include "pqueue.h"

PQueue* init_pqueue() {
    int inital_capacity = 256;
    PQueue* pqueue = (PQueue*)k_malloc(sizeof(PQueue));
    pqueue->entries = (Entry*)k_malloc(sizeof(Entry) * inital_capacity);

    pqueue->num_items = 0;
    pqueue->capacity = inital_capacity;
    return pqueue;
}

void free_pqueue(PQueue* pqueue) {
    if (pqueue) {
        k_free(pqueue->entries);
        k_free(pqueue);
    }
}

void resize_pqueue(PQueue* pqueue) {
    int old_capacity = pqueue->capacity;
    int old_num_items = pqueue->num_items;
    pqueue->capacity = old_capacity * 2;
    
    Entry* old_entries = pqueue->entries;
    Entry* new_entries = (Entry*)k_malloc(sizeof(Entry) * pqueue->capacity);
    pqueue->entries = new_entries;
    pqueue->num_items = 0;
    memset(new_entries, 0, sizeof(Entry) * pqueue->capacity);
    
    for (int i = 0; i < old_num_items; i++) {
        int priority = old_entries[i].priority;
        void* data = old_entries[i].data;
        insert_pqueue(pqueue, priority, data);
    }
    k_free(old_entries);

}

int get_parent(int index) {
    return (index - 1) / 2;
}

int get_left_child(int index) {
    return 2 * index + 1;
}

int get_right_child(int index) {
    return 2 * index + 2;
}

void swap_entries(Entry* a, Entry* b) {
    Entry temp = *a;
    *a = *b;
    *b = temp;
}

void insert_pqueue(PQueue* pqueue, int priority, void* data) {
    if (pqueue->num_items >= pqueue->capacity) {
        resize_pqueue(pqueue);
    }

    Entry new_entry;
    new_entry.priority = priority;
    new_entry.data = data;

    // append new element to back
    pqueue->entries[pqueue->num_items] = new_entry;
    pqueue->num_items++;

    int current_index = pqueue->num_items - 1;

    // restore heap property
    while (current_index > 0 && pqueue->entries[get_parent(current_index)].priority > pqueue->entries[current_index].priority) {
        swap_entries(&pqueue->entries[current_index], &pqueue->entries[get_parent(current_index)]);
        current_index = get_parent(current_index);
    }

}

Entry remove_pqueue_min(PQueue* pqueue) {
    if (pqueue->num_items == 0) {
        return (Entry){0, NULL};
    }

    Entry min_entry = pqueue->entries[0];
    
    pqueue->entries[0] = pqueue->entries[pqueue->num_items - 1];
    pqueue->num_items--;

    int index = 0;
    while (index < pqueue->num_items) {
        int left_child_index = get_left_child(index);
        int right_child_index = get_right_child(index);
        int min_index = index;

        // find smallest child
        if (left_child_index < pqueue->num_items && pqueue->entries[left_child_index].priority < pqueue->entries[min_index].priority) {
            min_index = left_child_index;
        }
        if (right_child_index < pqueue->num_items && pqueue->entries[right_child_index].priority < pqueue->entries[min_index].priority) {
            min_index = right_child_index;
        }

        // if smallest child was changed, swap
        if (min_index != index) {
            swap_entries(&pqueue->entries[index], &pqueue->entries[min_index]);
            index = min_index;
        } else {
            break;
        }
    }
    return min_entry;
}

int get_pqueue_size(PQueue* pqueue) {
    return pqueue->num_items;
}