#ifndef RECIEVE_H
#define RECIEVE_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/addr.h>

#define MOBILE_ADDR "D9:C5:AF:3E:87:62" 
#define MOBILE_ADDR_TYPE "random"

typedef struct packet_data {
	double displacement_x;
	double displacement_y;
	double mouse_x;
	double mouse_y;
	double obstacle_dist;
	int obstacle_dir;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} packet_data;

extern struct k_fifo packets_queue;

extern struct data_item_t {
    void *fifo_reserved; /* 1st word reserved for use by FIFO */
    packet_data data;
};

#endif