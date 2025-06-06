#ifndef RECIEVE_H
#define RECIEVE_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/addr.h>
#include <math.h>

#define MOBILE_ADDR "C4:BC:8A:CE:12:70" 
#define SENSOR_ADDR "D9:C5:AF:3E:87:62" 
#define MOBILE_ADDR_TYPE "random"
#define SENSOR_ADDR_TYPE "random"

typedef struct packet_data {
	double displacement_x; // sensor
	double displacement_y;
	double mouse_x;
	double mouse_y;
	int obstacle_detected;
	double obstacle_dist;
	int obstacle_dir;
} packet_data;

extern struct k_fifo packets_queue;

struct data_item_t {
    void *fifo_reserved; /* 1st word reserved for use by FIFO */
    packet_data data;
};

extern void recieve_packets(void);

#endif