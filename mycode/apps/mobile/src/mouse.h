#ifndef MOUSE_H 
#define MOUSE_H 
#endif

// c libraries
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

// standard zephyr
#include <zephyr/init.h>
#include <zephyr/kernel.h> 
#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

// terminal
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

// bluetooth
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

extern void create_mouse_thread(void);

struct PositionSender {
    int xInt;
    int xFrac;
    int yInt;
    int yFrac;
};

extern struct PositionSender positionSender;

extern struct k_sem connectedSem;
