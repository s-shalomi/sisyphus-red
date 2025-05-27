#ifndef INTEGRATION_H
#define INTEGRATION_H
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

extern void create_integration_thread(void);
extern void initialise_bluetooth(void);
