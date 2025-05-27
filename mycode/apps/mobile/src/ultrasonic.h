#ifndef ULTRASONIC_H 
#define ULTRASONIC_H 
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

extern void create_ultrasonic_thread(void);