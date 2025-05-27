// /*
//  * Ultrasonic ranger/Servo driver header source file
//  * This file will contain functions for servo/ultrasonic control
//  * 
//  * Functionality:
//  * - Utilise CLI to measure object distance from sensor
//  * - Move the servo using PWM
//  * 
//  * REF: 
//  * https://github.com/zephyrproject-rtos/zephyr/tree/main/
//  */

// #ifndef ULTRASONIC_H
// #define ULTRASONIC_H

// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/pwm.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/shell/shell.h>
// #include <zephyr/sys/ring_buffer.h>
// #include <zephyr/logging/log.h>
// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/hci.h>

// // Get the node identifiers using aliases
// #define TRIGGER_NODE  DT_ALIAS(trigger)
// #define ECHO_NODE1    DT_ALIAS(echo1)
// #define SERVO_NODE    DT_ALIAS(servo)
 
// #if !DT_NODE_HAS_STATUS(TRIGGER_NODE, okay)
// #error "Trigger pin node is not okay"
// #endif
 
// #if !DT_NODE_HAS_STATUS(ECHO_NODE1, okay)
// #error "Echo pin 1 node is not okay"
// #endif

// #if !DT_NODE_HAS_STATUS(SERVO_NODE, okay)
// #error "Servo node is not okay"
// #endif
 
// // Get GPIO specs from device tree
// static const struct gpio_dt_spec trig = GPIO_DT_SPEC_GET(TRIGGER_NODE, gpios);
// static const struct gpio_dt_spec echo1 = GPIO_DT_SPEC_GET(ECHO_NODE1, gpios);
// static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(SERVO_NODE);
 
// // PWM defines
// #define PWM_PERIOD_NS 20000000  // 20ms (50Hz)
// #define SERVO_MIN_PULSE_NS 1000000    
// #define SERVO_MAX_PULSE_NS 3000000

// // Warning distance macro
// #define WARNING_DIST 20 // Object warning distance
 
// // Thread stack size and priority
// #define ULTRA_STACK_SIZE 1024
// #define ULTRA_PRIORITY 5
// #define SERVO_STACK_SIZE 512
// #define SERVO_PRIORITY 4
// #define MAIN_STACK_SIZE 1024
// #define MAIN_PRIORITY 3

// static struct k_thread ultra_thread_data;
// static struct k_thread servo_thread_data;
// static struct k_thread main_thread_data;

// K_THREAD_STACK_DEFINE(ultra_stack_area, ULTRA_STACK_SIZE);
// static k_tid_t ultra_tid;

// K_THREAD_STACK_DEFINE(servo_stack_area, SERVO_STACK_SIZE);
// static k_tid_t servo_tid;

// K_THREAD_STACK_DEFINE(main_stack_area, main_STACK_SIZE);
// static k_tid_t main_tid;

// int angle;

// // BLE Advertising params
// const struct bt_le_adv_param adv_params_distance = {
//     .options = BT_LE_ADV_OPT_USE_IDENTITY,
//     .interval_min = 0x00A0,
//     .interval_max = 0x00F0,
//     .peer = NULL
// };

// void broadcast_distance(uint16_t distance_cm);
// void initialise_bluetooth(void);
// int servo_set_angle(uint8_t angle_deg);
// void servo_thread(void *a, void *b, void *c);
// void ultrasonic_thread(void *a, void *b, void *c);
// void start_ultrasonic_thread(void *a, void *b, void *c);

// #endif