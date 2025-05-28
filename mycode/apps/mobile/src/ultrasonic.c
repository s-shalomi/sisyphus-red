/*
 * Ultrasonic ranger driver main source file
 * This file will run the ultrasonic sensor
 * 
 * Functionality:
 * - Utilise CLI to measure object distance from sensor
 * 
 * REF: 
 * https://github.com/zephyrproject-rtos/zephyr/tree/main/
 */
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
 
// Get the node identifiers using aliases
#define TRIGGER_NODE  DT_ALIAS(trigger)
#define ECHO_NODE1    DT_ALIAS(echo1)
#define SERVO_NODE    DT_ALIAS(servo)
 
#if !DT_NODE_HAS_STATUS(TRIGGER_NODE, okay)
#error "Trigger pin node is not okay"
#endif
 
#if !DT_NODE_HAS_STATUS(ECHO_NODE1, okay)
#error "Echo pin 1 node is not okay"
#endif

#if !DT_NODE_HAS_STATUS(SERVO_NODE, okay)
#error "Servo node is not okay"
#endif
 
// Get GPIO specs from device tree
static const struct gpio_dt_spec trig = GPIO_DT_SPEC_GET(TRIGGER_NODE, gpios);
static const struct gpio_dt_spec echo1 = GPIO_DT_SPEC_GET(ECHO_NODE1, gpios);
static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(SERVO_NODE);
 
#define PWM_PERIOD_NS 20000000  // 20ms (50Hz)
#define SERVO_MIN_PULSE_NS 1000000    
#define SERVO_MAX_PULSE_NS 2600000

#define WARNING_DIST 20 // Object warning distance
 
// Thread stack size and priority
#define ULTRA_STACK_SIZE 2048
#define ULTRA_PRIORITY 5
#define SERVO_STACK_SIZE 2048
#define SERVO_PRIORITY 4
#define MAIN_STACK_SIZE 2048
#define MAIN_PRIORITY -1

static struct k_thread ultra_thread_data;
static struct k_thread servo_thread_data;
static struct k_thread main_thread_data;

K_THREAD_STACK_DEFINE(ultra_stack_area, ULTRA_STACK_SIZE);
static k_tid_t ultra_tid;

K_THREAD_STACK_DEFINE(servo_stack_area, SERVO_STACK_SIZE);
static k_tid_t servo_tid;

K_THREAD_STACK_DEFINE(main_stack_area, MAIN_STACK_SIZE);
static k_tid_t main_tid;

int angle;
int objectBool; 

// BLE Advertising params
const struct bt_le_adv_param adv_params_distance = {
     .options = BT_LE_ADV_OPT_USE_IDENTITY,
     .interval_min = 0x00A0,
     .interval_max = 0x00F0,
     .peer = NULL
};
 
LOG_MODULE_REGISTER(ultrasonic_node, LOG_LEVEL_DBG);  
 
// Broadcast distance (in centimeters)
void broadcast_distance(uint16_t objectBool, uint16_t distance_cm, uint16_t angle)
{
    uint8_t mfg_data[3];
    
    mfg_data[0] = (uint8_t)(objectBool & 0xFF);
    mfg_data[1] = (uint8_t)(distance_cm & 0xFF);
    mfg_data[2] = (uint8_t)(angle & 0xFF);
 
    struct bt_data ad[] = {
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
    };
 
    int err = bt_le_adv_start(&adv_params_distance, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)\n", err);
        return;
    }
 
    err = bt_le_adv_stop();
    if (err) {
        LOG_ERR("Advertising failed to stop (err %d)\n", err);
    }
}
 
static void initialise_bluetooth(void)
{
    int err;
    LOG_DBG("Initialisation in process");
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Initialisation failed (err %d)", err);
        return;
    }
    LOG_DBG("Initialisation successful");
}
 
// Convert angle (0–180°) to pulse width and set PWM
int servo_set_angle(uint8_t angle_deg)
{
    if (angle_deg > 180) {
        angle_deg = 180;
    }

    // Linear interpolation between min and max pulse widths
    uint32_t pulse_ns = SERVO_MIN_PULSE_NS +
        ((SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS) * angle_deg) / 180;

    return pwm_set_dt(&servo, PWM_PERIOD_NS, pulse_ns);
}

static void ultrasonic_init(void)
{
    if (!device_is_ready(trig.port) || 
        !device_is_ready(echo1.port) || 
        !device_is_ready(servo.dev)) {
        LOG_ERR("Error: One or more GPIO devices not ready\n");
        return;
    }

    gpio_pin_configure_dt(&trig, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&echo1, GPIO_INPUT);

    initialise_bluetooth();
}
 
static uint32_t ultrasonic_measure(const struct gpio_dt_spec *echo_pin)
{
    gpio_pin_set_dt(&trig, 1);
    k_busy_wait(10);
    gpio_pin_set_dt(&trig, 0);

    uint32_t timeout = k_cycle_get_32() + k_ms_to_cyc_ceil32(30);

    while (gpio_pin_get_dt(echo_pin) == 0) {
        if ((int32_t)(timeout - k_cycle_get_32()) <= 0) {
            return 0;
        }
    }

    uint32_t start = k_cycle_get_32();
    while (gpio_pin_get_dt(echo_pin) == 1) {
        if ((int32_t)(timeout - k_cycle_get_32()) <= 0) {
            return 0;
        }
    }

    uint32_t end = k_cycle_get_32();
    uint32_t cycles = end - start;
    uint64_t ns = k_cyc_to_ns_floor64(cycles);
    uint32_t time_us = ns / 1000;
    return time_us / 58;
}

void servo_thread(void *a, void *b, void *c)
{
    while (1) {
        // Sweep from 0° to 180°
        for (angle = 0; angle <= 180; angle += 15) {
            servo_set_angle(angle);
            k_msleep(500);
        }

        // Sweep from 180° back to 0°
        for (angle = 180; angle >= 0; angle -= 15) {
            servo_set_angle(angle);
            k_msleep(500);
        }

        k_msleep(100);
    }
}
 
void ultrasonic_thread(void *a, void *b, void *c)
{
    while (1) {
        uint32_t dist1 = ultrasonic_measure(&echo1);
        objectBool = 0;

        if (dist1 > 0) {
            if (dist1 <= WARNING_DIST) {
                objectBool = 1;     
                if (objectBool) {
                    broadcast_distance(objectBool, dist1, angle);
                    printk("Obstacle near: %d cm at %d degrees\n", dist1, angle);
                }
            } else {
                    printk("Obstacle not close: %d cm at %d degrees\n", dist1, angle);    
            }
        }

        k_msleep(500);
    }
}
 
void start_ultrasonic_thread(void *a, void *b, void *c) {
    ultrasonic_init();

    ultra_tid = k_thread_create(&ultra_thread_data, ultra_stack_area,
                                 K_THREAD_STACK_SIZEOF(ultra_stack_area),
                                 ultrasonic_thread, NULL, NULL, NULL,
                                 ULTRA_PRIORITY, 0, K_NO_WAIT);
 
    servo_tid = k_thread_create(&servo_thread_data, servo_stack_area,
                            K_THREAD_STACK_SIZEOF(servo_stack_area),
                            servo_thread,
                            NULL, NULL, NULL,
                            SERVO_PRIORITY, 0, K_NO_WAIT);
}
 
int main(void)
{
    main_tid = k_thread_create(&main_thread_data, main_stack_area,
                            K_THREAD_STACK_SIZEOF(main_stack_area),
                            start_ultrasonic_thread,
                            NULL, NULL, NULL,
                            MAIN_PRIORITY, 0, K_NO_WAIT);
    LOG_INF("Ultrasonic sensor ready.\n");
 
    return 0;
}
 