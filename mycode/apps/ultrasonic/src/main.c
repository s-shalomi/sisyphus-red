/*
 * Ultrasonic ranger driver main source file
 * This file will run the ultrasonic sensor for task 3 of prac 3
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
 #include <zephyr/sys/printk.h>
 #include <zephyr/shell/shell.h>
 #include <zephyr/sys/ring_buffer.h>
 #include <zephyr/logging/log.h>
 #include <zephyr/bluetooth/bluetooth.h>
 #include <zephyr/bluetooth/hci.h>
 
 // Get the node identifiers using aliases
 #define TRIGGER_NODE DT_ALIAS(trigger)
 #define ECHO_NODE    DT_ALIAS(echo)
 
 #if !DT_NODE_HAS_STATUS(TRIGGER_NODE, okay)
 #error "Trigger pin node is not okay"
 #endif
 
 #if !DT_NODE_HAS_STATUS(ECHO_NODE, okay)
 #error "Echo pin node is not okay"
 #endif
 
 // Get GPIO specs from device tree
 static const struct gpio_dt_spec trig = GPIO_DT_SPEC_GET(TRIGGER_NODE, gpios);
 static const struct gpio_dt_spec echo = GPIO_DT_SPEC_GET(ECHO_NODE, gpios);
 
 // Ring buffer size (small, because we store only 1 value)
 #define RING_BUF_SIZE 8  // enough for 1 uint32_t
 RING_BUF_DECLARE(ultra_ring_buf, RING_BUF_SIZE);
 
 // Thread stack size and priority
 #define ULTRA_STACK_SIZE 1024
 #define ULTRA_PRIORITY 5
 
 static struct k_thread ultra_thread_data;
 static K_THREAD_STACK_DEFINE(ultra_stack_area, ULTRA_STACK_SIZE);
 static k_tid_t ultra_tid;
 static atomic_t measure_active = ATOMIC_INIT(0);
 
 // BLE Advertising params
 const struct bt_le_adv_param adv_params_distance = {
     .options = BT_LE_ADV_OPT_USE_IDENTITY,
     .interval_min = 0x00A0,
     .interval_max = 0x00F0,
     .peer = NULL
 };
 
 LOG_MODULE_REGISTER(ultrasonic_node, LOG_LEVEL_DBG);  
 
 // Broadcast distance (in centimeters)
 void broadcast_distance(uint16_t distance_cm)
 {
     uint8_t mfg_data[1];
 
     mfg_data[0] = (uint8_t)(distance_cm & 0xFF);
 
     struct bt_data ad[] = {
         BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
     };
 
     int err = bt_le_adv_start(&adv_params_distance, ad, ARRAY_SIZE(ad), NULL, 0);
     if (err) {
         LOG_ERR("Advertising failed to start (err %d)\n", err);
         return;
     }
 
     printk("Broadcasting distance: %d cm\n", distance_cm);
 
     k_msleep(1000);  // Broadcast for 1000 ms
 
     err = bt_le_adv_stop();
     if (err) {
         LOG_ERR("Advertising failed to stop (err %d)\n", err);
     }
 }
 
 void initialise_bluetooth(void)
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
 
 static void ultrasonic_init(void)
 {
     if (!device_is_ready(trig.port) || !device_is_ready(echo.port)) {
         LOG_ERR("Error: GPIO device not ready\n");
         return;
     }
 
     gpio_pin_configure_dt(&trig, GPIO_OUTPUT_INACTIVE);
     gpio_pin_configure_dt(&echo, GPIO_INPUT);
 
     initialise_bluetooth();
 }
 
 // Measure distance (returns distance in cm or 0 on error)
 static uint32_t ultrasonic_measure(void)
 {
     gpio_pin_set_dt(&trig, 1);
     k_busy_wait(10);  // 10us pulse
     gpio_pin_set_dt(&trig, 0);
 
     uint32_t timeout = k_cycle_get_32() + k_ms_to_cyc_ceil32(30);
     while (gpio_pin_get_dt(&echo) == 0) {
         if ((int32_t)(timeout - k_cycle_get_32()) <= 0) {
             return 0;  // Timeout waiting for high
         }
     }
 
     uint32_t start = k_cycle_get_32();
     while (gpio_pin_get_dt(&echo) == 1) {
         if ((int32_t)(timeout - k_cycle_get_32()) <= 0) {
             return 0;  // Timeout waiting for low
         }
     }
     uint32_t end = k_cycle_get_32();
 
     uint32_t cycles = end - start;
     uint64_t ns = k_cyc_to_ns_floor64(cycles);
     uint32_t time_us = ns / 1000;
     uint32_t distance_cm = time_us / 58;
 
     return distance_cm;
 }
 
 // Continuous measurement thread
 void ultrasonic_thread(void *a, void *b, void *c)
 {
     while (1) {
         // if (atomic_get(&measure_active)) {
             uint32_t distance = ultrasonic_measure();
             if (distance > 0) {
                 // Clear buffer first to ensure only 1 value
                 ring_buf_reset(&ultra_ring_buf);
 
                 // Store the latest distance
                 ring_buf_put(&ultra_ring_buf, (uint8_t *)&distance, sizeof(distance));
 
                 broadcast_distance(distance);
             } else {
                 LOG_ERR("Measurement failed\n");
             }
 
             k_sleep(K_SECONDS(1));  // Wait 1 second before next measure
         // } 
         // else {
         //     k_sleep(K_MSEC(100));
         // }
     }
 }
 
 // Shell command to start measuring
 static int cmd_ultrasonic_start(const struct shell *shell, size_t argc, char **argv)
 {
     atomic_set(&measure_active, 1);
     printk("Started continuous measurement\n");
     return 0;
 }
 
 // Shell command to stop measuring
 static int cmd_ultrasonic_stop(const struct shell *shell, size_t argc, char **argv)
 {
     atomic_set(&measure_active, 0);
     printk("Stopped measurement\n");
     return 0;
 }
 
 // Shell command to print the last measurement
 static int cmd_ultrasonic_print(const struct shell *shell, size_t argc, char **argv)
 {
     uint8_t temp_buf[sizeof(uint32_t)];
     uint32_t distance;
 
     if (ring_buf_get(&ultra_ring_buf, temp_buf, sizeof(uint32_t)) == sizeof(uint32_t)) {
         memcpy(&distance, temp_buf, sizeof(uint32_t));
         printk("Last stored distance: %d cm\n", distance);
 
         // Put back into buffer to keep it stored
         ring_buf_put(&ultra_ring_buf, temp_buf, sizeof(uint32_t));
     } else {
         printk("No distance data available\n");
     }
 
     return 0;
 }
 
 // Register commands under "ultra"
 SHELL_STATIC_SUBCMD_SET_CREATE(sub_ultra,
     SHELL_CMD(start, NULL, "Start continuous measurement", cmd_ultrasonic_start),
     SHELL_CMD(stop, NULL, "Stop measurement", cmd_ultrasonic_stop),
     SHELL_CMD(print, NULL, "Print last stored distance", cmd_ultrasonic_print),
     SHELL_SUBCMD_SET_END
 );
 
 SHELL_CMD_REGISTER(ultra, &sub_ultra, "Ultrasonic sensor commands", NULL);
 
 int main(void)
 {
     ultrasonic_init();
 
     ultra_tid = k_thread_create(&ultra_thread_data, ultra_stack_area,
                                 K_THREAD_STACK_SIZEOF(ultra_stack_area),
                                 ultrasonic_thread, NULL, NULL, NULL,
                                 ULTRA_PRIORITY, 0, K_NO_WAIT);
 
     LOG_INF("Ultrasonic sensor ready. Type 'ultra start' to begin measuring, 'ultra stop' to stop, and 'ultra print' to show last distance.\n");
 
     return 0;
 }
 