#include "recieve.h"

struct k_fifo packets_queue;

int got_mobile = 0;
int got_sensor = 0;

double to_double(int int_part, int frac_part) {
    double result = fabs(frac_part) / 100.0;
    if (int_part < 0)
        result = -result;
    return int_part + result;
}

void print_float_rec(const char *label, float f) {
    int whole = (int)f;
    int frac = (int)((f - whole) * 1000); // 3 decimal places
    if (frac < 0) frac = -frac; // Ensure positive fraction

    printk("%s%d.%03d", label, whole, frac);
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
    bt_addr_le_t mobile_addr;
    bt_addr_le_t sensor_addr;
    bt_addr_le_from_str(MOBILE_ADDR, MOBILE_ADDR_TYPE, &mobile_addr);
    bt_addr_le_from_str(SENSOR_ADDR, SENSOR_ADDR_TYPE, &sensor_addr);
    if (!bt_addr_le_eq(addr, &mobile_addr) && !bt_addr_le_eq(addr, &sensor_addr)) {
        return; // make sure device is mobile node else return
    }

	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	// printk("Device found: %s (RSSI %d), type %u, AD data len %u\n",
	//        addr_str, rssi, type, ad->len);
	
	

	struct packet_data mobile_data;
	memset(&mobile_data, 0, sizeof(mobile_data));

	if (bt_addr_le_eq(addr, &mobile_addr)) {
		got_mobile = 1;
		mobile_data.mouse_x = to_double(ad->data[5], ad->data[6]);
		mobile_data.mouse_y = to_double(ad->data[7], ad->data[8]);
		mobile_data.obstacle_detected = ad->data[2]; // 0 or 1
		mobile_data.obstacle_dist = ad->data[3]; // distance in cm
		mobile_data.obstacle_dir = ad->data[4]; // direction 0-4
		printk("Mobile Data Received:\n");
		printk("Muse X: ");
		print_float_rec("", mobile_data.mouse_x);
		printk(", Mouse Y: ");
		print_float_rec("", mobile_data.mouse_y);
		// printk(", Obstacle Detected: %d, Obstacle Distance: %f cm, Obstacle Direction: %d\n",
		// 		mobile_data.obstacle_detected,
		// 		mobile_data.obstacle_dist,
		// 		mobile_data.obstacle_dir);
	} else if (bt_addr_le_eq(addr, &sensor_addr)) {
		got_sensor = 1;
		mobile_data.displacement_x = to_double(ad->data[2], ad->data[3]);
		mobile_data.displacement_y = to_double(ad->data[4], ad->data[5]);
		// printk("Sensor Data Received:\n");
		// printk("Displacement X: ");
		// print_float_rec("", mobile_data.displacement_x);
		// printk(", Displacement Y: ");
		// print_float_rec("", mobile_data.displacement_y);
		// printk("\n");
	}

	if (!got_mobile || !got_sensor) {
		return; // wait until both mobile and sensor data is received
	}
	// both mobile and sensor data received, reset flags
	got_mobile = 0;
	got_sensor = 0;
	printk("Both mobile and sensor data received\n");

	// send data to processing thread on a queue
	struct data_item_t tx_data = {.data = mobile_data};

	size_t size = sizeof(struct data_item_t);
	char *mem_ptr = k_malloc(size); // create pointer to memory location
	__ASSERT_NO_MSG(mem_ptr != 0); // ensure mem_ptr is valid

	memcpy(mem_ptr, &tx_data, size);

	k_fifo_put(&packets_queue, mem_ptr);
}

int observer_start(void)
{
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

	k_fifo_init(&packets_queue);

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Start scanning failed (err %d)\n", err);
		return err;
	}
	printk("Started scanning...\n");

	return 0;
}

void recieve_packets(void)
{
	int err;

	printk("Starting Observer Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	(void)observer_start();

	while (1) {
		k_sleep(K_MSEC(1000));
	}

}