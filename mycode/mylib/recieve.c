#include "recieve.h"

struct k_fifo packets_queue;

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
    bt_addr_le_t mobile_addr;
    bt_addr_le_from_str(MOBILE_ADDR, MOBILE_ADDR_TYPE, &mobile_addr);
    if (!bt_addr_le_eq(addr, &mobile_addr)) {
        return; // make sure device is mobile node else return
    }

	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d), type %u, AD data len %u\n",
	       addr_str, rssi, type, ad->len);

	// get data from advertisement and put in struct // TO DO
	struct packet_data mobile_data;
	memset(&mobile_data, 0, sizeof(mobile_data));

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
		return 0;
	}

	(void)observer_start();

}