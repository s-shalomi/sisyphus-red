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

// logging
LOG_MODULE_REGISTER(mobile_node, LOG_LEVEL_DBG); 

#define MOUSE_ADDR "D1:00:02:1F:03:24 (random)"
// laptop mac addr: 4C-D5-77-0B-18-DA 

// bluetooth structures
static struct bt_conn_cb conn_callbacks;
static const struct bt_le_conn_param *conn_params = BT_LE_CONN_PARAM_DEFAULT;
static struct bt_conn *default_conn = NULL;
static struct bt_conn *current_conn;


static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("Service discovery complete\n");
        memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    const struct bt_gatt_service_val *service = attr->user_data;

    char uuid_str[BT_UUID_STR_LEN];
    bt_uuid_to_str(service->uuid, uuid_str, sizeof(uuid_str));
    LOG_INF("Found service UUID: %s at handle: 0x%04x\n", uuid_str, attr->handle);

    if (bt_uuid_cmp(service->uuid, BT_UUID_DECLARE_16(BT_UUID_HIDS_VAL)) == 0) {
        LOG_INF("Found HID Service!\n");
    }

    return BT_GATT_ITER_CONTINUE;
}

static struct bt_gatt_discover_params discover_params;

void discover_services(struct bt_conn *conn)
{
    memset(&discover_params, 0, sizeof(discover_params));

    discover_params.uuid = NULL;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        LOG_ERR("Service discovery failed (err %d)\n", err);
    } else {
        LOG_INF("Starting GATT discovery...\n");
    }
}

static struct bt_gatt_discover_params discover_params = {
    .uuid = NULL,
    .start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
    .end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
    .type = BT_GATT_DISCOVER_PRIMARY,
    .func = discover_func,
};


static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Failed to connect (err %u)", err);
        return;
    }
    LOG_INF("Connected to mouse");
    default_conn = bt_conn_ref(conn);

    LOG_INF("Starting GATT discovery...");

    discover_services(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_ERR("Disconnected (reason %u)", reason);
}

int check_address_pattern(char* macAddress) {
    if (macAddress[0] != MOUSE_ADDR[0] || macAddress[1] != MOUSE_ADDR[1]) return 0;
    if (macAddress[2] != MOUSE_ADDR[2]) return 0;
    if (strcmp(&macAddress[5], &MOUSE_ADDR[5]) != 0) {
        return 0;
    }
    return 1;
}

void reception(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    if ((type != BT_GAP_ADV_TYPE_ADV_IND) && (type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND)) {
        return;
    }

    if (!check_address_pattern(addr_str)) {
        return;
    }

    bt_le_scan_stop();

    struct bt_conn *mouse_conn = NULL;

    int err = bt_conn_le_create(addr,
                                BT_CONN_LE_CREATE_CONN,
                                conn_params,
                                &mouse_conn);
    if (err) {
        LOG_ERR("Connection failed (err %d)", err);
    } else {
        LOG_INF("Connection initiated");
    }

    k_sleep(K_MSEC(500));
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

int main(void)
{
    initialise_bluetooth();

    conn_callbacks.connected = connected;
    conn_callbacks.disconnected = disconnected;
    bt_conn_cb_register(&conn_callbacks);

    struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

	err = bt_le_scan_start(&scan_param, reception);
	if (err) {
		LOG_ERR("Start scanning failed (err %d)", err);
		return err;
	}
	LOG_INF("Started scanning...\n");

    return 0;
}