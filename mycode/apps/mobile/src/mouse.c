// header file
#include "mouse.h"

// logging
LOG_MODULE_REGISTER(position_tracker, LOG_LEVEL_DBG); 

#define MOUSE_ADDR "D1:00:02:1F:03:24 (random)"

// bluetooth structures
static struct bt_conn_cb conn_callbacks;
static const struct bt_le_conn_param *conn_params = BT_LE_CONN_PARAM_DEFAULT;
static struct bt_conn *default_conn = NULL;
static struct bt_conn *current_conn;
static struct bt_gatt_discover_params discover_params;
static uint16_t report_char_handle = 0;
static struct bt_gatt_subscribe_params subscribe_params;
static uint16_t hids_start_handle = 0, hids_end_handle = 0;

enum discover_stage {
    DISCOVER_STAGE_SERVICE,
    DISCOVER_STAGE_CHAR,
    DISCOVER_STAGE_CCC,
};

static enum discover_stage discovery_stage = DISCOVER_STAGE_SERVICE;

volatile float current_x_mm = 0.0f;
volatile float current_y_mm = 0.0f;

#define DPI 600
#define MM_PER_COUNT (25.4f / DPI)

#define SW0_NODE	DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

// mouse thread
#define MOUSE_THREAD_STACKSIZE 2048
#define MOUSE_THREAD_PRIORITY 0
K_THREAD_STACK_DEFINE(mouseThreadStack, MOUSE_THREAD_STACKSIZE); 
struct k_thread mouseThreadData;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
    current_x_mm = 0.0f;
    current_y_mm = 0.0f;
}

void initialise_button(void)
{
    int ret;

	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);
}

static uint8_t notify_func(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    if (!data) {
        LOG_ERR("Unsubscribed");
        return BT_GATT_ITER_STOP;
    }

    const uint8_t *report = (const uint8_t *)data;

    int8_t dx = (int8_t)report[1];
    int8_t dy = (int8_t)report[3];

    float dx_mm = dx * MM_PER_COUNT;
    float dy_mm = dy * MM_PER_COUNT;

    current_x_mm += dx_mm;
    current_y_mm += dy_mm;

    int currentxInt = (int) current_x_mm;
    int currentxFrac = abs((int) ((current_x_mm - currentxInt) * 1000));
    int currentyInt = (int) current_y_mm;
    int currentyFrac = abs((int) ((current_y_mm - currentyInt) * 1000));

    // LOG_DBG("Mouse: dx=%d, dy=%d", dx, dy);
    LOG_DBG("position: x: %d.%d y: %d.%d", currentxInt, currentxFrac, currentyInt, currentyFrac);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("Discovery complete");
        return BT_GATT_ITER_STOP;
    }

    if (discovery_stage == DISCOVER_STAGE_SERVICE) {
        const struct bt_gatt_service_val *service = attr->user_data;

        if (!service) {
            LOG_WRN("Warning: attr->user_data is NULL at handle 0x%04x", attr->handle);
            return BT_GATT_ITER_CONTINUE;
        }

        if (bt_uuid_cmp(service->uuid, BT_UUID_HIDS) == 0) {
            LOG_INF("Found HID Service at handle range 0x%04x to 0x%04x",
                   attr->handle, service->end_handle);

            hids_start_handle = attr->handle + 1;
            hids_end_handle = service->end_handle;

            discovery_stage = DISCOVER_STAGE_CHAR;
            params->uuid = NULL;
            params->start_handle = hids_start_handle;
            params->end_handle = hids_end_handle;
            params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

            int err = bt_gatt_discover(conn, params);
            if (err) {
                LOG_ERR("Characteristic discovery failed (err %d)", err);
                return BT_GATT_ITER_STOP;
            }

            return BT_GATT_ITER_STOP;
        }
    } 
    else if (discovery_stage == DISCOVER_STAGE_CHAR) {
        const struct bt_gatt_chrc *chrc = attr->user_data;

        if (!chrc) {
            LOG_WRN("Warning: attr->user_data is NULL during characteristic discovery at handle 0x%04x", attr->handle);
            return BT_GATT_ITER_CONTINUE;
        }

        if (bt_uuid_cmp(chrc->uuid, BT_UUID_HIDS_REPORT) == 0) {
            // Check if this characteristic supports Notify (input report)
            if (chrc->properties & BT_GATT_CHRC_NOTIFY) {
                report_char_handle = chrc->value_handle;
                LOG_DBG("Found HID Input Report Characteristic with Notify at handle: 0x%04x", report_char_handle);

                discovery_stage = DISCOVER_STAGE_CCC;
                params->uuid = BT_UUID_GATT_CCC;
                params->start_handle = attr->handle + 1;
                params->end_handle = hids_end_handle;
                params->type = BT_GATT_DISCOVER_DESCRIPTOR;

                int err = bt_gatt_discover(conn, params);
                if (err) {
                    LOG_ERR("CCC discovery failed (err %d)\n", err);
                    return BT_GATT_ITER_STOP;
                }

                return BT_GATT_ITER_STOP;
            }
        }
    } 
    else if (discovery_stage == DISCOVER_STAGE_CCC) {
        if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC) == 0) {
            subscribe_params.notify = notify_func;
            subscribe_params.value_handle = report_char_handle;
            subscribe_params.ccc_handle = attr->handle;
            subscribe_params.value = BT_GATT_CCC_NOTIFY;

            int err = bt_gatt_subscribe(conn, &subscribe_params);
            if (err) {
                LOG_ERR("Subscribe failed (err %d)", err);
            } else {
                LOG_INF("Subscribed to HID input report notifications\n");
            }

            return BT_GATT_ITER_STOP;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}


static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Failed to connect (err %u)", err);
        return;
    }
    LOG_INF("Connected to mouse");
    default_conn = bt_conn_ref(conn);


    LOG_INF("Starting GATT discovery...");

    current_conn = bt_conn_ref(conn);

    discover_params.uuid = BT_UUID_HIDS;
    discover_params.func = discover_func;
    discover_params.start_handle = 0x0001;
    discover_params.end_handle = 0xffff;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int rc = bt_gatt_discover(conn, &discover_params);
    if (rc) {
        LOG_ERR("Discover failed (err %d)", rc);
    }
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
}

void mouse_thread(void* arg1, void* arg2, void* arg3)
{
    // initialise_bluetooth();
    initialise_button();

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
		return;
	}
	LOG_INF("Started scanning...\n");

    return;
}

void create_mouse_thread(void) 
{
    // mouse thread
    k_thread_create(
        &mouseThreadData,                            // pointer to thread data structure
        mouseThreadStack,                            // stack area allocated for thread 
        K_THREAD_STACK_SIZEOF(mouseThreadStack),     // total size of stack
        mouse_thread,                                         // thread function 
        NULL,                                           // arg 1
        NULL,                                           // arg 2
        NULL,                                           // arg 3
        MOUSE_THREAD_PRIORITY,                       // thread priority          
        0,                                              // thread options
        K_NO_WAIT                                       // start without waiting
    ); 
}