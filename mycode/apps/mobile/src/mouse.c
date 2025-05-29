// header file
#include "mouse.h"
#include "velocity_to_position.h"

// logging
LOG_MODULE_REGISTER(position_tracker, LOG_LEVEL_DBG); 

// semaphore
K_SEM_DEFINE(connectedSem, 0, 1);

#define MOUSE_ADDR "D1:00:02:1F:03:24 (random)"
static struct bt_uuid_16 my_hids_uuid = BT_UUID_INIT_16(0x1812);

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

KalmanFilter2D kf;
float dt = 0.05; // 50 ms

static float last_x = 0.0f, last_y = 0.0f;
static float current_x = 0.0f, current_y = 0.0f;

static float prev_dx_mm = 0.0f, prev_dy_mm = 0.0f;
static float acceleration_x = 0.0f, acceleration_y = 0.0f;

static enum discover_stage discovery_stage = DISCOVER_STAGE_SERVICE;

volatile float current_x_mm = 0.0f;
volatile float current_y_mm = 0.0f;

#define DPI 1000
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

// Parameters you can tweak
#define BASE_SENSITIVITY 1.0f
#define ACCEL_THRESHOLD 10.0f   // counts per report
#define ACCEL_FACTOR 2.0f       // how much to multiply by at high speeds

struct PositionSender positionSender = {
    .xInt = 0,
    .xFrac = 0,
    .yInt = 0,
    .yFrac = 0
};

const struct bt_le_adv_param adv_params = {
    .options = BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = 0x00A0,
    .interval_max = 0x00F0,
    .peer = NULL
};

float apply_acceleration(int8_t dx, int8_t dy, float dt_seconds) {
    float speed = sqrt(dx*dx + dy*dy) / dt_seconds;

    float scale = BASE_SENSITIVITY;

    if (speed > ACCEL_THRESHOLD) {
        scale *= ACCEL_FACTOR;
    }
    return scale;
}

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
    current_x_mm = 0.0f;
    current_y_mm = 0.0f;
    last_x = 0.0f, last_y = 0.0f;
    current_x = 0.0f, current_y = 0.0f;

    prev_dx_mm = 0.0f, prev_dy_mm = 0.0f;
    acceleration_x = 0.0f, acceleration_y = 0.0f;

    positionSender.xInt = 0;
    positionSender.xFrac = 0;
    positionSender.yInt = 0;
    positionSender.yFrac = 0;

    reset_kalman_filter(&kf);
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
	LOG_DBG("Set up button at %s pin %d\n", button.port->name, button.pin);
}

static void broadcast_position(void)
{
    uint8_t mfg_data[5];

    mfg_data[0] = 3; // MOUSE FLAG

    mfg_data[1] = positionSender.xInt;
    mfg_data[2] = positionSender.xFrac;
    mfg_data[3] = positionSender.yInt;
    mfg_data[4] = positionSender.yFrac;

    struct bt_data mobileAd[] = {
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
    };

    int err; 

    err = bt_le_adv_start(&adv_params, mobileAd, ARRAY_SIZE(mobileAd),
                    NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)\n", err);
        return;
    }

    k_msleep(100);

    err = bt_le_adv_stop();
    if (err) {
        LOG_ERR("Advertising failed to stop (err %d)\n", err);
        return;
    }
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

    // printk("Report: ");
    // for (int i = 0; i < length; ++i) {
    //     printk("%d ", ((const uint8_t *)data)[i]);
    // }
    // printk("\n");


    // TESTING REGION

    // float base_gain_x = 1.0f;
    // float base_gain_y = 1.0f;

    // float gain_x = base_gain_x * fabsf(dx);
    // float gain_y = base_gain_y * fabsf(dy);

    // float dx_scaled = gain_x * fabsf(dx);
    // if (dx < 0) dx_scaled = -dx_scaled;

    // float dy_scaled = gain_y * fabsf(dy);
    // if (dy < 0) dy_scaled = -dy_scaled;

    // float dx_mm = dx_scaled * MM_PER_COUNT;
    // float dy_mm = dy_scaled * MM_PER_COUNT;

    // END TESTING REGION


    float dx_mm = dx * MM_PER_COUNT;
    float dy_mm = dy * MM_PER_COUNT;

    last_x = current_x;
    last_y = current_y;

    float vel[2] = {dx_mm, dy_mm};
    kalman2d_predict(&kf, vel);

    current_x = kf.state_vector[0];
    current_y = kf.state_vector[1];

    float predicted_x = last_x + dx_mm;
    float predicted_y = last_y + dy_mm;

    float tolerance_x = 2.0f * fabs(dx_mm) + 0.1f; // +0.1 mm base tolerance
    float tolerance_y = 2.0f * fabs(dy_mm) + 0.1f;

    if (fabs(current_x - predicted_x) > tolerance_x ||
        fabs(current_y - predicted_y) > tolerance_y) {
        printk("SANITY CHECK FAILED, SKIPPING THIS UPDATE\n");
        current_x = last_x;
        current_y = last_y;
        return BT_GATT_ITER_CONTINUE;
    }

    positionSender.xInt = (int) dx_mm;
    positionSender.xFrac = abs((int)((dx_mm - positionSender.xInt) * 1000));

    positionSender.yInt = (int) dy_mm;
    positionSender.yFrac = abs((int)((dy_mm - positionSender.yInt) * 1000));

    // broadcast_position();

    print_state(&kf);
    // printk("dx: %d, dy: %d\n", dx, dy);
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

    // // Zephyr example for 7.5–10 ms interval (fastest allowed by BLE spec)
    // static struct bt_le_conn_param fast_params = {
    //     .interval_min = 6,  // 7.5 ms (6 * 1.25 ms)
    //     .interval_max = 8,  // 10 ms (8 * 1.25 ms)
    //     .latency = 0,
    //     .timeout = 400,
    // };

    // int kc = bt_conn_le_param_update(conn, &fast_params);
    // if (kc) {
    //     LOG_ERR("Param update failed (err %d)", kc);
    // }

    // default_conn = bt_conn_ref(conn);


    LOG_INF("Starting GATT discovery...");

    current_conn = bt_conn_ref(conn);

    discover_params.func = discover_func;
    discover_params.start_handle = 0x0001;
    discover_params.end_handle = 0xffff;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;
    
    // BT_UUID_DECLARE_16(BT_UUID_HIDS_VAL);
    // discover_params.uuid = BT_UUID_HIDS;
    discover_params.uuid = &my_hids_uuid;

    k_sleep(K_MSEC(100));

    int rc = bt_gatt_discover(conn, &discover_params);
    if (rc) {
        LOG_ERR("Discover failed (err %d)", rc);
    }

    k_sem_give(&connectedSem);
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
    kalman2d_init(&kf, dt);

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