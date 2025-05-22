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
#include <zephyr/bluetooth/hci.h>

#define ACCEL_THRESHOLD 0.2f // in m/s^2
#define VELOCITY_DECAY 0.98f

// gpio definitions
#define RED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec red = GPIO_DT_SPEC_GET(RED_NODE, gpios);
#define GREEN_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec green = GPIO_DT_SPEC_GET(GREEN_NODE, gpios);
#define BLUE_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec blue = GPIO_DT_SPEC_GET(BLUE_NODE, gpios);

// logging
LOG_MODULE_REGISTER(sensor_node, LOG_LEVEL_DBG); 

// colours
static const uint8_t BLUE[3] = {0, 0, 1};
static const uint8_t GREEN[3] = {0, 1, 0};
static const uint8_t CYAN[3] = {0, 1, 1};
static const uint8_t RED[3] = {1, 0, 0};
static const uint8_t MAGENTA[3] = {1, 0, 1};
static const uint8_t YELLOW[3] = {1, 1, 0};
static const uint8_t WHITE[3] = {1, 1, 1};

// advertising params
const struct bt_le_adv_param adv_params = {
    .options = BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = 0x00A0,
    .interval_max = 0x00F0,
    .peer = NULL
};

// motion state
typedef struct {
    float vx, vy, vz; // velocity
    float sx, sy, sz; // displacement
    float dt;         // dt
} MotionState;

void create_colour(const uint8_t* colour)
{
    gpio_pin_set_dt(&red, colour[0]);
    gpio_pin_set_dt(&green, colour[1]);
    gpio_pin_set_dt(&blue, colour[2]);
}

void initialise_indicator(void)
{
    if (!gpio_is_ready_dt(&red)) {
        return;
    }
    if (!gpio_is_ready_dt(&green)) {
        return;
    }
    if (!gpio_is_ready_dt(&blue)) {
        return;
    }

    gpio_pin_configure_dt(&red, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&green, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&blue, GPIO_OUTPUT_ACTIVE);

    create_colour(MAGENTA);
}

void broadcast(void)
{
    uint8_t mfg_data[1];

    mfg_data[0] = 1;

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

    k_msleep(1000);

    err = bt_le_adv_stop();
    if (err) {
        LOG_ERR("Advertising failed to stop (err %d)\n", err);
        return;
    }
}

float clean_target(float a) {
    return (fabsf(a) > ACCEL_THRESHOLD) ? a : 0.0f;
}

static void fetch_and_display(const struct device *sensor, 
                              float* ax,
                              float* ay,
                              float* az)
{
	struct sensor_value accel[3];
	const char *overrun = "";
	int rc = sensor_sample_fetch(sensor);

	if (rc == -EBADMSG) {
		if (IS_ENABLED(CONFIG_LIS2DH_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor,
					SENSOR_CHAN_ACCEL_XYZ, // SENSOR_CHAN_GYRO_XYZ FOR FUSION
					accel);
	}
	if (rc < 0) {
		printk("ERROR: Update failed: %d\n", rc);
	} 
    // else {
    //     printk("%u ms: %sx %d.%d , y %d.%d , z %d.%d\n",
	// 	       k_uptime_get_32(), overrun,
	// 	       accel[0].val1, abs(accel[0].val2),
	// 	       accel[1].val1, abs(accel[1].val2),
	// 	       accel[2].val1, abs(accel[2].val2));
	// }

    *ax = sensor_value_to_float(&accel[0]);
    *ay = sensor_value_to_float(&accel[1]);
    *az = sensor_value_to_float(&accel[2]);
}

void update_displacement(MotionState *state, float ax, float ay, float az) 
{
    float dt_sec = state->dt / 1000.0f;

    ax = clean_target(ax);
    ay = clean_target(ay);
    az = clean_target(az);

    // Integrate acceleration to velocity
    state->vx = clean_target((state->vx + ax * dt_sec) * VELOCITY_DECAY);
    state->vy = clean_target((state->vy + ay * dt_sec) * VELOCITY_DECAY);
    state->vz = clean_target((state->vz + az * dt_sec) * VELOCITY_DECAY);

    // Integrate velocity to displacement
    state->sx += state->vx * dt_sec;
    state->sy += state->vy * dt_sec;
    state->sz += state->vz * dt_sec;

    int sxInt = (int) (state->sx);
    int sxFrac = abs((int) ((state->sx - sxInt) * 100));
    int syInt = (int) (state->sy);
    int syFrac = abs((int) ((state->sy - syInt) * 100));
    // int szInt = (int) (state->sz);
    // int szFrac = abs((int) ((state->sz - szInt) * 100));

    printk("x: %d.%d, y: %d.%d\n",
        sxInt, sxFrac, syInt, syFrac);
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
    initialise_indicator();

	const struct device *const sensor = DEVICE_DT_GET_ANY(st_lis2dh);

	if (sensor == NULL) {
		printk("No device found\n");
		return 0;
	}
	if (!device_is_ready(sensor)) {
		printk("Device %s is not ready\n", sensor->name);
		return 0;
	}

    printk("Polling at 0.5 Hz\n");

    MotionState motion = {0, 0, 0,
                          0, 0, 0,
                          100};
    float ax;
    float ay;
    float az;

	for (;;) {
		fetch_and_display(sensor, &ax, &ay, &az);
        update_displacement(&motion, ax, ay, az);
		k_sleep(K_MSEC(motion.dt));
	}
}
