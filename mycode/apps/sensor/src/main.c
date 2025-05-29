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

#include "integral.h"

#define ACCEL_THRESHOLD 0.5f // in m/s^2
#define VELOCITY_DECAY 0.9f
#define SAMPLE_PERIOD 50 // in ms

#define STATE_SIZE 4     // [x, y, vx, vy]
#define CONTROL_SIZE 2   // [ax, ay]

#define M_PI 3.1415f

#define GYRO_TO_RAD (M_PI / 180.0f) // If gyro is in deg/s

// gpio definitions
#define RED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec red = GPIO_DT_SPEC_GET(RED_NODE, gpios);
#define GREEN_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec green = GPIO_DT_SPEC_GET(GREEN_NODE, gpios);
#define BLUE_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec blue = GPIO_DT_SPEC_GET(BLUE_NODE, gpios);

// button
static const struct gpio_dt_spec pushButton = GPIO_DT_SPEC_GET(DT_ALIAS(pushbutton), gpios);
static struct gpio_callback callbackData;

// logging
LOG_MODULE_REGISTER(sensor_node, LOG_LEVEL_DBG); 

static float heading = 0.0f;

KalmanFilter kf;

// colours
static const uint8_t BLUE[3] = {0, 0, 1};
static const uint8_t GREEN[3] = {0, 1, 0};
static const uint8_t CYAN[3] = {0, 1, 1};
static const uint8_t RED[3] = {1, 0, 0};
static const uint8_t MAGENTA[3] = {1, 0, 1};
static const uint8_t YELLOW[3] = {1, 1, 0};
static const uint8_t WHITE[3] = {1, 1, 1};

// // x, y position
// volatile int sxInt;
// volatile int sxFrac;
// volatile int syInt;
// volatile int syFrac;

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

// flags
volatile int resetPressed = 1;

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

void broadcast(MotionState* state)
{
    int8_t sxInt = (int8_t) (state->sx);
    int8_t sxFrac = abs((int8_t) ((state->sx - sxInt) * 100));
    int8_t syInt = (int8_t) (state->sy);
    int8_t syFrac = abs((int8_t) ((state->sy - syInt) * 100));
    // int szInt = (int) (state->sz);
    // int szFrac = abs((int) ((state->sz - szInt) * 100));

    // printk("x: %d.%d, y: %d.%d\n",
    //     sxInt, sxFrac, syInt, syFrac);

    uint8_t mfg_data[4];

    mfg_data[0] = sxInt;
    mfg_data[1] = sxFrac;
    mfg_data[2] = syInt;
    mfg_data[3] = syFrac;

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

float clean_target(float a) {

    if (fabsf(a) > ACCEL_THRESHOLD) {
        return a;
    } else {
        // kf.state_vector[2] = 0;
        // kf.state_vector[3] = 0;
        return 0.0f;
    }
}

void zupt_if_stationary(KalmanFilter *kf, float ax_global, float ay_global) {
    static int stationary_count = 0;
    if (fabsf(ax_global) < ACCEL_THRESHOLD && fabsf(ay_global) < ACCEL_THRESHOLD) {
        stationary_count++;
        if (stationary_count > 5) { // e.g. stationary for 5 consecutive samples
            kf->state_vector[2] = 0; // vx
            kf->state_vector[3] = 0; // vy
        }
    } else {
        stationary_count = 0;
    }
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

    *ax = sensor_value_to_float(&accel[0]);
    *ay = sensor_value_to_float(&accel[1]);
    *az = sensor_value_to_float(&accel[2]);
}

void fetch_accel_gyro(const struct device *sensor, float *ax, float *ay, float *az, float *gyro_z) {
    struct sensor_value accel[3], gyro[3];

    sensor_sample_fetch(sensor);
    sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel);
    *ax = sensor_value_to_float(&accel[0]);
    *ay = sensor_value_to_float(&accel[1]);
    *az = sensor_value_to_float(&accel[2]);

    sensor_channel_get(sensor, SENSOR_CHAN_GYRO_XYZ, gyro);
    *gyro_z = sensor_value_to_float(&gyro[2]); // deg/s
}

void update_heading(float gyro_z, float dt_sec) {
    float gyro_z_rad = gyro_z * GYRO_TO_RAD;
    heading += gyro_z_rad * dt_sec;
    // Wrap heading
    if (heading > M_PI) heading -= 2 * M_PI;
    if (heading < -M_PI) heading += 2 * M_PI;
}

void rotate_accel_to_global(float ax, float ay, float *ax_global, float *ay_global) {
    *ax_global = ax * cosf(heading) - ay * sinf(heading);
    *ay_global = ax * sinf(heading) + ay * cosf(heading);
}

void kalman_predict(KalmanFilter *kf, float *accel) {
    float temp[STATE_SIZE] = {0};

    // x' = A*x + B*u
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j)
            temp[i] += kf->transition[i][j] * kf->state_vector[j];
        for (int j = 0; j < CONTROL_SIZE; ++j)
            temp[i] += kf->control_matrix[i][j] * accel[j];
    }
    memcpy(kf->state_vector, temp, sizeof(float) * STATE_SIZE);
}

// void zupt_if_stationary(KalmanFilter *kf, float ax_global, float ay_global) {
//     if (fabsf(ax_global) < ACCEL_THRESHOLD)
//         kf->state_vector[2] = 0; // vx
//     if (fabsf(ay_global) < ACCEL_THRESHOLD)
//         kf->state_vector[3] = 0; // vy
// }

void update_displacement2(MotionState* state, float ax, float ay) 
{
    ax = clean_target(ax);
    ay = clean_target(ay);

    // zupt_if_stationary(&kf, ax, ay);

    float measurement[2];
    float acceleration[2] = {ax, ay};

    measurement[0] = kf.state_vector[0];  // previous estimate
    measurement[1] = kf.state_vector[1];

    kalman_update(&kf, measurement, acceleration);

    // **Now copy the UPDATED filter state**
    state->sx = kf.state_vector[0];
    state->sy = kf.state_vector[1];

    print_float("", state->sx);
    printk(", ");
    print_float("", state->sy);
    printk("\n");
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

    bt_addr_le_t addr;
    int id = 0;

    bt_id_get(&addr, &id);

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
    LOG_INF("Local Bluetooth address: %s\n", addr_str);
}

void reset_accelerometer(void)
{
    resetPressed = 1;
}

void initialise_thingy_button(void) {

    if (!gpio_is_ready_dt(&pushButton)) {
        return;
    }

    gpio_pin_configure_dt(&pushButton, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&pushButton, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&callbackData, reset_accelerometer, BIT(pushButton.pin));
    gpio_add_callback(pushButton.port, &callbackData);
}

int main(void)
{
    initialise_bluetooth();
    initialise_indicator();
    initialise_thingy_button();

	const struct device *const sensor = DEVICE_DT_GET_ANY(st_lis2dh);

	if (sensor == NULL) {
		printk("No device found\n");
		return 0;
	}
	if (!device_is_ready(sensor)) {
		printk("Device %s is not ready\n", sensor->name);
		return 0;
	}

    MotionState motion = {0, 0, 0,
                        0, 0, 0,
                        SAMPLE_PERIOD};

    float dt = motion.dt / 1000;
    kalman_init(&kf, dt);

    // float ax;
    // float ay;
    // float az;

	for (;;) {
        if (resetPressed) {
            motion.vx = 0;
            motion.vy = 0;
            motion.sx = 0;
            motion.sy = 0;
            motion.dt = SAMPLE_PERIOD;
            resetPressed = 0;

            // Also reset Kalman filter state!
            for (int i = 0; i < STATE_SIZE; i++) {
                kf.state_vector[i] = 0.0f;
                for (int j = 0; j < STATE_SIZE; j++) {
                    kf.error_covariance[i][j] = (i == j) ? 1.0f : 0.0f;
                }
            }
        }

        float ax, ay, az, gyro_z;
        fetch_accel_gyro(sensor, &ax, &ay, &az, &gyro_z);
        update_heading(gyro_z, dt);

        float ax_global, ay_global;
        rotate_accel_to_global(ax, ay, &ax_global, &ay_global);

        float accel[2] = {-ax_global,- ay_global};
        kalman_predict(&kf, accel);

        zupt_if_stationary(&kf, ax_global, ay_global);

        float x = kf.state_vector[0];
        float y = kf.state_vector[1];

        print_float("", x);
        printk(", ");
        print_float("", y);
        printk(", ");
        // print_float("", heading);
        printk("\n");

		// fetch_and_display(sensor, &ax, &ay, &az);
        // update_displacement2(&motion, ax, ay);
        broadcast(&motion);
		k_sleep(K_MSEC(motion.dt));
	}
}
