/*
 * Viewer (M5stack Core2) driver main source file
 * This file will run the M5stack Core2 for task 6 of prac 3
 * 
 * Functionality:
 * - Display grid
 * - Display the position of the mobile node
 * - Allow changing the grid size dynamically via shell commands
 *  
*/
// Zephyr files
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

// Bluetooth
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

// LVGL and Standard libraries
#include <lvgl.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

LOG_MODULE_REGISTER(viewer, CONFIG_LOG_DEFAULT_LEVEL);

// Default grid size
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240
static int ROWS = 6;
static int COLS = 8;
static lv_obj_t *marker;  // Circle marker to show position
static int current_row = 0;
static int current_col = 0;

// Node object for lv
static lv_obj_t *node;
static lv_obj_t *label;

// Base node address
#define BASE_NODE_ADDR "F2:5B:37:31:EC:20 (random)"

static void draw_node(lv_obj_t *node, float row, float col);
static void draw_label(lv_obj_t *label, float row, float col, char* string);

// Display drawing functions
static void draw_grid(void) {
    // Dynamically allocate memory for points arrays
    lv_point_precise_t *v_points = k_malloc(sizeof(lv_point_precise_t) * (COLS - 1) * 2);
    lv_point_precise_t *h_points = k_malloc(sizeof(lv_point_precise_t) * (ROWS - 1) * 2);

    if (!v_points || !h_points) {
        LOG_ERR("Memory allocation failed for points arrays");
        return;
    }

    draw_node(node, 0, 0);
    draw_node(node, 0, 1.5);
    draw_node(node, 0, 3);
    draw_node(node, 0, 4.5);
    draw_node(node, 0, 6);
    draw_node(node, 2, 0);
    draw_node(node, 2, 3);
    draw_node(node, 2, 6);
    draw_node(node, 4, 0);
    draw_node(node, 4, 1.5);
    draw_node(node, 4, 3);
    draw_node(node, 4, 4.5);
    draw_node(node, 4, 6);
    draw_label(label, 1.5, 1.5, "A");
    draw_label(label, 1.5, 4.5, "B");

    // Vertical lines
    for (int i = 1; i < COLS; i++) {
        int x = i * (SCREEN_WIDTH / COLS);

        v_points[(i - 1) * 2].x = x;
        v_points[(i - 1) * 2].y = 0;
        v_points[(i - 1) * 2 + 1].x = x;
        v_points[(i - 1) * 2 + 1].y = SCREEN_HEIGHT;

        lv_obj_t *line = lv_line_create(lv_scr_act());
        lv_line_set_points(line, &v_points[(i - 1) * 2], 2);

        lv_obj_set_style_line_color(line, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_line_width(line, 2, LV_PART_MAIN);
        lv_obj_align(line, LV_ALIGN_TOP_LEFT, 0, 0);
    }

    // Horizontal lines
    for (int i = 1; i < ROWS; i++) {
        int y = i * (SCREEN_HEIGHT / ROWS);

        h_points[(i - 1) * 2].x = 0;
        h_points[(i - 1) * 2].y = y;
        h_points[(i - 1) * 2 + 1].x = SCREEN_WIDTH;
        h_points[(i - 1) * 2 + 1].y = y;

        lv_obj_t *line = lv_line_create(lv_scr_act());
        lv_line_set_points(line, &h_points[(i - 1) * 2], 2);

        lv_obj_set_style_line_color(line, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_line_width(line, 2, LV_PART_MAIN);
        lv_obj_align(line, LV_ALIGN_TOP_LEFT, 0, 0);
    }

    // Free the allocated memory after usage
    k_free(v_points);
    k_free(h_points);
}

static void draw_node(lv_obj_t *node, float row, float col) {
    int cell_width = SCREEN_WIDTH / COLS;
    int cell_height = SCREEN_HEIGHT / ROWS;

    int x = (int)(col * cell_width + cell_width);
    int y = (int)(row * cell_height + cell_height);

    // Create node (circle)
    node = lv_obj_create(lv_scr_act());
    lv_obj_set_size(node, 20, 20);
    lv_obj_set_style_radius(node, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(node, lv_color_hex(0x000000), LV_PART_MAIN);  // black circle
    lv_obj_set_pos(node, x - 10, y - 10);  // center marker
}

static void draw_label(lv_obj_t *label, float row, float col, char* string) {
    int cell_width = SCREEN_WIDTH / COLS;
    int cell_height = SCREEN_HEIGHT / ROWS;

    int x = (int)(col * cell_width + cell_width);
    int y = (int)(row * cell_height + cell_height);

    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, string);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    lv_obj_set_pos(label, x - 10, y - 10);

}

static void draw_marker_float(float row, float col) {
    int cell_width = SCREEN_WIDTH / COLS;
    int cell_height = SCREEN_HEIGHT / ROWS;

    int x = (int)(col * cell_width + cell_width);
    int y = (int)(row * cell_height + cell_height);

    if (marker == NULL) {
        // Create marker (circle)
        marker = lv_obj_create(lv_scr_act());
        lv_obj_set_size(marker, 20, 20);
        lv_obj_set_style_radius(marker, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(marker, lv_color_hex(0xff0000), LV_PART_MAIN);  // red
    }

    // Update marker position
    lv_obj_set_pos(marker, x - 10, y - 10);  // center marker
}

static void draw_marker(int row, int col) {
    int cell_width = SCREEN_WIDTH / COLS;
    int cell_height = SCREEN_HEIGHT / ROWS;

    int x = col * cell_width + cell_width;
    int y = row * cell_height + cell_height;

    if (marker == NULL) {
        // Create marker (circle)
        marker = lv_obj_create(lv_scr_act());
        lv_obj_set_size(marker, 20, 20);
        lv_obj_set_style_radius(marker, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(marker, lv_color_hex(0xff0000), LV_PART_MAIN);  // red
    }

    // Update marker position
    lv_obj_set_pos(marker, x - 10, y - 10);  // center marker
}

// Bluetooth functionality
void reception(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    if (!strcmp(addr_str, BASE_NODE_ADDR)) {
        // Expecting exactly 4 bytes: {x_int, x_frac, y_int, y_frac}
        if ((ad->len != 6) || (type != 3)) {
            // Print out the invalid packet data
            LOG_ERR("Invalid packet received from %s: length %d, type %d, data:", addr_str, ad->len, type);
            for (int i = 0; i < ad->len; i++) {
                LOG_ERR(" 0x%02X", ad->data[i]);
            }
            return; // invalid packet
        }

        // Extract integers
        uint8_t y_int = ad->data[2];
        uint8_t y_frac = ad->data[3];
        uint8_t x_int = ad->data[4];
        uint8_t x_frac = ad->data[5];

        // Convert to floats (same logic as cmd_move)
        float c = x_int + (x_frac / 100.0f);
        float r = y_int + (y_frac / 100.0f);

        // Sanity check bounds (just like cmd_move)
        if (r < 0.0f || r >= ROWS-2 || c < 0.0f || c >= COLS-2) {
            printk("Invalid move coords: row %.2f col %.2f\n", (double)r, (double)c);
            if (r >= ROWS-2) {
                r = 4.00;
            } 
            
            if (c >= COLS-2) {
                c = 6.00;
            }
        }

        // Save current position as integers (like cmd_move)
        current_row = (int)r;
        current_col = (int)c;

        // Draw with float precision
        draw_marker_float(r, c);

        printk("Moved marker to: row %.2f col %.2f\n", (double)r, (double)c);
    }
    return;
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

int initialise_reception(void)
{
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
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

static int cmd_move(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 3) {
        shell_print(shell, "Usage: move <row> <col>");
        return -EINVAL;
    }

    float r = atof(argv[1]);
    float c = atof(argv[2]);

    if (r < 0.0f || r >= ROWS || c < 0.0f || c >= COLS) {
        shell_print(shell, "Invalid row/col (rows 0-%d, cols 0-%d)", ROWS - 1, COLS - 1);
        return -EINVAL;
    }

    current_row = (int)r;
    current_col = (int)c;

    draw_marker_float(r, c);  // NEW: draw with float precision

    // Avoid Wdouble-promotion warning by casting explicitly
    shell_print(shell, "Moved to row %.2f col %.2f", (double)r, (double)c);
    return 0;
}

SHELL_CMD_REGISTER(move, NULL, "Move marker: move <row> <col>", cmd_move);

static int cmd_resize_grid(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 3) {
        shell_print(shell, "Usage: resize_grid <rows> <cols>");
        return -EINVAL;
    }

    int new_rows = atoi(argv[1]);
    int new_cols = atoi(argv[2]);

    if (new_rows <= 0 || new_cols <= 0 || new_rows > 10 || new_cols > 10) {
        shell_print(shell, "Invalid grid size (1-10 rows and columns allowed)");
        return -EINVAL;
    }

    ROWS = new_rows;
    COLS = new_cols;

    // Clear the screen before drawing the grid
    lv_obj_clean(lv_scr_act());
    // Re-draw the grid with the new size
    draw_grid();
    // Redraw the marker in the new grid position
    draw_marker(0, 0);

    shell_print(shell, "Grid size changed to %d rows x %d columns", ROWS, COLS);
    return 0;
}

SHELL_CMD_REGISTER(resize_grid, NULL, "Resize grid: resize_grid <rows> <cols>", cmd_resize_grid);

int main(void) {
    const struct device *display_dev;

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return -ENODEV;
    }

    // Initialize bluetooth
    initialise_bluetooth();
    initialise_reception();

    lv_init();  // Initialize LVGL system
    lv_obj_clean(lv_scr_act());  // Clear screen
    draw_grid();
    draw_marker(current_row, current_col);

    lv_timer_handler();  // Make sure the LVGL event handler is called
    display_blanking_off(display_dev);

    while (1) {
        uint32_t sleep_ms = lv_timer_handler();
        k_msleep(MIN(sleep_ms, INT32_MAX));  // Ensure proper sleep time
    }

    return 0;
}