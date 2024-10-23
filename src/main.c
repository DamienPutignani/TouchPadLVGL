/*
 * Copyright (c) 2018 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <lvgl_input_device.h>
#include <zephyr/input/input.h> //Include input ?
//#include <debug/cpu_load.h> //cpu

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

static uint32_t count;

#ifdef CONFIG_GPIO
static struct gpio_dt_spec button_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(sw0), gpios, {0});
static struct gpio_callback button_callback;

static void button_isr_callback(const struct device *port,
				struct gpio_callback *cb,
				uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	count = 0;
}
#endif /* CONFIG_GPIO */

#ifdef CONFIG_LV_Z_ENCODER_INPUT
static const struct device *lvgl_encoder =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_lvgl_encoder_input));
#endif /* CONFIG_LV_Z_ENCODER_INPUT */

#ifdef CONFIG_LV_Z_KEYPAD_INPUT
static const struct device *lvgl_keypad =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_lvgl_keypad_input));
#endif /* CONFIG_LV_Z_KEYPAD_INPUT */

/* Récupère le périphérique LVGL Pointer */
#define LVGL_POINTER_NODE DT_PATH(lvgl_pointer)

#ifdef LVGL_POINTER_NODE
	static const struct device *lvgl_pointer_dev = DEVICE_DT_GET(LVGL_POINTER_NODE);
#endif


static void lv_btn_click_callback(lv_event_t *e)
{
	ARG_UNUSED(e);

	count = 0;
}


void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{

}

void lvgl_touch_init(void)
{
    lv_indev_drv_t indev_drv;
	
    lv_indev_drv_init(&indev_drv);          // Initialiser le driver
    indev_drv.type = LV_INDEV_TYPE_POINTER; // Le contrôleur tactile est un pointeur
    indev_drv.read_cb = touchpad_read;      // Fonction de lecture personnalisée
    lv_indev_drv_register(&indev_drv);      // Enregistrer le driver dans LVGL
}

int main(void)
{
	const struct device *display_dev;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}
	if (!device_is_ready(lvgl_pointer_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}

	lvgl_touch_init();

#ifdef CONFIG_GPIO
	if (gpio_is_ready_dt(&button_gpio)) {
		int err;

		err = gpio_pin_configure_dt(&button_gpio, GPIO_INPUT);
		if (err) {
			LOG_ERR("failed to configure button gpio: %d", err);
			return 0;
		}

		gpio_init_callback(&button_callback, button_isr_callback,
				   BIT(button_gpio.pin));

		err = gpio_add_callback(button_gpio.port, &button_callback);
		if (err) {
			LOG_ERR("failed to add button callback: %d", err);
			return 0;
		}

		err = gpio_pin_interrupt_configure_dt(&button_gpio,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (err) {
			LOG_ERR("failed to enable button callback: %d", err);
			return 0;
		}
	}
#endif /* CONFIG_GPIO */

#ifdef CONFIG_LV_Z_ENCODER_INPUT
	lv_obj_t *arc;
	lv_group_t *arc_group;

	arc = lv_arc_create(lv_scr_act());
	lv_obj_align(arc, LV_ALIGN_CENTER, 0, -15);
	lv_obj_set_size(arc, 150, 150);

	arc_group = lv_group_create();
	lv_group_add_obj(arc_group, arc);
	lv_indev_set_group(lvgl_input_get_indev(lvgl_encoder), arc_group);
#endif /* CONFIG_LV_Z_ENCODER_INPUT */

#ifdef CONFIG_LV_Z_KEYPAD_INPUT
	lv_obj_t *btn_matrix;
	lv_group_t *btn_matrix_group;
	static const char *const btnm_map[] = {"1", "2", "3", "4", ""};

	btn_matrix = lv_btnmatrix_create(lv_scr_act());
	lv_obj_align(btn_matrix, LV_ALIGN_CENTER, 0, 70);
	lv_btnmatrix_set_map(btn_matrix, (const char **)btnm_map);
	lv_obj_set_size(btn_matrix, 100, 50);

	btn_matrix_group = lv_group_create();
	lv_group_add_obj(btn_matrix_group, btn_matrix);
	lv_indev_set_group(lvgl_input_get_indev(lvgl_keypad), btn_matrix_group);
#endif /* CONFIG_LV_Z_KEYPAD_INPUT */

	
	if (IS_ENABLED(CONFIG_LV_Z_POINTER_KSCAN) || IS_ENABLED(CONFIG_LV_Z_POINTER_INPUT)) {

	}

	lv_task_handler();
	display_blanking_off(display_dev);

	ui_init();

	
	while (1) {
		lv_task_handler();
		k_sleep(K_MSEC(10));
	}
}
