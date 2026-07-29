/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/dk_prov.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/gpio.h>
#include "model_handler.h"
#include "smp_bt.h"
#include <zephyr/device.h>

static void bt_ready(int err)
{
	if (err) {
		return;
	}

	err = dk_leds_init();
	if (err) {
		return;
	}

	err = bt_mesh_init(bt_mesh_dk_prov_init(), model_handler_init());
	if (err) {
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	dk_set_leds(0x00); 
}

int main(void)
{
	int err;

	err = bt_enable(bt_ready);
	if (err) {
		// Xử lý lỗi ngầm không printk
	}

	return 0;
}