/*
 * ESP32-S3 Bluetooth Mesh Gateway Application
 * Fully equivalent to gateway_node in logic, features, and RTT/Console logging format.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_health_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "esp_mac.h"

#include "board.h"

#define TAG "model_handler"

#define SENSOR_GROUP_ADDR       0xC000
#define SPECIAL_SENSOR_OP       0x8299
#define MAX_TRACKED_SOURCES     32

#define CID_ESP                 0x02E5
#define PROV_OWN_ADDR           0x000C

struct source_measurement {
	uint16_t addr;
	uint32_t first_seen_ms;
	uint32_t last_seen_ms;
	uint32_t last_report_ms;
	uint8_t data;
	uint8_t battery;
	bool data_valid;
	bool battery_valid;
};

static struct source_measurement measurements[MAX_TRACKED_SOURCES];
static uint32_t sensor_callback_count = 0;
static uint32_t complete_packet_count = 0;

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };

static void ble_mesh_get_dev_uuid(uint8_t *uuid)
{
	if (!uuid) return;
	uuid[0] = 0x32;
	uuid[1] = 0x10;
	esp_read_mac(uuid + 2, ESP_MAC_BT);
}

/* Model declarations */
static esp_ble_mesh_cfg_srv_t config_server = {
	.net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
	.relay = ESP_BLE_MESH_RELAY_DISABLED,
	.relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
	.beacon = ESP_BLE_MESH_BEACON_ENABLED,
	.default_ttl = 7,
};

static uint8_t health_tests[] = {
	0x00,
};

static esp_ble_mesh_health_srv_t health_server = {
	.health_test = {
		.id_count = ARRAY_SIZE(health_tests),
		.test_ids = health_tests,
	},
};
ESP_BLE_MESH_MODEL_PUB_DEFINE(health_pub, 0, ROLE_NODE);

static esp_ble_mesh_client_t sensor_client;

static esp_ble_mesh_gen_onoff_srv_t onoff_server = {
	.rsp_ctrl = {
		.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
		.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
	},
};
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2, ROLE_NODE);

static esp_ble_mesh_model_t elem0_models[] = {
	ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
	ESP_BLE_MESH_MODEL_HEALTH_SRV(&health_server, &health_pub),
	ESP_BLE_MESH_MODEL_SENSOR_CLI(NULL, &sensor_client),
};

static esp_ble_mesh_model_t elem1_models[] = {
	ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub, &onoff_server),
};

static esp_ble_mesh_elem_t elements[] = {
	ESP_BLE_MESH_ELEMENT(0, elem0_models, ESP_BLE_MESH_MODEL_NONE),
	ESP_BLE_MESH_ELEMENT(1, elem1_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
	.cid = CID_ESP,
	.element_count = ARRAY_SIZE(elements),
	.elements = elements,
};

static esp_ble_mesh_prov_t provision = {
	.uuid = dev_uuid,
};

static esp_timer_handle_t status_timer;
static esp_timer_handle_t rx_led_timer;

static void rx_led_off_timer_cb(void *arg)
{
	board_led_operation(LED_1, LED_OFF);
}

static void indicate_sensor_rx(void)
{
	board_led_operation(LED_1, LED_ON);
	if (rx_led_timer) {
		esp_timer_stop(rx_led_timer);
		esp_timer_start_once(rx_led_timer, 1000000); /* 1 second in microseconds */
	}
}

static struct source_measurement *measurement_for(uint16_t addr)
{
	struct source_measurement *free_slot = NULL;

	for (int i = 0; i < ARRAY_SIZE(measurements); ++i) {
		if (measurements[i].addr == addr) {
			return &measurements[i];
		}
		if (!measurements[i].addr && !free_slot) {
			free_slot = &measurements[i];
		}
	}

	if (free_slot) {
		free_slot->addr = addr;
		free_slot->first_seen_ms = (uint32_t)(esp_timer_get_time() / 1000);
		return free_slot;
	}

	struct source_measurement *oldest = &measurements[0];
	for (int i = 1; i < ARRAY_SIZE(measurements); ++i) {
		if (measurements[i].last_seen_ms < oldest->last_seen_ms) {
			oldest = &measurements[i];
		}
	}
	*oldest = (struct source_measurement){
		.addr = addr,
		.first_seen_ms = (uint32_t)(esp_timer_get_time() / 1000),
	};
	return oldest;
}

static void print_measurement(struct source_measurement *m, uint16_t dst_addr, uint8_t recv_ttl, int8_t rssi)
{
	uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
	uint32_t delta_ms = (m->last_report_ms > 0) ? (now - m->last_report_ms) : 0;

	m->last_report_ms = now;
	complete_packet_count++;

	ESP_LOGI(TAG, "=========================================================================");
	ESP_LOGI(TAG, "GW_PACKET RECEIVED! count=%" PRIu32 " t_ms=%" PRIu32 " src=0x%04x data=%u battery=%u "
		 "delta_ms=%" PRIu32 " ttl=%u dst=0x%04x rssi=%d%s",
		 complete_packet_count, now, m->addr, m->data, m->battery,
		 delta_ms, recv_ttl, dst_addr, rssi,
		 (recv_ttl == 1 ? " [TTL=1 Direct Rx]" : ""));
	ESP_LOGI(TAG, "=========================================================================");

	m->data_valid = false;
	m->battery_valid = false;
	m->first_seen_ms = now;
}

int send_special_sensor_message(esp_ble_mesh_model_t *model, const esp_ble_mesh_msg_ctx_t *in_ctx, uint8_t data_val)
{
	esp_ble_mesh_model_t *target_model = model ? model : sensor_client.model;
	if (!target_model) {
		ESP_LOGW(TAG, "GW_TX_SPECIAL: target model not initialized");
		return -1;
	}

	uint16_t net_idx = in_ctx ? in_ctx->net_idx : 0;
	uint16_t app_idx = in_ctx ? in_ctx->app_idx : 0;

	/* Auto-sync AppKey 0 to sensor_client model so all models share valid AppKey */
	if (sensor_client.model && in_ctx) {
		sensor_client.model->keys[0] = app_idx;
	}

	esp_ble_mesh_msg_ctx_t ctx = {
		.net_idx = net_idx,
		.app_idx = app_idx,
		.addr = ESP_BLE_MESH_ADDR_ALL_NODES,   /* 0xFFFF (ESP_BLE_MESH_ADDR_ALL_NODES) */
		.send_ttl = 7,                          /* TTL = 7: Broadcast with TTL=7 as requested */
		.send_rel = false,
	};

	esp_err_t err = esp_ble_mesh_server_model_send_msg(target_model, &ctx, SPECIAL_SENSOR_OP, 1, &data_val);
	if (err != ESP_OK) {
		err = esp_ble_mesh_client_model_send_msg(target_model, &ctx, SPECIAL_SENSOR_OP, 1, &data_val, 0, false, ROLE_NODE);
	}

	if (err != ESP_OK) {
		ESP_LOGE(TAG, "GW_TX_SPECIAL_FAIL dst=0x%04x err=%d", ctx.addr, err);
	} else {
		ESP_LOGI(TAG, "GW_TX_SPECIAL_OK dst=0x%04x op=0x%04x data=%u ttl=7 (Broadcast with TTL=7)",
			 ctx.addr, SPECIAL_SENSOR_OP, data_val);
	}

	return err;
}

static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                                esp_ble_mesh_generic_server_cb_param_t *param)
{
	if (!param || !param->model) {
		return;
	}

	if (event == ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT) {
		if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
		    param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
			uint8_t onoff = param->value.set.onoff.onoff;
			board_led_operation(LED_0, onoff);

			if (onoff) {
				ESP_LOGI(TAG, "GW_ONOFF ON received from nRF Mesh app -> Broadcasting Special Sensor Message (data=1, TTL=7) to ALL NODES (0xFFFF)");
				send_special_sensor_message(param->model, &param->ctx, 1);
			} else {
				ESP_LOGI(TAG, "GW_ONOFF OFF received from nRF Mesh app -> Broadcasting Special Sensor Message (data=0, TTL=7) to ALL NODES (0xFFFF)");
				send_special_sensor_message(param->model, &param->ctx, 0);
			}

			if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
				esp_ble_mesh_gen_onoff_status_cb_t status = {
					.present_onoff = onoff,
					.target_onoff = onoff,
					.remain_time = 0,
				};
				esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
					ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(status), (uint8_t *)&status);
			}
		}
	} else if (event == ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT) {
		if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
			esp_ble_mesh_gen_onoff_status_cb_t status = {
				.present_onoff = 0,
				.target_onoff = 0,
				.remain_time = 0,
			};
			esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
				ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(status), (uint8_t *)&status);
		}
	}
}

static void example_ble_mesh_sensor_client_cb(esp_ble_mesh_sensor_client_cb_event_t event,
                                               esp_ble_mesh_sensor_client_cb_param_t *param)
{
	if (!param || !param->params) {
		return;
	}

	uint16_t src_addr = param->params->ctx.addr;
	uint16_t dst_addr = param->params->ctx.recv_dst;
	uint8_t recv_ttl = param->params->ctx.recv_ttl;
	int8_t recv_rssi = param->params->ctx.recv_rssi;

	indicate_sensor_rx();

	if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS ||
	    param->params->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS) {

		struct source_measurement *m = measurement_for(src_addr);
		uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
		if (!m->data_valid && !m->battery_valid) {
			m->first_seen_ms = now;
		}
		m->last_seen_ms = now;
		sensor_callback_count++;

		if (param->status_cb.sensor_status.marshalled_sensor_data &&
		    param->status_cb.sensor_status.marshalled_sensor_data->len > 0) {

			uint8_t *data = param->status_cb.sensor_status.marshalled_sensor_data->data;
			uint16_t len = param->status_cb.sensor_status.marshalled_sensor_data->len;
			uint16_t pos = 0;

			while (pos < len) {
				uint8_t fmt = ESP_BLE_MESH_GET_SENSOR_DATA_FORMAT(data + pos);
				uint8_t data_len = ESP_BLE_MESH_GET_SENSOR_DATA_LENGTH(data + pos, fmt);
				uint16_t prop_id = ESP_BLE_MESH_GET_SENSOR_DATA_PROPERTY_ID(data + pos, fmt);
				uint8_t mpid_len = (fmt == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A ? 2 : 3);
				uint8_t val = (pos + mpid_len < len) ? data[pos + mpid_len] : 0;

				if (prop_id == 0x0042) {
					m->data = val;
					m->data_valid = true;
				} else if (prop_id == 0x0054) {
					m->battery = val;
					m->battery_valid = true;
				}

				ESP_LOGI(TAG, "GW_RX count=%" PRIu32 " t_ms=%" PRIu32 " src=0x%04x dst=0x%04x property=0x%04x (%s) data=%u battery=%u ttl=%u rssi=%d%s",
					 sensor_callback_count, now, src_addr, dst_addr, prop_id,
					 (prop_id == 0x0042 ? "DATA/MOTION" : prop_id == 0x0054 ? "BATTERY" : "OTHER"),
					 m->data, m->battery, recv_ttl, recv_rssi,
					 (recv_ttl == 1 ? " [TTL=1 Direct Rx]" : ""));

				pos += mpid_len + data_len + 1;
			}
		}

		print_measurement(m, dst_addr, recv_ttl, recv_rssi);
	}
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                               esp_ble_mesh_cfg_server_cb_param_t *param)
{
	if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
		switch (param->ctx.recv_op) {
		case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
			ESP_LOGI(TAG, "[CONFIG SRV] AppKey Added: net_idx=0x%04x, app_idx=0x%04x",
				 param->value.state_change.appkey_add.net_idx,
				 param->value.state_change.appkey_add.app_idx);
			break;
		case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
			ESP_LOGI(TAG, "[CONFIG SRV] Model App Bound: elem_addr=0x%04x, app_idx=0x%04x, model_id=0x%04x",
				 param->value.state_change.mod_app_bind.element_addr,
				 param->value.state_change.mod_app_bind.app_idx,
				 param->value.state_change.mod_app_bind.model_id);
			break;
		case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
			ESP_LOGI(TAG, "[CONFIG SRV] Model Group Subscription Added: elem_addr=0x%04x, sub_addr=0x%04x, model_id=0x%04x",
				 param->value.state_change.mod_sub_add.element_addr,
				 param->value.state_change.mod_sub_add.sub_addr,
				 param->value.state_change.mod_sub_add.model_id);
			break;
		default:
			ESP_LOGI(TAG, "[CONFIG SRV] Config State Change opcode=0x%06" PRIx32, param->ctx.recv_op);
			break;
		}
	}
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                              esp_ble_mesh_prov_cb_param_t *param)
{
	switch (event) {
	case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
		ESP_LOGI(TAG, "[PROV] Provisioning Register Complete, err_code %d", param->prov_register_comp.err_code);
		break;
	case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
		ESP_LOGI(TAG, "[PROV] Node Provisioning Bearer Enabled (ADV|GATT), err_code %d", param->node_prov_enable_comp.err_code);
		break;
	case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
		ESP_LOGI(TAG, "[PROV] Provisioning Link Opened (bearer: %s)",
			 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
		break;
	case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
		ESP_LOGI(TAG, "[PROV] Provisioning Link Closed (bearer: %s)",
			 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
		break;
	case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
		ESP_LOGI(TAG, "[PROV] *** NODE PROVISIONED SUCCESSFULLY *** assigned unicast_addr=0x%04x, net_idx=0x%04x",
			 param->node_prov_complete.addr, param->node_prov_complete.net_idx);
		break;
	case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
		ESP_LOGW(TAG, "[PROV] Node Provisioning Reset event received");
		break;
	default:
		break;
	}
}

static void status_timer_callback(void* arg)
{
	uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
	ESP_LOGI(TAG, "GW_STATUS Listening on group 0x%04x... total_rx=%" PRIu32 ", complete_packets=%" PRIu32 ", uptime=%" PRIu32 "s",
		 SENSOR_GROUP_ADDR, sensor_callback_count, complete_packet_count, uptime_s);
}

static esp_err_t bluetooth_init(void)
{
	esp_err_t ret;

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_bt_controller_init failed: %d", ret);
		return ret;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_bt_controller_enable failed: %d", ret);
		return ret;
	}

	ret = esp_bluedroid_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_bluedroid_init failed: %d", ret);
		return ret;
	}

	ret = esp_bluedroid_enable();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_bluedroid_enable failed: %d", ret);
		return ret;
	}

	return ESP_OK;
}

static esp_err_t ble_mesh_init(void)
{
	esp_err_t err;

	esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
	esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
	esp_ble_mesh_register_sensor_client_callback(example_ble_mesh_sensor_client_cb);
	esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);

	err = esp_ble_mesh_init(&provision, &composition);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize mesh stack");
		return err;
	}

	err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to enable mesh node");
		return err;
	}

	esp_ble_mesh_set_unprovisioned_device_name("ESP32S3-Gateway");

	ESP_LOGI(TAG, "GW_INIT Sensor Client initialized, waiting for Sensor Status");

	/* Create periodic status timer (30s interval) */
	const esp_timer_create_args_t timer_args = {
		.callback = &status_timer_callback,
		.name = "gw_status_timer"
	};
	esp_timer_create(&timer_args, &status_timer);
	esp_timer_start_periodic(status_timer, 30000000); /* 30 seconds */

	/* Create 1s Rx LED auto-off timer */
	const esp_timer_create_args_t led_timer_args = {
		.callback = &rx_led_off_timer_cb,
		.name = "rx_led_off_timer"
	};
	esp_timer_create(&led_timer_args, &rx_led_timer);

	return ESP_OK;
}

void app_main(void)
{
	esp_err_t err;

	/* Disable hardware Brownout Detector instantly to prevent RF calibration power-dip resets */
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

	/* Enable full logging for debugging ESP32-S3 BLE Mesh operations */
	esp_log_level_set("*", ESP_LOG_INFO);

	ESP_LOGI(TAG, "Initializing...");

	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	board_init();

	err = bluetooth_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "bluetooth_init failed (err %d)", err);
		return;
	}

	ble_mesh_get_dev_uuid(dev_uuid);

	err = ble_mesh_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
	}
}

