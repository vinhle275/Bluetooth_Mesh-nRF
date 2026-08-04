/* Gateway composition and standard Sensor Client handling. */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/sensor_types.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

#include "model_handler.h"

extern uint16_t bt_mesh_primary_addr(void);

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_INF);

#define SENSOR_GROUP_ADDR 0xC000
#define MAX_TRACKED_SOURCES 32
#define SENSOR_RX_LED 1
#define SENSOR_RX_LED_DURATION K_SECONDS(1)

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
static struct bt_mesh_sensor_cli sensor_cli;
static struct k_work_delayable rx_led_off_work;
static struct k_work_delayable mesh_config_work;
static uint32_t sensor_callback_count;
static uint32_t complete_packet_count;

static void rx_led_off_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	dk_set_led(SENSOR_RX_LED, false);
}

static void indicate_sensor_rx(void)
{
	dk_set_led(SENSOR_RX_LED, true);
	k_work_reschedule(&rx_led_off_work, SENSOR_RX_LED_DURATION);
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
		free_slot->first_seen_ms = k_uptime_get_32();
		return free_slot;
	}

	/* Replace the oldest source if the table is full. */
	struct source_measurement *oldest = &measurements[0];
	for (int i = 1; i < ARRAY_SIZE(measurements); ++i) {
		if (measurements[i].last_seen_ms < oldest->last_seen_ms) {
			oldest = &measurements[i];
		}
	}
	*oldest = (struct source_measurement){
		.addr = addr,
		.first_seen_ms = k_uptime_get_32(),
	};
	return oldest;
}

static void log_sensor_client_state(const char *tag)
{
	const struct bt_mesh_model *model = sensor_cli.model;

	if (!model) {
		LOG_WRN("GW_MODEL %s unavailable", tag);
		return;
	}

	LOG_INF("GW_MODEL %s primary=0x%04x keys=%u groups=%u",
		tag, bt_mesh_primary_addr(), model->keys_cnt, model->groups_cnt);
	for (int i = 0; i < model->keys_cnt; ++i) {
		LOG_INF("GW_MODEL %s key[%d]=0x%03x", tag, i, model->keys[i]);
	}
	for (int i = 0; i < model->groups_cnt; ++i) {
		LOG_INF("GW_MODEL %s group[%d]=0x%04x", tag, i, model->groups[i]);
	}
}

static void mesh_config_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	const struct bt_mesh_model *model = sensor_cli.model;

	if (!model) {
		LOG_WRN("GW_MODEL configuration delayed: Sensor Client not ready");
		k_work_reschedule(&mesh_config_work, K_SECONDS(2));
		return;
	}

	bool has_subscription = false;
	for (int i = 0; i < model->groups_cnt; ++i) {
		if (model->groups[i] != BT_MESH_ADDR_UNASSIGNED) {
			has_subscription = true;
			break;
		}
	}
	bool has_key = (model->keys_cnt > 0 && model->keys[0] != BT_MESH_KEY_UNUSED);

	if (bt_mesh_is_provisioned() && has_subscription && has_key) {
		static bool state_logged = false;
		if (!state_logged) {
			log_sensor_client_state("configured");
			state_logged = true;
		}
		LOG_INF("GW_STATUS Listening on group 0x%04x... total_rx=%u, complete_packets=%u, uptime=%us",
			model->groups[0], sensor_callback_count, complete_packet_count, k_uptime_get_32() / 1000);
		k_work_reschedule(&mesh_config_work, K_SECONDS(30));
	} else {
		LOG_WRN("GW_MODEL status: provisioned=%u, has_key=%u, has_sub=%u (Use nRF Mesh app to Bind AppKey & Subscribe Sensor Client)",
			bt_mesh_is_provisioned(), has_key, has_subscription);
		k_work_reschedule(&mesh_config_work, K_SECONDS(5));
	}
}

static void print_measurement(struct source_measurement *m,
				      const struct bt_mesh_msg_ctx *ctx)
{
	uint32_t now = k_uptime_get_32();
	uint32_t delta_ms = (m->last_report_ms > 0) ? (now - m->last_report_ms) : 0;

	if (m->data_valid && m->battery_valid) {
		m->last_report_ms = now;
		complete_packet_count++;

		LOG_INF("=========================================================================");
		LOG_INF("GW_PACKET RECEIVED! count=%u t_ms=%u src=0x%04x motion=%u%% battery=%u%% "
			"delta_ms=%u ttl=%u dst=0x%04x rssi=%d",
			complete_packet_count, now, m->addr, m->data, m->battery,
			delta_ms, ctx->recv_ttl, ctx->recv_dst, ctx->recv_rssi);
		LOG_INF("=========================================================================");

		m->data_valid = false;
		m->battery_valid = false;
		m->first_seen_ms = now;
	} else if (m->data_valid) {
		m->last_report_ms = now;
		complete_packet_count++;

		LOG_INF("=========================================================================");
		LOG_INF("GW_PACKET RECEIVED! count=%u t_ms=%u src=0x%04x motion=%u%% "
			"delta_ms=%u ttl=%u dst=0x%04x rssi=%d",
			complete_packet_count, now, m->addr, m->data,
			delta_ms, ctx->recv_ttl, ctx->recv_dst, ctx->recv_rssi);
		LOG_INF("=========================================================================");
	}
}

static void sensor_data_cb(struct bt_mesh_sensor_cli *cli,
				   struct bt_mesh_msg_ctx *ctx,
				   const struct bt_mesh_sensor_type *type,
				   const struct bt_mesh_sensor_value *value)
{
	ARG_UNUSED(cli);
	indicate_sensor_rx();
	struct source_measurement *m = measurement_for(ctx->addr);
	uint32_t now = k_uptime_get_32();
	if (!m->data_valid && !m->battery_valid) {
		m->first_seen_ms = now;
	}
	m->last_seen_ms = now;
	sensor_callback_count++;

	float value_f;
	enum bt_mesh_sensor_value_status value_status =
		bt_mesh_sensor_value_to_float(value, &value_f);
	if (!bt_mesh_sensor_value_status_is_numeric(value_status)) {
		LOG_WRN("[SENSOR RX] src=0x%04x dst=0x%04x property=0x%04x "
			 "non-numeric status=%d ttl=%u rssi=%d",
			 ctx->addr, ctx->recv_dst, type->id, value_status,
			 ctx->recv_ttl, ctx->recv_rssi);
		return;
	}
	uint8_t percent = (uint8_t)CLAMP((int)value_f, 0, 100);

	LOG_INF("GW_RX count=%u t_ms=%u src=0x%04x dst=0x%04x property=0x%04x "
		"value=%u%% ttl=%u rssi=%d",
		sensor_callback_count, now, ctx->addr, ctx->recv_dst, type->id, percent,
		ctx->recv_ttl, ctx->recv_rssi);

	if (type == &bt_mesh_sensor_motion_sensed ||
	    type->id == bt_mesh_sensor_motion_sensed.id ||
	    type->id == 0x0042) {
		m->data = percent;
		m->data_valid = true;
	} else if (type == &bt_mesh_sensor_present_dev_op_efficiency ||
		   type->id == bt_mesh_sensor_present_dev_op_efficiency.id ||
		   type->id == 0x0054) {
		m->battery = percent;
		m->battery_valid = true;
	} else {
		m->data = percent;
		m->data_valid = true;
		LOG_WRN("[SENSOR RX] Known property 0x%04x value=%u%% from 0x%04x",
			type->id, percent, ctx->addr);
	}

	print_measurement(m, ctx);
}

static void sensor_unknown_type_cb(struct bt_mesh_sensor_cli *cli,
				   struct bt_mesh_msg_ctx *ctx, uint16_t id,
				   uint32_t opcode)
{
	ARG_UNUSED(cli);
	indicate_sensor_rx();
	sensor_callback_count++;
	LOG_WRN("GW_RX_UNKNOWN count=%u t_ms=%u src=0x%04x dst=0x%04x "
		 "property=0x%04x opcode=0x%08x ttl=%u rssi=%d",
		 sensor_callback_count, k_uptime_get_32(), ctx->addr, ctx->recv_dst, id, opcode,
		 ctx->recv_ttl, ctx->recv_rssi);
}

static const struct bt_mesh_sensor_cli_handlers sensor_handlers = {
	.data = sensor_data_cb,
	.unknown_type = sensor_unknown_type_cb,
};

#define SPECIAL_SENSOR_OP BT_MESH_MODEL_OP_2(0x82, 0x99)

int send_special_sensor_message(uint16_t dst_addr, uint8_t data_val)
{
	if (!sensor_cli.model) {
		LOG_WRN("GW_TX_SPECIAL: sensor_cli model not initialized");
		return -EINVAL;
	}

	uint16_t app_idx = (sensor_cli.model->keys_cnt > 0) ? sensor_cli.model->keys[0] : BT_MESH_KEY_UNUSED;
	if (app_idx == BT_MESH_KEY_UNUSED) {
		LOG_WRN("GW_TX_SPECIAL: AppKey not bound yet on Gateway");
		return -EADDRNOTAVAIL;
	}

	NET_BUF_SIMPLE_DEFINE(msg, 16);
	bt_mesh_model_msg_init(&msg, SPECIAL_SENSOR_OP);

	/* Add 8-bit data field with value = data_val (e.g. 1) */
	net_buf_simple_add_u8(&msg, data_val);

	struct bt_mesh_msg_ctx ctx = {
		.addr     = dst_addr,
		.net_idx  = 0,
		.app_idx  = app_idx,
		.send_ttl = 1, /* TTL = 1: Direct 1-hop BLE ADV broadcast, NO RELAY */
	};

	int err = bt_mesh_model_send(sensor_cli.model, &ctx, &msg, NULL, NULL);
	if (err) {
		LOG_ERR("GW_TX_SPECIAL_FAIL dst=0x%04x err=%d", dst_addr, err);
	} else {
		LOG_INF("GW_TX_SPECIAL_OK dst=0x%04x op=0x%04x data=%u ttl=1 (Direct 1-hop transmission without relay)",
			dst_addr, SPECIAL_SENSOR_OP, data_val);
	}

	return err;
}

static void onoff_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
			      const struct bt_mesh_onoff_set *set,
			      struct bt_mesh_onoff_status *rsp)
{
	ARG_UNUSED(srv);
	ARG_UNUSED(ctx);
	rsp->present_on_off = set->on_off;
	rsp->target_on_off = set->on_off;
	rsp->remaining_time = 0;
	dk_set_led(0, set->on_off);

	if (set->on_off) {
		LOG_INF("GW_ONOFF ON received from nRF Mesh app -> Broadcasting 1-hop Special Sensor Message (data=1, TTL=1) to ALL NODES (0xFFFF)");
		send_special_sensor_message(BT_MESH_ADDR_ALL_NODES, 1);
	} else {
		LOG_INF("GW_ONOFF OFF received from nRF Mesh app -> Broadcasting 1-hop Special Sensor Message (data=0, TTL=1) to ALL NODES (0xFFFF)");
		send_special_sensor_message(BT_MESH_ADDR_ALL_NODES, 0);
	}
}

static void onoff_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
			      struct bt_mesh_onoff_status *rsp)
{
	ARG_UNUSED(srv);
	ARG_UNUSED(ctx);
	rsp->present_on_off = 0;
	rsp->target_on_off = 0;
	rsp->remaining_time = 0;
}

static const struct bt_mesh_onoff_srv_handlers onoff_handlers = {
	.set = onoff_set,
	.get = onoff_get,
};

static struct bt_mesh_onoff_srv onoff_srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers);

static bool attention;
static struct k_work_delayable attention_blink_work;

static void attention_blink(struct k_work *work)
{
	ARG_UNUSED(work);
	if (attention) {
		dk_set_leds(DK_ALL_LEDS_MSK);
		k_work_reschedule(&attention_blink_work, K_MSEC(100));
	} else {
		dk_set_leds(DK_NO_LEDS_MSK);
	}
}

static void attention_on(const struct bt_mesh_model *mod)
{
	ARG_UNUSED(mod);
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(const struct bt_mesh_model *mod)
{
	ARG_UNUSED(mod);
	attention = false;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};
static struct bt_mesh_health_srv health_srv = { .cb = &health_srv_cb };
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(1,
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_CFG_SRV,
			BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
			BT_MESH_MODEL_SENSOR_CLI(&sensor_cli)),
		BT_MESH_MODEL_NONE),
	BT_MESH_ELEM(2,
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&onoff_srv)),
		BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

const struct bt_mesh_comp *model_handler_init(void)
{
	sensor_cli = (struct bt_mesh_sensor_cli)BT_MESH_SENSOR_CLI_INIT(&sensor_handlers);
	k_work_init_delayable(&attention_blink_work, attention_blink);
	k_work_init_delayable(&rx_led_off_work, rx_led_off_handler);
	k_work_init_delayable(&mesh_config_work, mesh_config_handler);
	dk_set_led(SENSOR_RX_LED, false);
	sensor_callback_count = 0;
	complete_packet_count = 0;
	LOG_INF("GW_INIT Sensor Client initialized, waiting for Sensor Status");
	k_work_reschedule(&mesh_config_work, K_SECONDS(2));
	return &comp;
}

void check_and_self_provision(void)
{
#if defined(CONFIG_BOARD_NRF52_BSIM)
	const char *addr_str = getenv("NODE_ADDR");
	if (!addr_str) {
		return;
	}
	uint16_t addr = (uint16_t)strtoul(addr_str, NULL, 0);
	if (!addr) {
		return;
	}
	static const uint8_t net_key[16] = {
		0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
		0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
	};
	static const uint8_t dev_key[16] = {
		0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
		0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
	};
	static const uint8_t app_key[16] = {
		0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00, 0x11,
		0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99,
	};
	int err = bt_mesh_provision(net_key, 0, 0, 0, addr, dev_key);
	if (err && err != -EALREADY) {
		return;
	}
	err = bt_mesh_app_key_add(0, 0, app_key);
	if (err && err != -EALREADY) {
		return;
	}
	for (int i = 0; i < ARRAY_SIZE(elements); ++i) {
		for (int j = 0; j < elements[i].model_count; ++j) {
			struct bt_mesh_model *model = &elements[i].models[j];
			if (model->id != BT_MESH_MODEL_ID_CFG_SRV &&
			    model->id != BT_MESH_MODEL_ID_HEALTH_SRV &&
			    model->keys_cnt > 0) {
				model->keys[0] = 0;
			}
		}
	}

	if (sensor_cli.model && sensor_cli.model->groups && sensor_cli.model->groups_cnt > 0) {
		sensor_cli.model->groups[0] = SENSOR_GROUP_ADDR;
	}
#endif
}