/* Leaf composition and application model handling. */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/sensor_types.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

#include "model_handler.h"

extern uint16_t bt_mesh_primary_addr(void);

LOG_MODULE_REGISTER(model_handler, CONFIG_LOG_DEFAULT_LEVEL);

#define SENSOR_AWAKE_DURATION_MS  (LEAF_AWAKE_DURATION_SEC * 1000)
#define SENSOR_SLEEP_DURATION_MS  (LEAF_SLEEP_DURATION_SEC * 1000)
#define SENSOR_TX_GUARD_MS         500

static uint8_t simulated_sensor_value;
static uint8_t battery_value;

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
static const struct adc_dt_spec adc_channel =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
#endif

static uint8_t read_battery_level(void)
{
#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	int16_t raw = 0;
	struct adc_sequence sequence = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};
	int err;

	if (!adc_is_ready_dt(&adc_channel)) {
		return 101;
	}
	err = adc_channel_setup_dt(&adc_channel);
	if (err) {
		return 101;
	}
	err = adc_sequence_init_dt(&adc_channel, &sequence);
	if (err) {
		return 101;
	}
	err = adc_read_dt(&adc_channel, &sequence);

	/* Tắt ngay ngoại vi ADC để cắt sạch dòng ~350uA khi không đo */
	pm_device_action_run(adc_channel.dev, PM_DEVICE_ACTION_SUSPEND);

	if (err || raw <= 0) {
		return 100; /* Pin chưa nối hoặc ADC đọc 0 -> Giữ mặc định 100% */
	}

	int32_t mv = raw;
	if (adc_raw_to_millivolts_dt(&adc_channel, &mv) < 0) {
		mv = (int32_t)raw * 3600 / 4095;
	}

	if (mv <= 0) {
		return 100;
	}

	return (uint8_t)CLAMP(((mv - 900) * 100) / 600, 0, 100);
#else
	return 100;
#endif
}

static int sensor_value_get(struct bt_mesh_sensor_srv *srv,
				    struct bt_mesh_sensor *sensor,
				    struct bt_mesh_msg_ctx *ctx,
				    struct bt_mesh_sensor_value *rsp)
{
	uint8_t percent = sensor->type->id == bt_mesh_sensor_motion_sensed.id ?
		simulated_sensor_value : battery_value;
	int err = bt_mesh_sensor_value_from_micro(
		sensor->type->channels[0].format, (int64_t)percent * 1000000LL, rsp);

	return err == -ERANGE ? 0 : err;
}

static struct bt_mesh_sensor sensor_data = {
	.type = &bt_mesh_sensor_motion_sensed,
	.get = sensor_value_get,
};

static struct bt_mesh_sensor sensor_battery = {
	.type = &bt_mesh_sensor_present_dev_op_efficiency,
	.get = sensor_value_get,
};

static struct bt_mesh_sensor *const sensors[] = {
	&sensor_data,
	&sensor_battery,
};

static struct bt_mesh_sensor_srv sensor_srv =
	BT_MESH_SENSOR_SRV_INIT(sensors, ARRAY_SIZE(sensors));

struct led_ctx {
	struct bt_mesh_onoff_srv srv;
	struct k_work_delayable work;
	uint32_t remaining;
	bool value;
};

static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
			const struct bt_mesh_onoff_set *set,
			struct bt_mesh_onoff_status *rsp);
static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
			struct bt_mesh_onoff_status *rsp);

static const struct bt_mesh_onoff_srv_handlers onoff_handlers = {
	.set = led_set,
	.get = led_get,
};

static struct led_ctx led_ctx[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
};

static struct k_work_delayable cycle_work;
static struct k_work_delayable publish_work;
static struct k_work_delayable suspend_work;
static struct k_work_delayable mesh_config_work;
static bool cycle_mode_active;
static bool is_awake = true;
static uint8_t tx_retry_count;
static uint32_t tx_sequence;

static void update_awake_led(void)
{
	if (is_awake) {
		dk_set_led(0, true);
	} else {
		/* Trong trạng thái ngủ: Tắt TOÀN BỘ các đèn LED để tiết kiệm điện tối đa */
		dk_set_leds(DK_NO_LEDS_MSK);
	}
}

static uint32_t compute_tx_delay_ms(void)
{
#if defined(CONFIG_BOARD_NRF52_BSIM)
	const char *index_str = getenv("NODE_INDEX");
	uint32_t index = index_str ? strtoul(index_str, NULL, 0) : 1;
	const char *nodes_str = getenv("NUM_NODES");
	uint32_t nodes = nodes_str ? strtoul(nodes_str, NULL, 0) : 9;
	uint32_t leaves = MAX(nodes > 1 ? nodes - 1 : 1, 1U);
	return SENSOR_TX_GUARD_MS + ((index - 1U) % leaves) * (15000U / leaves);
#else
	return SENSOR_TX_GUARD_MS;
#endif
}

static void log_mesh_model_state(const char *tag)
{
	const struct bt_mesh_model *model = sensor_srv.model;

	if (!model || !model->pub) {
		LOG_WRN("LEAF_MODEL %s unavailable", tag);
		return;
	}

	LOG_INF("LEAF_MODEL %s primary=0x%04x pub=0x%04x pub_key=0x%03x ttl=%u "
		"keys=%u groups=%u",
		tag, bt_mesh_primary_addr(), model->pub->addr, model->pub->key,
		model->pub->ttl, model->keys_cnt, model->groups_cnt);

	for (int i = 0; i < model->keys_cnt; ++i) {
		LOG_INF("LEAF_MODEL %s key[%d]=0x%03x", tag, i, model->keys[i]);
	}
	for (int i = 0; i < model->groups_cnt; ++i) {
		LOG_INF("LEAF_MODEL %s group[%d]=0x%04x", tag, i, model->groups[i]);
	}
}

static void mesh_config_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	const struct bt_mesh_model *model = sensor_srv.model;
	if (!model || !model->pub) {
		LOG_WRN("LEAF_MODEL configuration delayed: Sensor Server not ready");
		k_work_reschedule(&mesh_config_work, K_SECONDS(2));
		return;
	}

	bool provisioned = bt_mesh_is_provisioned();
	bool pub_configured = (model->pub->addr != BT_MESH_ADDR_UNASSIGNED) &&
			     (model->pub->key != BT_MESH_KEY_UNUSED);

	if (provisioned && pub_configured) {
		static bool state_logged = false;
		if (!state_logged) {
			log_mesh_model_state("configured");
			state_logged = true;
		}
		if (!cycle_mode_active) {
			LOG_INF("LEAF_STATUS Ready & Provisioned! Send ON command to Element 2 (e.g. Group 0xC002) via nRF Mesh app to start sensor cycle.");
			k_work_reschedule(&mesh_config_work, K_SECONDS(30));
		}
	} else {
		LOG_WRN("LEAF_MODEL status: provisioned=%u, pub_configured=%u (Use nRF Mesh app to provision & set publication)",
			provisioned, pub_configured);
		k_work_reschedule(&mesh_config_work, K_SECONDS(5));
	}
}

static void led_status(struct led_ctx *led, struct bt_mesh_onoff_status *status)
{
	status->remaining_time = led->remaining ? led->remaining :
		k_ticks_to_ms_ceil32(k_work_delayable_remaining_get(&led->work));
	status->target_on_off = led->value;
	status->present_on_off = led->value || status->remaining_time;
}

static void led_transition_start(struct led_ctx *led)
{
	int idx = led - &led_ctx[0];
	if (idx != 0) {
		dk_set_led(idx, true);
	}
	k_work_reschedule(&led->work, K_MSEC(led->remaining));
	led->remaining = 0;
}

static void led_work(struct k_work *work)
{
	struct led_ctx *led = CONTAINER_OF(work, struct led_ctx, work.work);
	int idx = led - &led_ctx[0];

	if (led->remaining) {
		led_transition_start(led);
		return;
	}
	if (is_awake && idx != 0) {
		dk_set_led(idx, led->value);
	}
	struct bt_mesh_onoff_status status;
	led_status(led, &status);
	bt_mesh_onoff_srv_pub(&led->srv, NULL, &status);
}

#define SENSOR_OP_STATUS 0x52

static int send_sensor_status_message(uint8_t motion_val, uint8_t battery_val)
{
	if (!sensor_srv.model || !sensor_srv.model->pub) {
		return -EINVAL;
	}

	uint16_t dst_addr = sensor_srv.model->pub->addr;
	uint16_t app_idx = sensor_srv.model->pub->key;

	if (dst_addr == BT_MESH_ADDR_UNASSIGNED || app_idx == BT_MESH_KEY_UNUSED) {
		return -EADDRNOTAVAIL;
	}

	uint8_t msg_data[32];
	struct net_buf_simple msg;
	net_buf_simple_init_with_data(&msg, msg_data, sizeof(msg_data));
	bt_mesh_model_msg_init(&msg, SENSOR_OP_STATUS);

	/* Simple, rock-solid 2-byte Payload: Byte 0 = Motion Data, Byte 1 = Battery Level */
	net_buf_simple_add_u8(&msg, motion_val);
	net_buf_simple_add_u8(&msg, battery_val);

	/* Always use node's SDK Default TTL (bt_mesh_default_ttl_get()), ignoring nRF Mesh app TTL override */
	uint8_t send_ttl = bt_mesh_default_ttl_get();

	struct bt_mesh_msg_ctx ctx = {
		.addr     = dst_addr,
		.net_idx  = 0,
		.app_idx  = app_idx,
		.send_ttl = send_ttl,
	};

	return bt_mesh_model_send(sensor_srv.model, &ctx, &msg, NULL, NULL);
}

#define SPECIAL_SENSOR_OP 0x8299
static struct k_work_delayable special_seq_work;

static void special_seq_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_INF("=========================================================================");
	LOG_INF("[LEAF 5s TIMER EXPIRED] Transmitting Sensor Model Status & Battery packet...");
	LOG_INF("=========================================================================");

	/* Send Sensor Status Data + Battery Packet */
	battery_value = read_battery_level();
	LOG_INF("[LEAF SENSOR TX] Transmitting Sensor Model data packet (motion=%u%%, battery=%u%%, send_ttl=%u)",
		simulated_sensor_value, battery_value, bt_mesh_default_ttl_get());
	send_sensor_status_message(simulated_sensor_value, battery_value);
}

void bt_mesh_special_pkt_rx_notify(uint8_t data_val)
{
	LOG_INF("=========================================================================");
	LOG_INF("[LEAF RX SPECIAL PKT] Received data=%u! Scheduling %us timer for Sensor Status packet...", data_val, SPECIAL_PKT_DELAY_SEC);
	LOG_INF("=========================================================================");
	k_work_reschedule(&special_seq_work, K_SECONDS(SPECIAL_PKT_DELAY_SEC));
}

static void publish_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!cycle_mode_active || !is_awake || !sensor_srv.model) {
		LOG_WRN("LEAF_TX skipped active=%u awake=%u model=%u",
			cycle_mode_active, is_awake, sensor_srv.model != NULL);
		return;
	}

	if (!sensor_srv.model->pub ||
	    sensor_srv.model->pub->addr == BT_MESH_ADDR_UNASSIGNED ||
	    sensor_srv.model->pub->key == BT_MESH_KEY_UNUSED) {
		LOG_WRN("LEAF_TX skipped: Dynamic publication not configured via nRF Mesh app yet (addr=0x%04x key=0x%03x)",
			sensor_srv.model->pub ? sensor_srv.model->pub->addr : 0,
			sensor_srv.model->pub ? sensor_srv.model->pub->key : BT_MESH_KEY_UNUSED);
		return;
	}

	battery_value = read_battery_level();
	uint32_t sequence = ++tx_sequence;
	uint32_t now = k_uptime_get_32();
	uint8_t actual_ttl = bt_mesh_default_ttl_get();
	LOG_INF("LEAF_TX seq=%u t_ms=%u dst=0x%04x pub_key=0x%03x ttl=%u data=%u battery=%u retry=%u",
		sequence, now, sensor_srv.model->pub->addr,
		sensor_srv.model->pub->key, actual_ttl,
		simulated_sensor_value, battery_value, tx_retry_count);

	int err = send_sensor_status_message(simulated_sensor_value, battery_value);
	if (err) {
		LOG_ERR("LEAF_TX_FAIL seq=%u err=%d (%s)", sequence, err,
			err == -EADDRNOTAVAIL ? "publication address unavailable" :
			err == -EINVAL ? "model/AppKey not bound or invalid" : "mesh send error");
	}
	if (err && tx_retry_count++ < SENSOR_TX_MAX_RETRIES) {
		LOG_WRN("LEAF_TX_RETRY seq=%u next_in_ms=%u count=%u",
			sequence, SENSOR_TX_RETRY_DELAY_MS, tx_retry_count);
		k_work_reschedule(&publish_work, K_MSEC(SENSOR_TX_RETRY_DELAY_MS));
	} else if (!err) {
		LOG_INF("LEAF_TX_OK seq=%u (Packet successfully transmitted over BLE Mesh)", sequence);
		tx_retry_count = 0;
	} else {
		LOG_ERR("LEAF_TX_GIVE_UP seq=%u", sequence);
	}
}

static void suspend_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (is_awake || !cycle_mode_active) {
		return;
	}

	/* Tắt 100% tất cả các đèn LED phần cứng */
	dk_set_leds(DK_NO_LEDS_MSK);

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	if (adc_is_ready_dt(&adc_channel)) {
		pm_device_action_run(adc_channel.dev, PM_DEVICE_ACTION_SUSPEND);
	}
#endif

#if !defined(CONFIG_BOARD_NRF52_BSIM)
	bt_mesh_suspend();
	bt_le_adv_stop();
#endif
	LOG_INF("LEAF_RADIO Suspended radio for sleep cycle");
}

static void cycle_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!cycle_mode_active) {
		return;
	}

	is_awake = !is_awake;
	LOG_INF("LEAF_CYCLE t_ms=%u state=%s", k_uptime_get_32(),
		is_awake ? "awake" : "sleep");
	if (is_awake) {
#if !defined(CONFIG_BOARD_NRF52_BSIM)
		bt_mesh_resume();
#endif
		simulated_sensor_value = (simulated_sensor_value + 5U) % 201U;
		tx_retry_count = 0;
		update_awake_led();
		k_work_reschedule(&publish_work, K_MSEC(compute_tx_delay_ms()));
		k_work_reschedule(&cycle_work, K_MSEC(SENSOR_AWAKE_DURATION_MS));
	} else {
		k_work_cancel_delayable(&publish_work);
		update_awake_led();
		/* Delay radio suspend by 1.5s so in-flight BLE ADV buffers can transmit */
		k_work_reschedule(&suspend_work, K_MSEC(1500));
		k_work_reschedule(&cycle_work, K_MSEC(SENSOR_SLEEP_DURATION_MS));
	}
}

static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
			const struct bt_mesh_onoff_set *set,
			struct bt_mesh_onoff_status *rsp)
{
	struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);
	int idx = led - &led_ctx[0];
	led->value = set->on_off;
	LOG_INF("LEAF_ONOFF idx=%d value=%u src=0x%04x", idx, set->on_off,
		ctx ? ctx->addr : BT_MESH_ADDR_UNASSIGNED);

	if (idx == 1) {
		/* Element 2 (idx 1) controls cycle mode via Group / Unicast OnOff commands */
		if (set->on_off == LEAF_LED_STATE_ON) {
			if (!cycle_mode_active) {
				led->value = LEAF_LED_STATE_ON;
				cycle_mode_active = true;
				is_awake = true;
				tx_retry_count = 0;
				update_awake_led();
				LOG_INF("LEAF_CYCLE Started cycle mode via OnOff ON command!");
				k_work_reschedule(&publish_work, K_MSEC(500));
				k_work_reschedule(&cycle_work, K_MSEC(SENSOR_AWAKE_DURATION_MS));
			}
		} else {
			if (cycle_mode_active) {
				cycle_mode_active = false;
				is_awake = false;
				k_work_cancel_delayable(&publish_work);
				k_work_cancel_delayable(&cycle_work);
				k_work_cancel_delayable(&suspend_work);
				update_awake_led();
#if !defined(CONFIG_BOARD_NRF52_BSIM)
				bt_mesh_suspend();
#endif
				LOG_INF("LEAF_CYCLE Stopped cycle mode via OnOff OFF command");
			}
			led->value = LEAF_LED_STATE_OFF;
		}
	} else if (!bt_mesh_model_transition_time(set->transition)) {
		led->remaining = 0;
		if (is_awake && idx != 0) {
			dk_set_led(idx, set->on_off);
		}
	} else {
		led->remaining = set->transition->time;
		k_work_reschedule(&led->work, K_MSEC(set->transition->delay));
	}

	if (rsp) {
		led_status(led, rsp);
	}
}

static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
			struct bt_mesh_onoff_status *rsp)
{
	ARG_UNUSED(ctx);
	led_status(CONTAINER_OF(srv, struct led_ctx, srv), rsp);
}

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
		update_awake_led();
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
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};
static struct bt_mesh_health_srv health_srv = { .cb = &health_srv_cb };
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_elem elements[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
	BT_MESH_ELEM(1,
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_CFG_SRV,
			BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
			BT_MESH_MODEL_SENSOR_SRV(&sensor_srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
	BT_MESH_ELEM(3,
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[1].srv)),
		BT_MESH_MODEL_NONE),
#endif
};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

extern bool ttl_modified_by_special_pkt;

const struct bt_mesh_comp *model_handler_init(void)
{
	ttl_modified_by_special_pkt = false;
	bt_mesh_default_ttl_set(7);

	k_work_init_delayable(&cycle_work, cycle_handler);
	k_work_init_delayable(&publish_work, publish_handler);
	k_work_init_delayable(&suspend_work, suspend_handler);
	k_work_init_delayable(&special_seq_work, special_seq_handler);
	k_work_init_delayable(&mesh_config_work, mesh_config_handler);
	k_work_init_delayable(&attention_blink_work, attention_blink);
	for (int i = 0; i < ARRAY_SIZE(led_ctx); ++i) {
		k_work_init_delayable(&led_ctx[i].work, led_work);
	}
	is_awake = true;
	tx_sequence = 0;
	update_awake_led();
	LOG_INF("LEAF_INIT Sensor Server ready; waiting for trigger/configuration (TTL reset to default, ttl_modified=false)");
	k_work_reschedule(&mesh_config_work, K_SECONDS(2));
	return &comp;
}

