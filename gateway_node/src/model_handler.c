/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * GATEWAY NODE - Gradient Routing Hub
 * ====================================
 * - Luon bat (khong co chu ky ngu/thuc)
 * - Khoi xuong GRADIENT_DISCOVER de xay dung bang dinh tuyen
 * - Nhan SENSOR_DATA tu tat ca cac node va hien thi
 * - Nhan GRADIENT_REPLY de biet trang thai cac node
 * - Tu dong tai kham pha gradient moi 200s (~10 chu ky leaf)
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/bluetooth/mesh/main.h>
#include <stdlib.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"

#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>

extern uint16_t bt_mesh_primary_addr(void);

LOG_MODULE_REGISTER(model_handler, CONFIG_LOG_DEFAULT_LEVEL);

#define ELEMENTS_SIZE (DT_NODE_EXISTS(DT_ALIAS(led0)) + DT_NODE_EXISTS(DT_ALIAS(led1)))
static struct bt_mesh_elem *p_elements;

/* ========================================================================= */
/* --- DINH NGHIA VENDOR MODEL VA OPCODE ---                                 */
/* ========================================================================= */
#define BT_MESH_MODEL_ID_SENSOR_CLI 0x1102
#define SENSOR_OP_STATUS            0x52

#define PROP_ID_SUBNET_REPORT       0xEEEE

struct subnet_report_val_t {
	uint16_t origin;
	int16_t  sensor_value;
	uint8_t  battery_level;
} __packed;

struct gradient_discover_msg_t {
	uint16_t origin;       /* Dia chi gateway goc */
	uint8_t  hop_count;    /* So hop tu gateway */
	uint16_t energy_cost;  /* Tong chi phi nang luong tich luy */
	uint8_t  sequence;     /* So thu tu de tranh xu ly goi cu */
} __packed;

struct gradient_reply_msg_t {
	uint16_t origin;       /* Dia chi node gui reply */
	uint8_t  battery_level;
	uint8_t  hop_count;    /* Khoang cach den gateway */
} __packed;

/* ========================================================================= */
/* --- DANH SACH SENSOR NODE ---                                             */
/* ========================================================================= */
#define MAX_SENSORS 256

struct sensor_node_t {
	uint16_t addr;
	int32_t  value;
	uint8_t  battery;
	bool     is_active;
};

static struct sensor_node_t sensor_list[MAX_SENSORS];
static uint32_t total_rx_count;
static uint32_t unique_rx_count;
static uint32_t current_cycle_rx_count;
static uint8_t current_cycle_seq;

static void update_and_print_sensor_list(uint16_t sender_addr, int32_t sensor_value, uint8_t battery_level)
{
	bool found = false;
	for (int i = 0; i < MAX_SENSORS; i++) {
		if (sensor_list[i].is_active && sensor_list[i].addr == sender_addr) {
			sensor_list[i].value = sensor_value;
			sensor_list[i].battery = battery_level;
			found = true;
			break;
		}
	}
	if (!found) {
		for (int i = 0; i < MAX_SENSORS; i++) {
			if (!sensor_list[i].is_active) {
				sensor_list[i].addr = sender_addr;
				sensor_list[i].value = sensor_value;
				sensor_list[i].battery = battery_level;
				sensor_list[i].is_active = true;
				break;
			}
		}
	}

	total_rx_count++;
	current_cycle_rx_count++;
	if (!found) {
		unique_rx_count++;
	}

	if ((total_rx_count % 25U) == 0U) {
		LOG_INF("GW_SUMMARY total_rx=%u unique_rx=%u cycle_rx=%u",
			total_rx_count, unique_rx_count, current_cycle_rx_count);
	}
}

/* ========================================================================= */
/* --- GRADIENT DISCOVERY STATE ---                                          */
/* ========================================================================= */
static uint8_t gradient_sequence = 0;
static const struct bt_mesh_model *vnd_srv_model;

static struct k_work_delayable gradient_discover_work;
static struct k_work_delayable gradient_rediscover_work;
static struct k_work_delayable init_subscription_work;
static int discover_burst_count = 0;

#define GRADIENT_DISCOVER_BURST_COUNT      5
#define GRADIENT_DISCOVER_BURST_INTERVAL   3000   /* ms giua moi lan gui trong 1 burst */
#define GRADIENT_REDISCOVER_INTERVAL       200000 /* ms ~ 10 chu ky leaf (10s thuc + 10s ngu) */

/* ========================================================================= */
/* --- FORWARD DECLARATIONS ---                                              */
/* ========================================================================= */
#define VND_COMPANY_ID            0x0059
#define VND_MODEL_ID_SRV          0x0002
#define VND_OP_GRADIENT_DISCOVER  BT_MESH_MODEL_OP_3(0x02, VND_COMPANY_ID)
#define VND_OP_GRADIENT_REPLY     BT_MESH_MODEL_OP_3(0x03, VND_COMPANY_ID)

static int handle_sensor_status(const struct bt_mesh_model *model,
				struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf);

static const struct bt_mesh_model *sensor_cli_model;

/* ========================================================================= */
/* --- MODEL OP ARRAYS ---                                                  */
/* ========================================================================= */
BT_MESH_MODEL_PUB_DEFINE(vnd_pub, NULL, 15);
BT_MESH_MODEL_PUB_DEFINE(sensor_cli_pub, NULL, 15);

static const struct bt_mesh_model_op vnd_srv_ops[] = {
	BT_MESH_MODEL_OP_END,
};

static const struct bt_mesh_model_op sensor_cli_ops[] = {
	{ SENSOR_OP_STATUS, 3, handle_sensor_status },
	BT_MESH_MODEL_OP_END,
};

/* ========================================================================= */
/* --- HAM GUI GRADIENT DISCOVER ---                                         */
/* ========================================================================= */
static void send_gradient_discover(void)
{
	if (!vnd_srv_model) {
		return;
	}

	/* Tim AppKey da bind */
	uint16_t app_idx = BT_MESH_KEY_UNUSED;
	for (int i = 0; i < CONFIG_BT_MESH_MODEL_KEY_COUNT; i++) {
		if (vnd_srv_model->keys[i] != BT_MESH_KEY_UNUSED) {
			app_idx = vnd_srv_model->keys[i];
			break;
		}
	}
	if (app_idx == BT_MESH_KEY_UNUSED) {
		LOG_WRN("--- [GRADIENT]: Chua co AppKey, bo qua discover ---");
		return;
	}

	/* Lay group address tu publish address (cau hinh boi provisioner) */
	uint16_t group_addr = BT_MESH_ADDR_UNASSIGNED;
	if (vnd_srv_model->pub && vnd_srv_model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		group_addr = vnd_srv_model->pub->addr;
	}
	if (group_addr == BT_MESH_ADDR_UNASSIGNED) {
		/* Fallback: doc tu subscription list */
		for (int i = 0; i < CONFIG_BT_MESH_MODEL_GROUP_COUNT; i++) {
			if (vnd_srv_model->groups[i] != BT_MESH_ADDR_UNASSIGNED) {
				group_addr = vnd_srv_model->groups[i];
				break;
			}
		}
	}
	if (group_addr == BT_MESH_ADDR_UNASSIGNED) {
		LOG_WRN("--- [GRADIENT]: Chua co group address, bo qua discover ---");
		return;
	}

	/* Tao goi GRADIENT_DISCOVER */
	uint8_t msg_data[16];
	struct net_buf_simple msg;
	net_buf_simple_init_with_data(&msg, msg_data, sizeof(msg_data));
	bt_mesh_model_msg_init(&msg, VND_OP_GRADIENT_DISCOVER);
	net_buf_simple_add_le16(&msg, bt_mesh_primary_addr()); /* origin = gateway */
	net_buf_simple_add_u8(&msg, 0);                        /* hop_count = 0 */
	net_buf_simple_add_le16(&msg, 0);                      /* energy_cost = 0 */
	net_buf_simple_add_u8(&msg, gradient_sequence);        /* sequence */

	/* Gui voi TTL=0 de chi node trong tam nhan truc tiep nhan duoc */
	struct bt_mesh_msg_ctx ctx = {
		.addr     = group_addr,
		.net_idx  = app_idx, /* Gia dinh net_idx == app_idx */
		.app_idx  = app_idx,
		.send_ttl = 0,       /* Khong relay - chi direct neighbors */
	};

	int err = bt_mesh_model_send(vnd_srv_model, &ctx, &msg, NULL, NULL);
	if (err) {
		LOG_ERR("--- [GRADIENT]: Loi gui discover (err %d) ---", err);
	} else {
		LOG_INF("--- [GRADIENT]: Broadcast DISCOVER seq=%u hop=0 cost=0 -> 0x%04X (net=%u, app=%u) ---",
			gradient_sequence, group_addr, app_idx, app_idx);
	}
}

/* ========================================================================= */
/* --- TIMER HANDLERS ---                                                    */
/* ========================================================================= */

/* Gui burst gradient discover (5 lan, cach nhau 3s) */
static void gradient_discover_handler(struct k_work *work)
{
	send_gradient_discover();
	discover_burst_count++;
	if (discover_burst_count < GRADIENT_DISCOVER_BURST_COUNT) {
		k_work_reschedule(&gradient_discover_work, K_MSEC(GRADIENT_DISCOVER_BURST_INTERVAL));
	}
}

/* Tai kham pha gradient dinh ky */
static void gradient_rediscover_handler(struct k_work *work)
{
	gradient_sequence++;
	discover_burst_count = 0;
	LOG_INF("--- [GRADIENT]: === BAT DAU TAI KHAM PHA, seq=%u === ---", gradient_sequence);
	k_work_reschedule(&gradient_discover_work, K_NO_WAIT);
	k_work_reschedule(&gradient_rediscover_work, K_MSEC(GRADIENT_REDISCOVER_INTERVAL));
}

/* Tu dong subscribe group address */
static void init_subscription_handler(struct k_work *work)
{
	/* Tu dong gan dia chi publish/subscribe cua Vendor Model ve group 0xC000 */
	if (vnd_srv_model && vnd_srv_model->groups && vnd_srv_model->pub) {
		vnd_srv_model->pub->addr = 0xC000;
		if (vnd_srv_model->groups[0] != 0xC000) {
			vnd_srv_model->groups[0] = 0xC000;
			LOG_INF("--- [SYSTEM]: Gateway tu dong gan Vendor model den Group: 0xC000 ---");
		}
	}

	/* Tu dong gan dia chi publish/subscribe cua Sensor Client ve group 0xC000 */
	if (sensor_cli_model && sensor_cli_model->groups && sensor_cli_model->pub) {
		sensor_cli_model->pub->addr = 0xC000;
		if (sensor_cli_model->groups[0] != 0xC000) {
			sensor_cli_model->groups[0] = 0xC000;
			LOG_INF("--- [SYSTEM]: Gateway tu dong gan Sensor Client den Group: 0xC000 ---");
		}
	}

	/* OnOff Server cua LED1 (Index 1) se subscribe group 0xC002 de nhan tin hieu dieu khien */
	struct bt_mesh_model *onoff_srv_model = NULL;
	if (ELEMENTS_SIZE > 1 && p_elements) {
		onoff_srv_model = &p_elements[1].models[0];
	}
	if (onoff_srv_model && onoff_srv_model->groups) {
		if (onoff_srv_model->groups[0] != 0xC002) {
			onoff_srv_model->groups[0] = 0xC002;
			LOG_INF("--- [SYSTEM]: Gateway tu dong subscribe ONOFF Server den Group: 0xC002 ---");
		}
	}

	/* Kiem tra xem da san sang chua */
	bool has_key = false;
	for (int i = 0; i < CONFIG_BT_MESH_MODEL_KEY_COUNT; i++) {
		if (vnd_srv_model && vnd_srv_model->keys[i] != BT_MESH_KEY_UNUSED) {
			has_key = true;
			break;
		}
	}

	if (!has_key) {
		k_work_reschedule(&init_subscription_work, K_MSEC(5000));
	}
}

static int handle_sensor_status(const struct bt_mesh_model *model,
				struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{
	while (buf->len >= 3) {
		uint8_t header = net_buf_simple_pull_u8(buf);
		uint16_t prop_id = net_buf_simple_pull_le16(buf);

		bool is_format_b = (header & 0x01) != 0;
		uint8_t length = 0;

		if (is_format_b) {
			length = (header >> 1) + 1;
		} else {
			length = ((header >> 1) & 0x07) + 1;
		}

		if (buf->len < length) {
			break;
		}

		uint8_t *val_ptr = net_buf_simple_pull_mem(buf, length);

		if (prop_id == PROP_ID_SUBNET_REPORT && length == sizeof(struct subnet_report_val_t)) {
			struct subnet_report_val_t *report = (struct subnet_report_val_t *)val_ptr;
			uint16_t origin = report->origin;
			int16_t sensor_val = report->sensor_value;
			uint8_t battery = report->battery_level;

			if (current_cycle_seq == 0) {
				current_cycle_seq = 1;
			}

			LOG_INF("GW_PACKET seq=%u origin=0x%04X sender=0x%04X value=%d pin=%u%%",
				current_cycle_seq, origin, ctx->addr, sensor_val, battery);

			update_and_print_sensor_list(origin, sensor_val, battery);
		}
	}

	return 0;
}

/* ========================================================================= */
/* --- LED / ONOFF / HEALTH SERVER (GIU NGUYEN) ---                          */
/* ========================================================================= */

static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp);

static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    struct bt_mesh_onoff_status *rsp);

static const struct bt_mesh_onoff_srv_handlers onoff_handlers = {
	.set = led_set,
	.get = led_get,
};

struct led_ctx {
	struct bt_mesh_onoff_srv srv;
	struct k_work_delayable work;
	uint32_t remaining;
	bool value;
};

static struct led_ctx led_ctx[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
};

static void led_transition_start(struct led_ctx *led)
{
	int led_idx = led - &led_ctx[0];
	dk_set_led(led_idx, true);
	k_work_reschedule(&led->work, K_MSEC(led->remaining));
	led->remaining = 0;
}

static void led_status(struct led_ctx *led, struct bt_mesh_onoff_status *status)
{
	status->remaining_time = led->remaining ? led->remaining :
		k_ticks_to_ms_ceil32(k_work_delayable_remaining_get(&led->work));
	status->target_on_off = led->value;
	status->present_on_off = led->value || status->remaining_time;
}

static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp)
{
	LOG_INF("Tin nhan nhan duoc: %d, tu source: 0x%04x", set->on_off, ctx->addr);
	struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);
	int led_idx = led - &led_ctx[0];

	if (set->on_off == led->value) {
		goto respond;
	}

	led->value = set->on_off;

	/* NetGroup Control: LED1 tuong ung voi chan trang thai thuc ngu/kich hoat he thong */
	if (led_idx == 1 && set->on_off == 1) {
		LOG_INF("--- [GATEWAY]: Kich hoat he thong tu dt (Group 0xC002), khoi chay gradient discovery ngay lap tuc! ---");
		gradient_sequence++;
		discover_burst_count = 0;
		k_work_reschedule(&gradient_discover_work, K_NO_WAIT);
		k_work_reschedule(&gradient_rediscover_work, K_MSEC(GRADIENT_REDISCOVER_INTERVAL));
	}

	if (!bt_mesh_model_transition_time(set->transition)) {
		led->remaining = 0;
		dk_set_led(led_idx, set->on_off);
		goto respond;
	}

	led->remaining = set->transition->time;

	if (set->transition->delay) {
		k_work_reschedule(&led->work, K_MSEC(set->transition->delay));
	} else {
		led_transition_start(led);
	}

respond:
	if (rsp) {
		led_status(led, rsp);
	}
}

static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    struct bt_mesh_onoff_status *rsp)
{
	struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);
	led_status(led, rsp);
}

static void led_work(struct k_work *work)
{
	struct led_ctx *led = CONTAINER_OF(work, struct led_ctx, work.work);
	int led_idx = led - &led_ctx[0];

	if (led->remaining) {
		led_transition_start(led);
	} else {
		dk_set_led(led_idx, led->value);
		struct bt_mesh_onoff_status status;
		led_status(led, &status);
		bt_mesh_onoff_srv_pub(&led->srv, NULL, &status);
	}
}

/* --- ATTENTION BLINK --- */
static struct k_work_delayable attention_blink_work;
static bool attention;

static void attention_blink(struct k_work *work)
{
	static int idx;
	const uint8_t pattern[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
		BIT(0),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
		BIT(1),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
		BIT(2),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
		BIT(3),
#endif
	};

	if (attention) {
		dk_set_leds(pattern[idx++ % ARRAY_SIZE(pattern)]);
		k_work_reschedule(&attention_blink_work, K_MSEC(30));
	} else {
		dk_set_leds(DK_NO_LEDS_MSK);
	}
}

static void attention_on(const struct bt_mesh_model *mod)
{
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(const struct bt_mesh_model *mod)
{
	attention = false;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/* ========================================================================= */
/* --- MESH COMPOSITION ---                                                  */
/* ========================================================================= */
static struct bt_mesh_elem elements[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
	BT_MESH_ELEM(
		1, BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_CFG_SRV,
			BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
			BT_MESH_MODEL_ONOFF_SRV(&led_ctx[0].srv),
			BT_MESH_MODEL_CB(BT_MESH_MODEL_ID_SENSOR_CLI, sensor_cli_ops, &sensor_cli_pub, NULL, NULL)
		),
		BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_VND(VND_COMPANY_ID, VND_MODEL_ID_SRV,
					   vnd_srv_ops, &vnd_pub, NULL)
		)),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
	BT_MESH_ELEM(
		3, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[1].srv)),
		BT_MESH_MODEL_NONE),
#endif
};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/* ========================================================================= */
/* --- KHOI TAO ---                                                          */
/* ========================================================================= */
const struct bt_mesh_comp *model_handler_init(void)
{
	p_elements = elements;
	k_work_init_delayable(&attention_blink_work, attention_blink);

	for (int i = 0; i < ARRAY_SIZE(led_ctx); ++i) {
		k_work_init_delayable(&led_ctx[i].work, led_work);
	}

	/* Khoi tao gradient discovery timers */
	k_work_init_delayable(&gradient_discover_work, gradient_discover_handler);
	k_work_init_delayable(&gradient_rediscover_work, gradient_rediscover_handler);
	k_work_init_delayable(&init_subscription_work, init_subscription_handler);

	/* Luu tham chieu den Vendor Model va Sensor Client Model */
	vnd_srv_model = &elements[0].vnd_models[0];
	sensor_cli_model = &elements[0].models[3];

	/* Len lich kham pha gradient som hon de san sang cho chu ky gui dau tien */
	gradient_sequence = 1;
	discover_burst_count = 0;
	k_work_reschedule(&gradient_discover_work, K_MSEC(3000));
	k_work_reschedule(&gradient_rediscover_work,
			  K_MSEC(GRADIENT_REDISCOVER_INTERVAL + 3000));

	/* Tu dong subscribe sau 2s */
	k_work_reschedule(&init_subscription_work, K_MSEC(2000));
	total_rx_count = 0;
	unique_rx_count = 0;
	current_cycle_rx_count = 0;
	current_cycle_seq = 1;

	return &comp;
}

void check_and_self_provision(void)
{
#if defined(CONFIG_BOARD_NRF52_BSIM)
	const char *addr_str = getenv("NODE_ADDR");
	if (addr_str != NULL) {
		uint16_t addr = (uint16_t)strtol(addr_str, NULL, 0);
		if (addr != 0) {
			LOG_INF("--- [SYSTEM]: BabbleSim detected. NODE_ADDR=%s (0x%04X) ---", addr_str, addr);
			static const uint8_t net_key[16] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0 };
			static const uint8_t dev_key[16] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00 };
			static const uint8_t app_key[16] = { 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99 };

			int err = bt_mesh_provision(net_key, 0, 0, 0, addr, dev_key);
			if (err && err != -EALREADY) {
				LOG_ERR("--- [SYSTEM]: Self-provisioning failed (err %d) ---", err);
			} else {
				LOG_INF("--- [SYSTEM]: Self-provisioning successful! ---");
				err = bt_mesh_app_key_add(0, 0, app_key);
				if (err && err != -EALREADY) {
					LOG_ERR("--- [SYSTEM]: Failed to add AppKey (err %d) ---", err);
				} else {
					LOG_INF("--- [SYSTEM]: AppKey 0 added successfully! ---");

					// Bind AppKey 0 to all models
					for (int i = 0; i < ARRAY_SIZE(elements); i++) {
						struct bt_mesh_elem *elem = &elements[i];
						for (int j = 0; j < elem->model_count; j++) {
							struct bt_mesh_model *model = &elem->models[j];
							if (model->id == BT_MESH_MODEL_ID_CFG_SRV || model->id == BT_MESH_MODEL_ID_HEALTH_SRV) {
								continue;
							}
							if (model->keys_cnt > 0) {
								model->keys[0] = 0;
							}
						}
						for (int j = 0; j < elem->vnd_model_count; j++) {
							struct bt_mesh_model *model = &elem->vnd_models[j];
							if (model->keys_cnt > 0) {
								model->keys[0] = 0;
							}
						}
					}
					LOG_INF("--- [SYSTEM]: Bound AppKey 0 to all models! ---");
				}
			}
		}
	}
#endif
}
