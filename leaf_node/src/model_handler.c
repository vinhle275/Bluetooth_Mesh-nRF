/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * LEAF NODE - Gradient Routing Sensor Node
 * =========================================
 * - Ngu/thuc theo chu ky (10s thuc / 10s ngu)
 * - Nhan GRADIENT_DISCOVER tu gateway/bridge de xay dung bang dinh tuyen
 * - Gui SENSOR_DATA den next_hop (gateway hoac bridge) theo gradient routing
 * - Gui GRADIENT_REPLY de thong bao trang thai ve gateway
 * - Reset bang dinh tuyen moi 10 chu ky de phong node trung gian bi loi
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/bluetooth/mesh/main.h>
#include <stdlib.h>

extern uint16_t bt_mesh_primary_addr(void);
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"

#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/drivers/adc.h>

LOG_MODULE_REGISTER(model_handler, CONFIG_LOG_DEFAULT_LEVEL);

#define ELEMENTS_SIZE (DT_NODE_EXISTS(DT_ALIAS(led0)) + DT_NODE_EXISTS(DT_ALIAS(led1)))
static struct bt_mesh_elem *p_elements;

/* ========================================================================= */
/* --- DINH NGHIA VENDOR MODEL VA OPCODE ---                                 */
/* ========================================================================= */
#define BT_MESH_MODEL_ID_SENSOR_SRV 0x1100
#define BT_MESH_MODEL_ID_SENSOR_CLI 0x1102
#define SENSOR_OP_STATUS            0x52
#define PROP_ID_SUBNET_REPORT       0xEEEE

struct subnet_report_val_t {
	uint16_t origin;
	uint16_t sensor_value;
	uint8_t  battery_level;
} __packed;

static struct k_work_delayable discover_rebroadcast_work;
static uint16_t pending_discover_origin;
static uint8_t pending_discover_hop;
static uint16_t pending_discover_cost;
static uint8_t pending_discover_seq;
static struct bt_mesh_msg_ctx pending_discover_ctx;
static const struct bt_mesh_model *pending_discover_model;

static void discover_rebroadcast_handler(struct k_work *work);
static int send_mesh_sensor_data(uint32_t sensor_value, uint8_t battery_level);
static void retry_sensor_publish(void);

struct gradient_discover_msg_t {
	uint16_t origin;
	uint8_t  hop_count;
	uint16_t energy_cost;
	uint8_t  sequence;
} __packed;

struct gradient_reply_msg_t {
	uint16_t origin;
	uint8_t  battery_level;
	uint8_t  hop_count;
} __packed;

/* --- HANG SO GRADIENT ROUTING --- */
#define HOP_PENALTY              10
#define GRADIENT_RESET_CYCLES    10

/* --- CHU KY GUI SENSOR --- */
#define SENSOR_AWAKE_DURATION_MS  22000
#define SENSOR_SLEEP_DURATION_MS  10000
#define SENSOR_TX_GUARD_MS         4500
#define SENSOR_TX_SLOT_WINDOW_MS  15000
#define SENSOR_TX_RETRY_DELAY_MS   700
#define SENSOR_TX_MAX_RETRIES      2

/* ========================================================================= */
/* --- ADC DOC MUC PIN ---                                                   */
/* ========================================================================= */
#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
#endif

static uint8_t read_battery_level(void)
{
#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	int16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		.buffer_size = sizeof(buf),
	};

	if (!adc_is_ready_dt(&adc_channel)) {
		LOG_ERR("ADC device not ready");
		return 2;
	}

	/* Luon goi setup lai cau hinh kenh truoc moi lan doc */
	/* Dam bao thanh ghi cau hinh khong bi mat sau khi thuc day tu che do ngu */
	int err = adc_channel_setup_dt(&adc_channel);
	if (err < 0) {
		LOG_ERR("ADC channel setup failed (%d)", err);
		return 6;
	}

	err = adc_sequence_init_dt(&adc_channel, &sequence);
	if (err < 0) {
		LOG_ERR("ADC sequence init failed (%d)", err);
		return 3;
	}

	err = adc_read_dt(&adc_channel, &sequence);
	if (err < 0) {
		LOG_ERR("ADC read failed (%d)", err);
		return 4;
	}

	int32_t val_mv = buf;
	int err_conv = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
	if (err_conv < 0) {
		val_mv = (int32_t)buf * 3600 / 4095;
	}

	/* Map 0.9V (900mV) -> 1.5V (1500mV) */
	int32_t percent = ((val_mv - 900) * 100) / (1500 - 900);
	if (percent > 100) {
		percent = 100;
	} else if (percent < 0) {
		percent = 0;
	}
	return (uint8_t)percent;
#else
	return 100;
#endif
}

/* ========================================================================= */
/* --- GRADIENT ROUTING STATE ---                                            */
/* ========================================================================= */
static uint16_t gradient_next_hop      = BT_MESH_ADDR_UNASSIGNED;
static uint16_t gradient_cost          = 0xFFFF;
static uint16_t gradient_route_net_idx = 0;
static uint16_t gradient_route_app_idx = 0;
static uint8_t  gradient_hop_count     = 0xFF;
static uint8_t  gradient_last_seq      = 0xFF; /* 0xFF = chua nhan discover nao */
static bool     gradient_route_valid   = false;

#define VND_COMPANY_ID            0x0059
#define VND_MODEL_ID_CLI          0x0001
#define VND_OP_GRADIENT_DISCOVER  BT_MESH_MODEL_OP_3(0x02, VND_COMPANY_ID)
#define VND_OP_GRADIENT_REPLY     BT_MESH_MODEL_OP_3(0x03, VND_COMPANY_ID)

static int handle_sensor_status_relay(const struct bt_mesh_model *model,
				      struct bt_mesh_msg_ctx *ctx,
				      struct net_buf_simple *buf);
static int handle_gradient_discover(const struct bt_mesh_model *model,
				    struct bt_mesh_msg_ctx *ctx,
				    struct net_buf_simple *buf);

static const struct bt_mesh_model *sensor_srv_model;
static const struct bt_mesh_model *sensor_cli_model;
static const struct bt_mesh_model *vnd_cli_model;

/* ========================================================================= */
/* --- MODEL OP ARRAYS ---                                                  */
/* ========================================================================= */
BT_MESH_MODEL_PUB_DEFINE(vnd_pub, NULL, 15);
BT_MESH_MODEL_PUB_DEFINE(sensor_srv_pub, NULL, 15);
BT_MESH_MODEL_PUB_DEFINE(sensor_cli_pub, NULL, 15);

static const struct bt_mesh_model_op vnd_cli_ops[] = {
	{ VND_OP_GRADIENT_DISCOVER, sizeof(struct gradient_discover_msg_t), handle_gradient_discover },
	BT_MESH_MODEL_OP_END,
};

static const struct bt_mesh_model_op sensor_srv_ops[] = {
	BT_MESH_MODEL_OP_END,
};

static const struct bt_mesh_model_op sensor_cli_ops[] = {
	{ SENSOR_OP_STATUS, 3, handle_sensor_status_relay },
	BT_MESH_MODEL_OP_END,
};

/* ========================================================================= */
/* --- XU LY GRADIENT DISCOVER ---                                           */
/* ========================================================================= */
static int handle_gradient_discover(const struct bt_mesh_model *model,
				    struct bt_mesh_msg_ctx *ctx,
				    struct net_buf_simple *buf)
{
	if (buf->len < sizeof(struct gradient_discover_msg_t)) {
		return -EINVAL;
	}

	uint16_t discover_origin = net_buf_simple_pull_le16(buf);
	uint8_t  hop_count       = net_buf_simple_pull_u8(buf);
	uint16_t energy_cost     = net_buf_simple_pull_le16(buf);
	uint8_t  sequence        = net_buf_simple_pull_u8(buf);

	/* Tinh chi phi tong cho duong di qua node nay */
	uint8_t my_battery = read_battery_level();
	uint16_t my_cost = energy_cost + (100 - my_battery) + HOP_PENALTY;

	LOG_INF("--- [GRADIENT DISCOVER]: tu sender=0x%04X, origin=0x%04X, seq=%u, hop=%u, cost_recv=%u, my_cost=%u ---",
		ctx->addr, discover_origin, sequence, hop_count, energy_cost, my_cost);

	/* Kiem tra sequence: neu khac sequence cu -> vong kham pha moi */
	if (sequence != gradient_last_seq) {
		gradient_last_seq = sequence;
		gradient_cost = 0xFFFF; /* Reset best cost cho vong moi */
	}

	/* 🟢 Bỏ qua nếu đường này đi xa hơn (chi phí cao hơn cost hiện tại) */
	if (my_cost > gradient_cost) {
		LOG_INF("--- [GRADIENT]: Bo qua - cost hien tai (%u) tot hon cost moi (%u) ---",
			gradient_cost, my_cost);
		return 0;
	}

	bool should_rebroadcast = false;

	/* 🟢 Cập nhật đường đi khi tìm thấy đường tối ưu hơn (cost nhỏ hơn) */
	if (my_cost < gradient_cost) {
		gradient_next_hop      = ctx->addr;
		gradient_cost          = my_cost;
		gradient_route_net_idx = ctx->net_idx;
		gradient_route_app_idx = ctx->app_idx;
		gradient_hop_count     = hop_count + 1;
		gradient_route_valid   = true;
		should_rebroadcast     = true;

		LOG_INF("--- [GRADIENT]: === DA CAP NHAT ROUTE === next_hop=0x%04X, cost=%u, hops=%u, net=%u, app=%u ---",
			gradient_next_hop, gradient_cost, gradient_hop_count,
			gradient_route_net_idx, gradient_route_app_idx);
	}
	/* 🟢 Vẫn cho phép Rebroadcast các gói Burst nếu nó đến từ đúng Next Hop hiện tại */
	else if (my_cost == gradient_cost && ctx->addr == gradient_next_hop) {
		should_rebroadcast = true;
		LOG_INF("--- [GRADIENT]: Nhan goi Burst tu next_hop, Rebroadcast de danh thuc node con! ---");
	}

	if (!should_rebroadcast) {
		return 0;
	}

	/* Coalesce and delay discover rebroadcasts to prevent broadcast storms */
	pending_discover_origin = discover_origin;
	pending_discover_hop    = hop_count + 1;
	pending_discover_cost   = my_cost;
	pending_discover_seq    = sequence;
	pending_discover_ctx    = *ctx;
	pending_discover_model  = model;

	/* Dynamic backoff delay based on hop count and random jitter.
	 * In large grids, many nodes hear the same discovery at once. A wider,
	 * hop-dependent window prevents discover rebroadcast storms while keeping
	 * the route ready before the first sensor transmit slot.
	 */
	uint32_t delay_ms = 80 + (hop_count * 60) + (rand() % 260);
	k_work_reschedule(&discover_rebroadcast_work, K_MSEC(delay_ms));

	LOG_INF("--- [GRADIENT]: Scheduled delayed discovery rebroadcast in %u ms ---", delay_ms);

	return 0;
}


static int handle_sensor_status_relay(const struct bt_mesh_model *model,
				      struct bt_mesh_msg_ctx *ctx,
				      struct net_buf_simple *buf)
{
	uint16_t my_addr = bt_mesh_primary_addr();

	/* Chi relay neu goi tin gui cho minh lam next hop */
	if (ctx->recv_dst != my_addr && !BT_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
		return 0;
	}

	uint8_t relay_data[64];
	if (buf->len > sizeof(relay_data)) {
		return 0;
	}
	uint16_t data_len = buf->len;
	memcpy(relay_data, buf->data, data_len);

	/* Check loop */
	struct net_buf_simple temp_buf;
	net_buf_simple_init_with_data(&temp_buf, relay_data, data_len);

	if (temp_buf.len >= 3) {
		(void)net_buf_simple_pull_u8(&temp_buf);
		uint16_t prop_id = net_buf_simple_pull_le16(&temp_buf);

		if (prop_id == PROP_ID_SUBNET_REPORT && temp_buf.len >= sizeof(struct subnet_report_val_t)) {
			struct subnet_report_val_t *report = (struct subnet_report_val_t *)temp_buf.data;
			if (report->origin == my_addr) {
				return 0;
			}
		}
	}

	if (!gradient_route_valid || gradient_next_hop == BT_MESH_ADDR_UNASSIGNED) {
		LOG_WRN("--- [SENSOR RELAY]: Leaf chua co duong di gradient, bo qua relay ---");
		return 0;
	}

	if (gradient_next_hop == my_addr) {
		return 0;
	}

	/* Forward data */
	uint8_t msg_data[64];
	struct net_buf_simple msg;
	net_buf_simple_init_with_data(&msg, msg_data, sizeof(msg_data));
	bt_mesh_model_msg_init(&msg, SENSOR_OP_STATUS);
	net_buf_simple_add_mem(&msg, relay_data, data_len);

	struct bt_mesh_msg_ctx relay_ctx = {
		.addr     = gradient_next_hop,
		.net_idx  = gradient_route_net_idx,
		.app_idx  = gradient_route_app_idx,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	/* Add random sleep to desynchronize multi-hop relays and prevent congestion
	 * when many children forward through the same parent.
	 */
	k_sleep(K_MSEC(60 + (rand() % 220)));

	int err = bt_mesh_model_send(model, &relay_ctx, &msg, NULL, NULL);
	if (err) {
		LOG_ERR("--- [SENSOR RELAY]: Leaf loi forward (err %d) ---", err);
	} else {
		LOG_INF("--- [SENSOR RELAY]: Leaf forward -> next_hop=0x%04X ---", gradient_next_hop);
	}

	return 0;
}

static int send_mesh_sensor_data(uint32_t sensor_value, uint8_t battery_level)
{
	if (!sensor_srv_model) {
		LOG_WRN("--- [PHAT SONG]: Model chua khoi tao ---");
		return -EINVAL;
	}

	if (!gradient_route_valid || gradient_next_hop == BT_MESH_ADDR_UNASSIGNED) {
		LOG_WRN("--- [PHAT SONG]: Chua co duong di gradient, cho discover... ---");
		return -EAGAIN;
	}

	/* Tao goi SENSOR_STATUS (Format B) */
	uint8_t msg_data[16];
	struct net_buf_simple msg;
	net_buf_simple_init_with_data(&msg, msg_data, sizeof(msg_data));
	bt_mesh_model_msg_init(&msg, SENSOR_OP_STATUS);

	/* Add Format B Header */
	net_buf_simple_add_u8(&msg, 0x09);              /* Length=4 (5 bytes thuc te) */
	net_buf_simple_add_le16(&msg, PROP_ID_SUBNET_REPORT);

	net_buf_simple_add_le16(&msg, bt_mesh_primary_addr()); /* Origin = my addr */
	net_buf_simple_add_le16(&msg, (uint16_t)sensor_value);
	net_buf_simple_add_u8(&msg, battery_level);

	struct bt_mesh_msg_ctx ctx = {
		.addr     = gradient_next_hop,
		.net_idx  = gradient_route_net_idx,
		.app_idx  = gradient_route_app_idx,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	int err = bt_mesh_model_send(sensor_srv_model, &ctx, &msg, NULL, NULL);
	if (err) {
		LOG_ERR("--- [PHAT SONG]: Loi gui sensor data (err %d) ---", err);
		return err;
	} else {
		LOG_INF("--- [PHAT SONG]: SENSOR=%u PIN=%u%% -> next_hop=0x%04X (net=%u, app=%u) ---",
			sensor_value, battery_level, gradient_next_hop,
			gradient_route_net_idx, gradient_route_app_idx);
		return 0;
	}
}

/* ========================================================================= */
/* --- LED / ONOFF INFRASTRUCTURE ---                                        */
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

#define LED_ON   true
#define LED_OFF  false

static void led_transition_start(struct led_ctx *led)
{
	int led_idx = led - &led_ctx[0];
	dk_set_led(led_idx, LED_ON);
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

/* ========================================================================= */
/* --- CHU KY NGU/THUC VA TRUYEN DU LIEU ---                                 */
/* ========================================================================= */
static struct k_work_delayable cycle_work;
static struct k_work_delayable suspend_work;
static struct k_work_delayable blink_work;
static struct k_work_delayable publish_work;
static struct k_work_delayable init_subscription_work;
// Discover rebroadcast declarations moved to top of file


static bool cycle_mode_active = false;
static bool is_awake = true;
static bool led2_blink_state = false;
static bool next_state_on = true;

static struct bt_mesh_onoff_cli onoff_cli;

static uint32_t simulated_sensor_value = 0;
static uint32_t wake_cycle_count = 0;
static uint8_t tx_retry_count = 0;

static uint32_t compute_tx_delay_ms(void)
{
#if defined(CONFIG_BOARD_NRF52_BSIM)
	const char *addr_str = getenv("NODE_ADDR");
	const char *node_index_str = getenv("NODE_INDEX");
	const char *num_nodes_str = getenv("NUM_NODES");
	uint32_t addr = addr_str ? (uint32_t)strtoul(addr_str, NULL, 0) : 0;
	uint32_t node_index = node_index_str ? (uint32_t)strtoul(node_index_str, NULL, 0) : 0;
	uint32_t num_nodes = num_nodes_str ? (uint32_t)strtoul(num_nodes_str, NULL, 0) : 9U;
	if (num_nodes < 2U) {
		num_nodes = 2U;
	}

	/* Phan tan goi theo index + chu ky de giam collision trong luoi lon */
	uint32_t active_nodes = num_nodes - 1U;
	uint32_t slot_gap = SENSOR_TX_SLOT_WINDOW_MS / active_nodes;
	if (slot_gap < 55U) {
		slot_gap = 55U;
	}

	/* NODE_INDEX 1..N-1 are physical leaves. Do not derive slot from NODE_ADDR:
	 * the Python runner allocates primary addresses with stride 2 because each
	 * composition has two elements. Using NODE_ADDR directly would alias slots.
	 * Rotate slot order each wake cycle so the same parent is not always hit by
	 * the same local burst pattern.
	 */
	uint32_t leaf_index = (node_index > 0U) ? (node_index - 1U) : ((addr > 1U) ? (addr - 2U) : 0U);
	uint32_t slot_index = (leaf_index + (wake_cycle_count * 17U)) % active_nodes;
	uint32_t cycle_jitter = (wake_cycle_count % 5U) * 13U;
	return SENSOR_TX_GUARD_MS + (slot_index * slot_gap) + cycle_jitter;
#else
	return SENSOR_TX_GUARD_MS;
#endif
}

static void display_sensor_value(void)
{
	uint8_t mod_val = simulated_sensor_value % 4;
	bool bit0 = (mod_val & 0x01) != 0;
	bool bit1 = (mod_val & 0x02) != 0;
	dk_set_led(0, bit0 ? LED_ON : LED_OFF);
	dk_set_led(1, bit1 ? LED_ON : LED_OFF);
}

/* Nhay LED khi thuc (Da vo hieu hoa de tiet kiem nang luong) */
static void blink_handler(struct k_work *work)
{
	/* Khong lam gi de CPU khong bi thuc giac lien tuc */
}

/* Gui goi tin sensor dinh ky khi thuc */
static void publish_handler(struct k_work *work)
{
	if (!cycle_mode_active || !is_awake) {
		return;
	}

	if (send_mesh_sensor_data(simulated_sensor_value, read_battery_level()) != 0) {
		retry_sensor_publish();
	} else {
		tx_retry_count = 0;
	}
}

static void schedule_sensor_publish(void)
{
	uint32_t delay_ms = compute_tx_delay_ms();
	k_work_cancel_delayable(&publish_work);
	k_work_reschedule(&publish_work, K_MSEC(delay_ms));
}

static void retry_sensor_publish(void)
{
	if (tx_retry_count >= SENSOR_TX_MAX_RETRIES) {
		tx_retry_count = 0;
		return;
	}
	tx_retry_count++;
	k_work_reschedule(&publish_work, K_MSEC(SENSOR_TX_RETRY_DELAY_MS));
}

static void discover_rebroadcast_handler(struct k_work *work)
{
	uint16_t group_addr = BT_MESH_ADDR_UNASSIGNED;
	const struct bt_mesh_model *model = pending_discover_model;

	if (!model) {
		return;
	}

	if (model->pub && model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		group_addr = model->pub->addr;
	}
	if (group_addr == BT_MESH_ADDR_UNASSIGNED) {
		for (int i = 0; i < CONFIG_BT_MESH_MODEL_GROUP_COUNT; i++) {
			if (model->groups[i] != BT_MESH_ADDR_UNASSIGNED) {
				group_addr = model->groups[i];
				break;
			}
		}
	}
	if (group_addr == BT_MESH_ADDR_UNASSIGNED) {
		LOG_WRN("--- [GRADIENT]: Delay rebroadcast khong co group address ---");
		return;
	}

	uint8_t msg_data[16];
	struct net_buf_simple msg;
	net_buf_simple_init_with_data(&msg, msg_data, sizeof(msg_data));
	bt_mesh_model_msg_init(&msg, VND_OP_GRADIENT_DISCOVER);
	net_buf_simple_add_le16(&msg, pending_discover_origin);
	net_buf_simple_add_u8(&msg, pending_discover_hop);
	net_buf_simple_add_le16(&msg, pending_discover_cost);
	net_buf_simple_add_u8(&msg, pending_discover_seq);

	struct bt_mesh_msg_ctx rebroadcast_ctx = {
		.addr     = group_addr,
		.net_idx  = pending_discover_ctx.net_idx,
		.app_idx  = pending_discover_ctx.app_idx,
		.send_ttl = 0, /* Chi truyen toi neighbors truc tiep */
	};

	int err = bt_mesh_model_send(model, &rebroadcast_ctx, &msg, NULL, NULL);
	if (err) {
		LOG_ERR("--- [GRADIENT]: Delayed discover rebroadcast loi (err %d) ---", err);
	} else {
		LOG_INF("--- [GRADIENT]: Delayed Rebroadcast DISCOVER -> 0x%04X, hop=%u, cost=%u ---",
			group_addr, pending_discover_hop, pending_discover_cost);
	}
}


/* Tat radio (ngu sau de tiet kiem pin) */
static void suspend_handler(struct k_work *work)
{
	if (is_awake || !cycle_mode_active) {
		return;
	}

	k_work_cancel_delayable(&blink_work);

	for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
		dk_set_led(i, LED_OFF);
	}

#if !defined(CONFIG_BOARD_NRF52_BSIM)
	bt_mesh_suspend();
#endif
	LOG_INF("--- [RADIO]: Da dong bang Radio, TAT HET LED, ngat mach di ngu! ---");
}


/* Tu dong subscribe group address */
static void init_subscription_handler(struct k_work *work)
{
	/* Vendor model bind den Group 0xC000 de nhan discover */
	if (vnd_cli_model && vnd_cli_model->groups && vnd_cli_model->pub) {
		vnd_cli_model->pub->addr = 0xC000;
		if (vnd_cli_model->groups[0] != 0xC000) {
			vnd_cli_model->groups[0] = 0xC000;
			LOG_INF("--- [SYSTEM]: Leaf tu dong subscribe Vendor model den Group: 0xC000 ---");
		}
	}

	/* Sensor Server bind den Group 0xC000 */
	if (sensor_srv_model && sensor_srv_model->groups && sensor_srv_model->pub) {
		sensor_srv_model->pub->addr = 0xC000;
		if (sensor_srv_model->groups[0] != 0xC000) {
			sensor_srv_model->groups[0] = 0xC000;
			LOG_INF("--- [SYSTEM]: Leaf tu dong subscribe Sensor Server den Group: 0xC000 ---");
		}
	}

	/* Sensor Client bind den Group 0xC000 */
	if (sensor_cli_model && sensor_cli_model->groups && sensor_cli_model->pub) {
		sensor_cli_model->pub->addr = 0xC000;
		if (sensor_cli_model->groups[0] != 0xC000) {
			sensor_cli_model->groups[0] = 0xC000;
			LOG_INF("--- [SYSTEM]: Leaf tu dong subscribe Sensor Client den Group: 0xC000 ---");
		}
	}
	/* OnOff Server (Element 3 tương ứng LED1) bind den Group 0xC002 de nhan lenh thuc ngu */
	const struct bt_mesh_model *onoff_srv_model = NULL;
	if (ELEMENTS_SIZE > 1 && p_elements) {
		onoff_srv_model = &p_elements[1].models[0];
	}
	if (onoff_srv_model && onoff_srv_model->groups) {
		if (onoff_srv_model->groups[0] != 0xC002) {
			onoff_srv_model->groups[0] = 0xC002;
			LOG_INF("--- [SYSTEM]: Leaf tu dong subscribe ONOFF Server den Group: 0xC002 ---");
		}
	}

	/* Kiem tra xem da san sang chua (da bind key va provision xong) */
	bool has_key = false;
	for (int i = 0; i < CONFIG_BT_MESH_MODEL_KEY_COUNT; i++) {
		if (vnd_cli_model && vnd_cli_model->keys[i] != BT_MESH_KEY_UNUSED) {
			has_key = true;
			break;
		}
	}

	if (!has_key) {
		k_work_reschedule(&init_subscription_work, K_MSEC(5000));
	} else {
		LOG_INF("--- [SYSTEM]: Leaf da co khoa key, ngung thuc day quet subscribe! ---");
	}
}

/* ========================================================================= */
/* --- QUAN LY CHU KY NGUON THUC/NGU ---                                     */
/* ========================================================================= */
static void cycle_handler(struct k_work *work)
{
	if (!cycle_mode_active) {
		return;
	}

	is_awake = !is_awake;

	if (is_awake) {
		/* --- MACH THUC GIAC (10s) --- */
#if !defined(CONFIG_BOARD_NRF52_BSIM)
		bt_mesh_resume();
#endif
		LOG_INF("--- [RADIO]: Da khoi dong lai Radio! ---");


		wake_cycle_count++;
		LOG_INF("--- [CYCLE]: Chu ky thuc #%u ---", wake_cycle_count);

		/* Kiem tra co can reset gradient routing khong */
		// if (wake_cycle_count >= GRADIENT_RESET_CYCLES) {
		// 	gradient_next_hop    = BT_MESH_ADDR_UNASSIGNED;
		// 	gradient_cost        = 0xFFFF;
		// 	gradient_route_valid = false;
		// 	wake_cycle_count     = 0;
		// 	LOG_INF("--- [GRADIENT]: === RESET BANG DINH TUYEN, cho discover moi === ---");
		// }

		for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
			if (i != 1) {
				dk_set_led(i, led_ctx[i].value ? LED_ON : LED_OFF);
			}
		}

		led2_blink_state = true;
		dk_set_led(1, LED_ON);
		// k_work_reschedule(&blink_work, K_MSEC(100));

		next_state_on = true;

		/* Tat ca node deu se gui trong cung mot chu ky wakeup. Awake window phai
		 * dai hon guard + slot_window de 13x13 (168 leaf) khong bi lo slot.
		 */
		tx_retry_count = 0;
		schedule_sensor_publish();
		k_work_reschedule(&cycle_work, K_MSEC(SENSOR_AWAKE_DURATION_MS));

		simulated_sensor_value += 1;
		LOG_INF("--- [SENSOR]: Gia tri sensor hien tai = %d ---", simulated_sensor_value);
		display_sensor_value();

	} else {
		/* --- MACH SAP DI NGU (10s) --- */
		k_work_cancel_delayable(&publish_work);
		k_work_cancel_delayable(&blink_work);

		/* Tắt tất cả các LED ngay lập tức để tiết kiệm điện năng trong thời gian chờ 1.5s */
		for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
			dk_set_led(i, LED_OFF);
		}

		/* Cho tre 1.5s de cac goi tin truyen xong truoc khi tat radio */
		k_work_reschedule(&suspend_work, K_MSEC(1500));
		k_work_reschedule(&cycle_work, K_MSEC(SENSOR_SLEEP_DURATION_MS));
	}
}

/* ========================================================================= */
/* --- LED SET/GET (DIEU KHIEN CHU KY) ---                                   */
/* ========================================================================= */
static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp)
{
	struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);
	int led_idx = led - &led_ctx[0];

	if (led_idx == 1) {
		/* LED1 dieu khien che do chu ky */
		led->value = set->on_off;

		if (set->on_off == 1) {
			if (!cycle_mode_active) {
				cycle_mode_active = true;
				is_awake = true;
				wake_cycle_count = 0;

				// k_work_reschedule(&blink_work, K_NO_WAIT);
				k_work_reschedule(&cycle_work, K_MSEC(SENSOR_AWAKE_DURATION_MS));

				next_state_on = true;
				k_work_reschedule(&publish_work, K_MSEC(500));

				LOG_INF("--- [SENSOR]: Bat dau chu ky, gia tri sensor = %d ---",
					simulated_sensor_value);
				display_sensor_value();
			}
		} else {
			cycle_mode_active = false;
			is_awake = true;

			k_work_cancel_delayable(&cycle_work);
			k_work_cancel_delayable(&blink_work);
			k_work_cancel_delayable(&publish_work);
			k_work_cancel_delayable(&suspend_work);

			bt_mesh_resume();
			dk_set_led(1, LED_ON);

			for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
				if (i != 1) {
					dk_set_led(i, led_ctx[i].value ? LED_ON : LED_OFF);
				}
			}
		}
	} else {
		led->value = set->on_off;
		if (!bt_mesh_model_transition_time(set->transition)) {
			led->remaining = 0;
			if (is_awake) {
				dk_set_led(led_idx, set->on_off ? LED_ON : LED_OFF);
			}
		} else {
			led->remaining = set->transition->time;
			if (set->transition->delay) {
				k_work_reschedule(&led->work, K_MSEC(set->transition->delay));
			} else {
				if (is_awake) {
					led_transition_start(led);
				}
			}
		}
	}

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
		if (is_awake && led_idx != 1) {
			dk_set_led(led_idx, led->value ? LED_ON : LED_OFF);
		}
		struct bt_mesh_onoff_status status;
		led_status(led, &status);
		bt_mesh_onoff_srv_pub(&led->srv, NULL, &status);
	}
}

/* ========================================================================= */
/* --- ATTENTION / HEALTH SERVER ---                                         */
/* ========================================================================= */
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
		for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
			dk_set_led(i, LED_OFF);
		}
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
			BT_MESH_MODEL_ONOFF_CLI(&onoff_cli),
			BT_MESH_MODEL_CB(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_srv_ops, &sensor_srv_pub, NULL, NULL),
			BT_MESH_MODEL_CB(BT_MESH_MODEL_ID_SENSOR_CLI, sensor_cli_ops, &sensor_cli_pub, NULL, NULL)
		),
		BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_VND(VND_COMPANY_ID, VND_MODEL_ID_CLI,
					   vnd_cli_ops, &vnd_pub, NULL)
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
		led_ctx[i].value = 0;
	}

	k_work_init_delayable(&blink_work, blink_handler);
	k_work_init_delayable(&cycle_work, cycle_handler);
	k_work_init_delayable(&publish_work, publish_handler);
	k_work_init_delayable(&suspend_work, suspend_handler);
	k_work_init_delayable(&init_subscription_work, init_subscription_handler);
	k_work_init_delayable(&discover_rebroadcast_work, discover_rebroadcast_handler);
	k_work_reschedule(&init_subscription_work, K_MSEC(2000));

	vnd_cli_model = &elements[0].vnd_models[0];
	sensor_srv_model = &elements[0].models[4];
	sensor_cli_model = &elements[0].models[5];

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	if (adc_is_ready_dt(&adc_channel)) {
		int err = adc_channel_setup_dt(&adc_channel);
		if (err < 0) {
			LOG_ERR("ADC channel setup failed (%d)", err);
		}
	} else {
		LOG_ERR("ADC device not ready");
	}
#endif

#if defined(CONFIG_BOARD_NRF52_BSIM)
	cycle_mode_active = true;
	is_awake = true;
	wake_cycle_count = 0;
	next_state_on = true;
	k_work_reschedule(&cycle_work, K_MSEC(SENSOR_AWAKE_DURATION_MS));
	tx_retry_count = 0;
	schedule_sensor_publish();
#else
	cycle_mode_active = false;
	is_awake = true;
	wake_cycle_count = 0;
#endif


	/* Khoi tao gradient routing */
	gradient_next_hop    = BT_MESH_ADDR_UNASSIGNED;
	gradient_cost        = 0xFFFF;
	gradient_route_valid = false;
	gradient_last_seq    = 0xFF;

	for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
		dk_set_led(i, LED_OFF);
	}
	dk_set_led(1, LED_ON);

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
