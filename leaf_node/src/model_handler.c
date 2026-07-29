/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h> 
#include <zephyr/bluetooth/mesh/main.h>

extern uint16_t bt_mesh_primary_addr(void);
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h> 

LOG_MODULE_REGISTER(model_handler, CONFIG_LOG_DEFAULT_LEVEL);

/* ========================================================================= */
/* --- PHẦN ĐỊNH NGHĨA VÀ XỬ LÝ VENDOR MODEL GỬI SỐ NGUYÊN 32-BIT --- */
#define VND_COMPANY_ID   0x0059   // Mã công ty Nordic
#define VND_MODEL_ID_CLI 0x0001   // ID Model Gửi cho mạch Sensor
#define VND_OP_SENSOR_DATA BT_MESH_MODEL_OP_3(0x01, VND_COMPANY_ID)
#define VND_OP_RREQ        BT_MESH_MODEL_OP_3(0x0A, VND_COMPANY_ID)
#define VND_OP_RREP        BT_MESH_MODEL_OP_3(0x0B, VND_COMPANY_ID)

struct rreq_msg_t {
	uint16_t origin;
	uint8_t seq;
	uint8_t hop_count;
} __packed;

struct rrep_msg_t {
	uint16_t origin;
	uint16_t gateway;
	uint16_t next_hop;
	uint8_t hop_count;
} __packed;

struct sensor_data_msg_t {
	uint16_t origin;
	uint16_t next_hop;
	uint32_t sensor_value;
} __packed;

static bool route_discovery_in_progress = false;
static uint16_t route_next_hop = BT_MESH_ADDR_UNASSIGNED;
static bool path_discovered = false;
static uint8_t rreq_seq = 0;
static uint8_t best_hop_count = 0xFF;

struct reverse_route_t {
	uint16_t origin;
	uint16_t reverse_hop;
	uint8_t seq;
};
static struct reverse_route_t rev_route = {0};

static uint16_t relay_for_client = BT_MESH_ADDR_UNASSIGNED;

static int handle_rreq(const struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf);

static int handle_rrep(const struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf);

static int handle_sensor_data_relay(const struct bt_mesh_model *model,
				    struct bt_mesh_msg_ctx *ctx,
				    struct net_buf_simple *buf);

BT_MESH_MODEL_PUB_DEFINE(vnd_pub, NULL, 15);
static const struct bt_mesh_model *vnd_cli_model;
static const struct bt_mesh_model_op vnd_cli_ops[] = {
	{ VND_OP_RREQ, sizeof(struct rreq_msg_t), handle_rreq },
	{ VND_OP_RREP, sizeof(struct rrep_msg_t), handle_rrep },
	{ VND_OP_SENSOR_DATA, sizeof(struct sensor_data_msg_t), handle_sensor_data_relay },
	BT_MESH_MODEL_OP_END,
};

/* HÀM PHÁT SÓNG SENSOR ĐỊNH KỲ */
static void send_mesh_sensor_data_32bit(uint32_t sensor_value)
{
	if (!vnd_cli_model || route_next_hop == BT_MESH_ADDR_UNASSIGNED) {
		LOG_WRN("--- [PHAT SONG]: Chua thiet lap duong truyen hoac chua cau hinh! ---");
		return;
	}

	struct net_buf_simple *msg = vnd_cli_model->pub->msg;
	net_buf_simple_reset(msg);
	
	bt_mesh_model_msg_init(msg, VND_OP_SENSOR_DATA);
	net_buf_simple_add_le16(msg, bt_mesh_primary_addr()); // origin address
	net_buf_simple_add_le16(msg, route_next_hop);        // next hop address
	net_buf_simple_add_le32(msg, sensor_value); 

	// Cau hinh tu dong gui lai: 1 lan (tong cong 2 lan gui), cach nhau 50ms de tang do tin cay
	vnd_cli_model->pub->retransmit = BT_MESH_PUB_TRANSMIT(1, 50);

	int err = bt_mesh_model_publish(vnd_cli_model);
	if (err) {
		LOG_ERR("Loi gui tin Vendor (err %d)", err);
	} else {
		LOG_INF("--- [PHAT SONG VENDOR]: Da gui SENSOR = %u den Next Hop [0x%04X] ---", 
				sensor_value, route_next_hop);
	}
}
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

#define LED_ON   false
#define LED_OFF  true

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

/* ====================================================================
 * LOGIC CHU KỲ NGUỒN VÀ TRUYỀN DỮ LIỆU CÓ BỘ ĐỆM AN TOÀN
 * ==================================================================== */
static struct k_work_delayable cycle_work;   
static struct k_work_delayable suspend_work; 
static struct k_work_delayable blink_work;   
static struct k_work_delayable publish_work; 
static struct k_work_delayable discovery_timeout_work; 
static struct k_work_delayable init_subscription_work; 

static bool cycle_mode_active = false;      
static bool is_awake = true;                
static bool led2_blink_state = false;       
static bool next_state_on = true; 

static struct bt_mesh_onoff_cli onoff_cli; 

static uint32_t simulated_sensor_value = 0;

static void display_sensor_value(void)
{
	uint8_t mod_val = simulated_sensor_value % 4;

	bool bit0 = (mod_val & 0x01) != 0; 
	bool bit1 = (mod_val & 0x02) != 0; 

	dk_set_led(0, bit0 ? LED_ON : LED_OFF);
	dk_set_led(1, bit1 ? LED_ON : LED_OFF);
}

/* HÀM XỬ LÝ NHÁY LED 2: Chỉ nháy khi thức */
static void blink_handler(struct k_work *work)
{
	if (!cycle_mode_active || !is_awake) {
		return; 
	}

	led2_blink_state = !led2_blink_state;
	dk_set_led(2, led2_blink_state ? LED_ON : LED_OFF);
	
	k_work_reschedule(&blink_work, K_MSEC(100));
}

/* HÀM BẮN GÓI TIN CHU KỲ */
static void publish_handler(struct k_work *work)
{
	if (!cycle_mode_active || !is_awake) {
		return;
	}

	send_mesh_sensor_data_32bit(simulated_sensor_value);
	k_work_reschedule(&publish_work, K_MSEC(1000));
}

/* HÀM TẮT RADIO (NGỦ SÂU ĐỂ TIẾT KIỆM PIN) */
static void suspend_handler(struct k_work *work)
{
	if (is_awake || !cycle_mode_active) {
		return;
	}

	k_work_cancel_delayable(&blink_work);  

	for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
		dk_set_led(i, LED_OFF);
	}

	bt_mesh_suspend(); 
	LOG_INF("--- [RADIO]: Da dong bang Radio, TAT HET LED, ngat mach di ngu! ---");
}

/* KHỞI ĐỘNG TÌM ĐƯỜNG ĐỊNH TUYẾN DIRECTED FORWARDING */
static void start_path_discovery(void)
{
	int err;

	if (!vnd_cli_model || vnd_cli_model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
		LOG_WRN("--- [ROUTE INIT]: Chua cau hinh Publish Address de tim duong! ---");
		return;
	}

	// Reset routing state
	route_next_hop = BT_MESH_ADDR_UNASSIGNED;
	path_discovered = false;
	relay_for_client = BT_MESH_ADDR_UNASSIGNED;
	route_discovery_in_progress = true;
	rreq_seq++;
	best_hop_count = 0xFF;

	// Prepare RREQ packet
	struct net_buf_simple *msg = vnd_cli_model->pub->msg;
	net_buf_simple_reset(msg);
	bt_mesh_model_msg_init(msg, VND_OP_RREQ);
	
	uint16_t my_addr = bt_mesh_primary_addr();
	net_buf_simple_add_le16(msg, my_addr);
	net_buf_simple_add_u8(msg, rreq_seq);
	net_buf_simple_add_u8(msg, 0); // hop count is 0 initially

	// Mac dinh gui tin RREQ: 1 lan gui lai, cach nhau 50ms de tranh nghen kenh
	vnd_cli_model->pub->retransmit = BT_MESH_PUB_TRANSMIT(1, 50);

	err = bt_mesh_model_publish(vnd_cli_model);
	if (err) {
		LOG_ERR("Loi publish RREQ (err %d)", err);
	} else {
		LOG_INF("--- [ROUTE RREQ]: Da phat RREQ: origin=0x%04X, seq=%u, gui toi group [0x%04X] ---", 
				my_addr, rreq_seq, vnd_cli_model->pub->addr);
	}

	k_work_reschedule(&discovery_timeout_work, K_MSEC(2000));
}

/* XỬ LÝ HẾT THỜI GIAN CHỜ ROUTE REPLY */
static void discovery_timeout_handler(struct k_work *work)
{
	route_discovery_in_progress = false;
	
	if (path_discovered) {
		LOG_INF("--- [ROUTE]: Ket thuc tim duong. Duong truyen hop le, next hop = 0x%04X ---", route_next_hop);
	} else {
		// Fallback to sending direct/mesh publish destination address
		if (vnd_cli_model && vnd_cli_model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
			route_next_hop = vnd_cli_model->pub->addr;
			LOG_INF("--- [ROUTE]: Ket thuc tim duong. Khong tim thay, fallback gui thang toi 0x%04X ---", route_next_hop);
		} else {
			LOG_WRN("--- [ROUTE]: Ket thuc tim duong. Khong tim thay va chua cau hinh publish! ---");
		}
	}
	
	// Start periodic publish
	k_work_reschedule(&publish_work, K_MSEC(500));
}

static void init_subscription_handler(struct k_work *work)
{
	if (vnd_cli_model && vnd_cli_model->pub && vnd_cli_model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		vnd_cli_model->groups[0] = vnd_cli_model->pub->addr;
		LOG_INF("--- [SYSTEM]: Tu dong subscribe Group Address: 0x%04X ---", vnd_cli_model->pub->addr);
	}
}

/* HANDLERS CHO VENDOR MODEL OPCODES */
static int handle_rreq(const struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf)
{
	uint16_t my_addr = bt_mesh_primary_addr();
	uint16_t origin = net_buf_simple_pull_le16(buf);
	uint8_t seq = net_buf_simple_pull_u8(buf);
	uint8_t hop_count = net_buf_simple_pull_u8(buf);

	if (origin == my_addr) {
		// Ignore our own RREQ
		return 0;
	}

	if (rev_route.origin == origin && rev_route.seq == seq) {
		// Ignore duplicate RREQ
		return 0;
	}

	LOG_INF("--- [ROUTE RREQ RECV]: Nhan RREQ tu 0x%04X: origin=0x%04X, seq=%u, hop=%u ---", 
			ctx->addr, origin, seq, hop_count);

	// Save reverse route
	rev_route.origin = origin;
	rev_route.reverse_hop = ctx->addr; // neighbor that forwarded this RREQ to us
	rev_route.seq = seq;

	// Forward the RREQ
	struct net_buf_simple *msg = model->pub->msg;
	net_buf_simple_reset(msg);
	bt_mesh_model_msg_init(msg, VND_OP_RREQ);
	net_buf_simple_add_le16(msg, origin);
	net_buf_simple_add_u8(msg, seq);
	net_buf_simple_add_u8(msg, hop_count + 1);

	// Chuyen tiep RREQ: 1 lan gui lai, cach nhau 50ms
	model->pub->retransmit = BT_MESH_PUB_TRANSMIT(1, 50);

	int err = bt_mesh_model_publish(model);
	if (err) {
		LOG_ERR("Loi forward RREQ (err %d)", err);
	} else {
		LOG_INF("--- [ROUTE RREQ]: Da forward RREQ: origin=0x%04X, hop=%u ---", origin, hop_count + 1);
	}

	return 0;
}

static int handle_rrep(const struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf)
{
	uint16_t my_addr = bt_mesh_primary_addr();
	uint16_t origin = net_buf_simple_pull_le16(buf);
	uint16_t gateway = net_buf_simple_pull_le16(buf);
	uint16_t next_hop = net_buf_simple_pull_le16(buf);
	uint8_t hop_count = net_buf_simple_pull_u8(buf);

	if (next_hop != my_addr) {
		// Not for us
		return 0;
	}

	LOG_INF("--- [ROUTE RREP RECV]: Nhan RREP: origin=0x%04X, gateway=0x%04X, next_hop=0x%04X, hop=%u ---", 
			origin, gateway, next_hop, hop_count);

	if (origin == my_addr) {
		// We are the initiator! Route established
		if (hop_count < best_hop_count) {
			best_hop_count = hop_count;
			route_next_hop = ctx->addr; // The neighbor that sent us this RREP
			path_discovered = true;
			LOG_INF("--- [ROUTE RREP]: Cap nhat duong truyen tot nhat qua next hop 0x%04X, hop=%u ---", 
					route_next_hop, hop_count);
		} else {
			LOG_INF("--- [ROUTE RREP]: Bo qua RREP voi hop=%u vi da co duong tot hon (hop=%u) ---", 
					hop_count, best_hop_count);
		}
	} else {
		// We are an intermediate relay!
		relay_for_client = origin;
		route_next_hop = ctx->addr; // For forwarding messages to Gateway, go through this neighbor
		LOG_INF("--- [ROUTE RREP]: Dang ky lam relay cho 0x%04X. Next hop den Gateway la 0x%04X ---", 
				origin, route_next_hop);

		// Forward RREP ve reverse hop
		struct net_buf_simple *msg = model->pub->msg;
		net_buf_simple_reset(msg);
		bt_mesh_model_msg_init(msg, VND_OP_RREP);
		net_buf_simple_add_le16(msg, origin);
		net_buf_simple_add_le16(msg, gateway);
		net_buf_simple_add_le16(msg, rev_route.reverse_hop); // update next_hop for next segment
		net_buf_simple_add_u8(msg, hop_count);

		// Chuyen tiep RREP: 1 lan gui lai, cach nhau 50ms
		model->pub->retransmit = BT_MESH_PUB_TRANSMIT(1, 50);

		int err = bt_mesh_model_publish(model);
		if (err) {
			LOG_ERR("Loi forward RREP (err %d)", err);
		} else {
			LOG_INF("--- [ROUTE RREP]: Da forward RREP ve 0x%04X cho origin 0x%04X ---", 
					rev_route.reverse_hop, origin);
		}
	}

	return 0;
}

static int handle_sensor_data_relay(const struct bt_mesh_model *model,
				    struct bt_mesh_msg_ctx *ctx,
				    struct net_buf_simple *buf)
{
	uint16_t origin = net_buf_simple_pull_le16(buf);
	uint16_t next_hop = net_buf_simple_pull_le16(buf);
	uint32_t sensor_value = net_buf_simple_pull_le32(buf);

	uint16_t my_addr = bt_mesh_primary_addr();

	if (next_hop != my_addr) {
		// Not for us
		return 0;
	}

	LOG_INF("--- [SENSOR DATA RECV]: Nhan sensor tu 0x%04X cho next hop 0x%04X, value=%u ---", 
			origin, next_hop, sensor_value);

	if (relay_for_client == origin) {
		// Relay the data
		LOG_INF("--- [SENSOR DATA RELAY]: Dang relay du lieu cua 0x%04X qua next hop den Gateway 0x%04X ---", 
				origin, route_next_hop);

		struct net_buf_simple *msg = model->pub->msg;
		net_buf_simple_reset(msg);
		bt_mesh_model_msg_init(msg, VND_OP_SENSOR_DATA);
		net_buf_simple_add_le16(msg, origin);
		net_buf_simple_add_le16(msg, route_next_hop);
		net_buf_simple_add_le32(msg, sensor_value);

		// Yeu cua cua nguoi dung: Khi phai relay, gui lai 6 lan (tong cong 7 lan gui) cach nhau 50ms
		model->pub->retransmit = BT_MESH_PUB_TRANSMIT(6, 50);

		int err = bt_mesh_model_publish(model);
		if (err) {
			LOG_ERR("Loi relay sensor data (err %d)", err);
		}
	} else {
		LOG_INF("--- [SENSOR DATA RECV]: Toi khong phai relay cho 0x%04X. Bo qua. ---", origin);
	}

	return 0;
}

/* HÀM QUẢN LÝ CHU KỲ NGUỒN THỨC/NGỦ */
static void cycle_handler(struct k_work *work)
{
	if (!cycle_mode_active) {
		return;
	}

	is_awake = !is_awake;

	if (is_awake) {
		/* --- 🟢 MẠCH THỨC GIẤC (10s) --- */
		bt_mesh_resume(); 
		LOG_INF("--- [RADIO]: Da khoi dong lai Radio! ---");

		for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
			if (i != 1 && i != 2 && i != 3) { 
				dk_set_led(i, led_ctx[i].value ? LED_ON : LED_OFF);
			}
		}
		
		led2_blink_state = true;
		dk_set_led(2, LED_ON);
		k_work_reschedule(&blink_work, K_MSEC(100)); 

		next_state_on = true; 
		
		if (path_discovered) {
			k_work_reschedule(&publish_work, K_MSEC(500));
		} else {
			start_path_discovery();
		}
		k_work_reschedule(&cycle_work, K_MSEC(10000));

		simulated_sensor_value += 1; 
		LOG_INF("--- [SENSOR]: Gia tri sensor hien tai = %d ---", simulated_sensor_value);
		display_sensor_value();

	} else {
		/* --- 🔴 MẠCH SẮP ĐI NGỦ (10s) --- */
		k_work_cancel_delayable(&publish_work); 
		// Cho tre 1.5 giay de cac goi tin dang cho hoac dang gui lai (retransmit) truyen xong truoc khi tat radio, tranh ro ri bo dem advertiser.
		k_work_reschedule(&suspend_work, K_MSEC(1500));
		k_work_reschedule(&cycle_work, K_MSEC(10000));
	}
}

static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp)
{
	struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);
	int led_idx = led - &led_ctx[0];

	if (led_idx == 1) { 
		led->value = set->on_off;
		if (set->on_off == 1) {
			dk_set_led(1, LED_ON);
		} else {
			dk_set_led(1, LED_OFF);
		}
	} 
	else if (led_idx == 2) { 
		led->value = set->on_off;

		if (set->on_off == 1) {
			if (!cycle_mode_active) {
				cycle_mode_active = true;
				is_awake = true;
				
				k_work_reschedule(&blink_work, K_NO_WAIT);    
				k_work_reschedule(&cycle_work, K_MSEC(10000));
				
				next_state_on = true;
				start_path_discovery();

				LOG_INF("--- [SENSOR]: Bat dau chu ky, gia tri sensor = %d ---", simulated_sensor_value);
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
			dk_set_led(2, LED_ON);

			for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
				if (i != 1 && i != 2 && i != 3) {
					dk_set_led(i, led_ctx[i].value ? LED_ON : LED_OFF);
				}
			}
		}
	} 
	else {
		led->value = set->on_off; 
		if (!bt_mesh_model_transition_time(set->transition)) {
			led->remaining = 0;
			if (is_awake && led_idx != 3) {
				dk_set_led(led_idx, set->on_off ? LED_ON : LED_OFF);
			}
		} else {
			led->remaining = set->transition->time;
			if (set->transition->delay) {
				k_work_reschedule(&led->work, K_MSEC(set->transition->delay));
			} else {
				if (is_awake && led_idx != 3) {
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
		if (is_awake && led_idx != 1 && led_idx != 2 && led_idx != 3) {
			dk_set_led(led_idx, led->value ? LED_ON : LED_OFF);
		}
		struct bt_mesh_onoff_status status;
		led_status(led, &status);
		bt_mesh_onoff_srv_pub(&led->srv, NULL, &status);
	}
}

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

static struct bt_mesh_elem elements[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
	BT_MESH_ELEM(
		1, BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_CFG_SRV,
			BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
			BT_MESH_MODEL_ONOFF_SRV(&led_ctx[0].srv),
			BT_MESH_MODEL_ONOFF_CLI(&onoff_cli)),
		BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_VND(VND_COMPANY_ID, VND_MODEL_ID_CLI, vnd_cli_ops, &vnd_pub, NULL)
		)),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
	BT_MESH_ELEM(
		2, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[1].srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
	BT_MESH_ELEM(
		3, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[2].srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
	BT_MESH_ELEM(
		4, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[3].srv)),
		BT_MESH_MODEL_NONE),
#endif
};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

const struct bt_mesh_comp *model_handler_init(void)
{
	k_work_init_delayable(&attention_blink_work, attention_blink);

	for (int i = 0; i < ARRAY_SIZE(led_ctx); ++i) {
		k_work_init_delayable(&led_ctx[i].work, led_work);
		led_ctx[i].value = 0; 
	}

	k_work_init_delayable(&blink_work, blink_handler);
	k_work_init_delayable(&cycle_work, cycle_handler);
	k_work_init_delayable(&publish_work, publish_handler); 
	k_work_init_delayable(&suspend_work, suspend_handler); 
	k_work_init_delayable(&discovery_timeout_work, discovery_timeout_handler);
	k_work_init_delayable(&init_subscription_work, init_subscription_handler);
	k_work_reschedule(&init_subscription_work, K_MSEC(2000));

	vnd_cli_model = &elements[0].vnd_models[0];

	cycle_mode_active = false;
	is_awake = true;


	
	for (int i = 0; i < ARRAY_SIZE(led_ctx); i++) {
		dk_set_led(i, LED_OFF); 
	}
	dk_set_led(2, LED_ON);

	return &comp;
}