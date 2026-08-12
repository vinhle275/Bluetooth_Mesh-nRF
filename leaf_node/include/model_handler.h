/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @brief Model handler
 */

#ifndef MODEL_HANDLER_H__
#define MODEL_HANDLER_H__

#include <zephyr/bluetooth/mesh.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * CẤU HÌNH THAM SỐ GÓI TIN ĐẶC BIỆT & CHU KỲ NỔI / NGỦ CỦA LEAF NODE
 * =========================================================================
 */
#ifndef SPECIAL_PKT_DELAY_SEC
#define SPECIAL_PKT_DELAY_SEC        5       /* Thời gian delay gửi gói tin đặc biệt (giây) */
#endif

#ifndef LEAF_SLEEP_DURATION_SEC
#define LEAF_SLEEP_DURATION_SEC      5      /* Thời gian Leaf ngủ trong 1 chu kỳ (giây) */
#endif

#ifndef LEAF_AWAKE_DURATION_SEC
#define LEAF_AWAKE_DURATION_SEC      5       /* Thời gian Leaf thức trong 1 chu kỳ (giây) */
#endif

#ifndef SENSOR_TX_MAX_RETRIES
#define SENSOR_TX_MAX_RETRIES        3       /* Số lần thử lại khi gửi tin bị lỗi */
#endif

#ifndef SENSOR_TX_RETRY_DELAY_MS
#define SENSOR_TX_RETRY_DELAY_MS     500     /* Thời gian trễ giữa mỗi lần thử lại (ms) */
#endif

/* =========================================================================
 * ĐỊNH NGHĨA TRẠNG THÁI ĐÈN / LỆNH ONOFF (ON / OFF)
 * =========================================================================
 */
#ifndef LEAF_LED_STATE_ON
#define LEAF_LED_STATE_ON            0       /* Giá trị BẬT đèn / Kích hoạt Cycle mode (1 = ON) */
#endif

#ifndef LEAF_LED_STATE_OFF
#define LEAF_LED_STATE_OFF           1       /* Giá trị TẮT đèn / Dừng Cycle mode (0 = OFF) */
#endif

const struct bt_mesh_comp *model_handler_init(void);


#ifdef __cplusplus
}
#endif

#endif /* MODEL_HANDLER_H__ */
