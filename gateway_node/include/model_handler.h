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

const struct bt_mesh_comp *model_handler_init(void);
void check_and_self_provision(void);
int send_special_sensor_message(uint16_t dst_addr, uint8_t data_val);


#ifdef __cplusplus
}
#endif

#endif /* MODEL_HANDLER_H__ */
