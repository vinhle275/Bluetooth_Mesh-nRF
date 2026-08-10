# Gateway ESP32-S3 Firmware

Dự án firmware Bluetooth Mesh Gateway cho mạch **ESP32-S3**, được thiết kế và hiện thực đồng bộ 100% chức năng, cấu trúc Element/Model, và định dạng Console/RTT Log với **`gateway_node`** (nRF52840).

## 1. Cấu Trúc Element & Model (Y Hệt `gateway_node`)

- **Element 1 (Primary Element, Provisioned Element 0 trong ESP-IDF)**:
  - Configuration Server Model (`ESP_BLE_MESH_MODEL_CFG_SRV`)
  - Health Server Model (`ESP_BLE_MESH_MODEL_HEALTH_SRV`)
  - Sensor Client Model (`ESP_BLE_MESH_MODEL_SENSOR_CLI`), lắng nghe dữ liệu Cảm biến trên Group Address `0xC000`.

- **Element 2 (Secondary Element, Provisioned Element 1 trong ESP-IDF)**:
  - Generic OnOff Server Model (`ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV`).
  - Khi nhận lệnh **ON** hoặc **OFF** từ ứng dụng nRF Mesh trên điện thoại, tự động kích hoạt gửi gói tin đặc biệt Broadcast `0xFFFF` với `TTL = 7`.

## 2. Gói Tin Đặc Biệt (Special Sensor Message)

- **Opcode**: `0x8299` (`SPECIAL_SENSOR_OP`).
- **Địa chỉ phát**: `0xFFFF` (`ESP_BLE_MESH_ADDR_ALL_NODES` / Broadcast toàn mạng).
- **TTL**: `7` (Multi-hop mesh broadcast với TTL = 7).
- **Trường dữ liệu**: `1` khi ON, `0` khi OFF.
- **Log định dạng**:
  ```text
  GW_ONOFF ON received from nRF Mesh app -> Broadcasting Special Sensor Message (data=1, TTL=7) to ALL NODES (0xFFFF)
  GW_TX_SPECIAL_OK dst=0xffff op=0x8299 data=1 ttl=7 (Broadcast with TTL=7)
  ```

## 3. Quản Lý Dữ Liệu Cảm Biến & Log Nhận Dữ Liệu (Bao gồm TTL = 1)

- Theo dõi mảng `measurements[32]` cho các nút gửi dữ liệu Motion (`0x0042`) và Battery Level (`0x0054`) từ các chip nRF và node khác.
- **Log khi nhận được từng thuộc tính (`GW_RX`)**:
  ```text
  GW_RX count=1 t_ms=189114 src=0x0010 dst=0xc000 property=0x0042 value=57% ttl=1 rssi=-55 [TTL=1 Direct Rx]
  ```
- **Log khi hoàn tất gói tin tổng hợp (`GW_PACKET RECEIVED!`)**:
  ```text
  =========================================================================
  GW_PACKET RECEIVED! count=1 t_ms=189114 src=0x0010 motion=57% battery=50% delta_ms=2 ttl=1 dst=0xc000 rssi=-55 [TTL=1 Direct Rx]
  =========================================================================
  ```
- **Log định kỳ trạng thái Gateway (`GW_STATUS`)**:
  ```text
  GW_STATUS Listening on group 0xc000... total_rx=1, complete_packets=1, uptime=30s
  ```

## 4. Hướng Dẫn Biên Dịch & Nạp Cho ESP32-S3 (ESP-IDF)

### Yêu cầu:
- ESP-IDF v4.4, v5.0 hoặc v5.1+
- Đã cài đặt môi trường `idf.py` trong terminal.

### Các lệnh biên dịch & nạp:

```bash
# 1. Di chuyển vào thư mục gateway_esp32s3
cd gateway_esp32s3

# 2. Chọn target chip ESP32-S3
idf.py set-target esp32s3

# 3. Biên dịch dự án
idf.py build

# 4. Nạp firmware vào bo mạch ESP32-S3 và mở màn hình Monitor log
idf.py -p COMx flash monitor
```
*(Thay `COMx` bằng cổng Serial COM thực tế của mạch ESP32-S3 trên máy tính)*.

