# Bluetooth Mesh nRF: Leaf / Gateway

Project này gồm hai firmware Bluetooth Mesh dùng Sensor Model chuẩn của NCS:

- `leaf_node`: Sensor Server, phát nhiệt độ mock định kỳ.
- `gateway_node`: Sensor Client, nhận và ghi log dữ liệu sensor.

Vai trò `leaf_node` và `gateway_node` **không phụ thuộc vào loại chip**. Mỗi firmware
được build riêng cho board thực tế trước khi nạp.

> Lưu ý: `nrf52480` thường là cách gọi nhầm của **nRF52840**.

## Các board được hỗ trợ

| Board | Board target |
|---|---|
| nRF52840 DK hoặc mạch dùng target nRF52840 | `nrf52840dk/nrf52840` |
| nRF54L15 DK | `nrf54l15dk/nrf54l15/cpuapp` |

Có thể dùng bốn tổ hợp sau:

| Leaf | Gateway | Firmware cần nạp |
|---|---|---|
| nRF52840 | nRF54L15 | `leaf_node` trên nRF52840, `gateway_node` trên nRF54L15 |
| nRF54L15 | nRF52840 | `leaf_node` trên nRF54L15, `gateway_node` trên nRF52840 |
| nRF52840 | nRF52840 | `leaf_node` và `gateway_node` đều build cho nRF52840 |
| nRF54L15 | nRF54L15 | `leaf_node` và `gateway_node` đều build cho nRF54L15 |

Sensor Model là giao thức Bluetooth Mesh chuẩn nên hai dòng chip có thể giao tiếp
chéo với nhau.

## Build và nạp firmware

Các lệnh dưới đây tạo thư mục build riêng cho từng board, tránh ghi đè firmware.

### Leaf trên nRF52840

```bat
west build -p always -b nrf52840dk/nrf52840 -d build/leaf_nrf52840 leaf_node
west flash -d build/leaf_nrf52840
```

### Leaf trên nRF54L15

```bat
west build -p always -b nrf54l15dk/nrf54l15/cpuapp -d build/leaf_nrf54l15 leaf_node
west flash -d build/leaf_nrf54l15
```

### Gateway trên nRF52840

```bat
west build -p always -b nrf52840dk/nrf52840 -d build/gateway_nrf52840 gateway_node
west flash -d build/gateway_nrf52840
```

### Gateway trên nRF54L15

```bat
west build -p always -b nrf54l15dk/nrf54l15/cpuapp -d build/gateway_nrf54l15 gateway_node
west flash -d build/gateway_nrf54l15
```

Ví dụ cấu hình nRF52840 làm Leaf và nRF54L15 làm Gateway:

```text
nRF52840  + leaf_node     ───── Sensor Model ─────►  nRF54L15 + gateway_node
```

Có thể đảo ngược phần cứng mà không cần đổi mã nguồn:

```text
nRF54L15  + leaf_node     ───── Sensor Model ─────►  nRF52840 + gateway_node
```

## Cấu hình Bluetooth Mesh sau khi nạp

Sau khi provision hai node bằng nRF Mesh hoặc provisioner tương thích:

1. Bind cùng AppKey cho Sensor Server trên Leaf và Sensor Client trên Gateway.
2. Cấu hình publication của Sensor Server trên Leaf đến địa chỉ của Gateway hoặc
   một group address.
3. Nếu dùng group address, cấu hình Sensor Client của Gateway subscribe group đó.
4. Kiểm tra log RTT của Gateway để thấy nhiệt độ mock từ Leaf.

Leaf tạo giá trị mock từ 20 đến 30 °C, tăng mỗi 5 giây rồi lặp lại. Giá trị này
không phụ thuộc cảm biến phần cứng thật.

## Lưu ý về DeviceTree

- `nrf54l15dk_nrf54l15.overlay` định nghĩa các LED GPIO riêng cho nRF54L15 DK.
- `nrf52840dk_nrf52840.overlay` trong project đang chứa chân GPIO theo mạch
  nRF52840 custom. Nếu dùng nRF52840 DK nguyên bản, cần kiểm tra lại LED/button
  hoặc điều chỉnh overlay trước khi sử dụng các LED đó.
- Sensor mock không cần chân GPIO cảm biến, nên phần Sensor Model vẫn hoạt động
  nếu không dùng LED.

Không cần sửa các file trong `ncs/v2.9.2`.