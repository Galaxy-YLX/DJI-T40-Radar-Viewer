# DJI DUML V1 Protocol - T40 Radar ESC

Reverse engineered from `RAD_ESC_gd02_2400_v06.00.00.07_2400.bin` (GD32F305, uCos-III).

---

## 1. 帧格式

```
+------+----------+------+--------+----------+-----+-------+---------+---------+---------+-------+
| SOF  | LEN+VER  | CRC8 | SENDER | RECEIVER | SEQ | FLAGS | CMD_SET | CMD_ID  | PAYLOAD | CRC16 |
| 1B   | 2B       | 1B   | 1B     | 1B       | 2B  | 1B    | 1B      | 1B      | NB      | 2B    |
+------+----------+------+--------+----------+-----+-------+---------+---------+---------+-------+
  [0]    [1:2]      [3]    [4]      [5]       [6:7]  [8]     [9]      [10]     [11..L-3]  [L-2:L-1]
```

- **最小帧长**: 13 字节 (无 payload)
- **最大帧长**: 1023 字节 (受 10-bit 长度字段限制)
- **字节序**: 全部小端

### 1.1 SOF (1 byte)

固定 `0x55`。

### 1.2 LEN+VER (2 bytes)

```
byte[1] = total_length & 0xFF
byte[2] = ((total_length >> 8) & 0x03) | (version << 2)
```

- `total_length`: 整帧字节数 (含 SOF 和 CRC16)
- `version`: 协议版本, V1 = 1, 所以 byte[2] 的高 6 位为 `0x04`

解码:
```
total_length = ((byte[2] & 0x03) << 8) | byte[1]
version      = byte[2] >> 2
```

### 1.3 CRC8 (1 byte)

覆盖 byte[0..2] (SOF + LEN+VER), init = `0x77`, 256 字节查找表。

校验方式: 对 byte[0..3] 计算 CRC8, 结果应为 0。

### 1.4 SENDER / RECEIVER (各 1 byte)

```
bit[7:5] = device_type (3 bits)
bit[4:0] = device_id   (5 bits)
```

编码: `byte = (type << 5) | (id & 0x1F)`

### 1.5 SEQ (2 bytes, 小端)

序列号, 用于请求/应答配对。

### 1.6 FLAGS (1 byte)

```
bit 7   : (保留)
bit 6   : need_ack (1=请求帧需要回复)
bit[5:4]: ack_type
bit[3:0]: encrypt
```

- 请求帧: `0x40` (need_ack=1)
- ACK 帧: `0x00` 或 `0x80`

### 1.7 CMD_SET + CMD_ID (各 1 byte)

命令集和命令 ID, 决定具体功能。

### 1.8 PAYLOAD (可变长)

长度 = total_length - 13。可为空。

### 1.9 CRC16 (2 bytes, 小端)

覆盖 byte[0..L-3] (除 CRC16 自身外的全部字节), init = `0x3692`, 256 x 16-bit 查找表。

校验方式: 对整帧计算 CRC16, 结果应为 0。

---

## 2. CRC 算法

### 2.1 CRC8

- 多项式: 0x5E (通过查表实现)
- 初始值: 0x77
- 输入/输出不反转

```python
CRC8_TABLE = [
    0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,
    0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,
    0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,
    0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,
    0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,
    0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,
    0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,
    0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,
    0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,
    0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,
    0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,
    0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,
    0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,
    0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,
    0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,
    0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35,
]

def crc8(data: bytes) -> int:
    crc = 0x77
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc
```

### 2.2 CRC16

- 反射多项式: 0x1189
- 初始值: 0x3692

```python
CRC16_TABLE = [
    0x0000,0x1189,0x2312,0x329B,0x4624,0x57AD,0x6536,0x74BF,
    0x8C48,0x9DC1,0xAF5A,0xBED3,0xCA6C,0xDBE5,0xE97E,0xF8F7,
    0x1081,0x0108,0x3393,0x221A,0x56A5,0x472C,0x75B7,0x643E,
    0x9CC9,0x8D40,0xBFDB,0xAE52,0xDAED,0xCB64,0xF9FF,0xE876,
    0x2102,0x308B,0x0210,0x1399,0x6726,0x76AF,0x4434,0x55BD,
    0xAD4A,0xBCC3,0x8E58,0x9FD1,0xEB6E,0xFAE7,0xC87C,0xD9F5,
    0x3183,0x200A,0x1291,0x0318,0x77A7,0x662E,0x54B5,0x453C,
    0xBDCB,0xAC42,0x9ED9,0x8F50,0xFBEF,0xEA66,0xD8FD,0xC974,
    0x4204,0x538D,0x6116,0x709F,0x0420,0x15A9,0x2732,0x36BB,
    0xCE4C,0xDFC5,0xED5E,0xFCD7,0x8868,0x99E1,0xAB7A,0xBAF3,
    0x5285,0x430C,0x7197,0x601E,0x14A1,0x0528,0x37B3,0x263A,
    0xDECD,0xCF44,0xFDDF,0xEC56,0x98E9,0x8960,0xBBFB,0xAA72,
    0x6306,0x728F,0x4014,0x519D,0x2522,0x34AB,0x0630,0x17B9,
    0xEF4E,0xFEC7,0xCC5C,0xDDD5,0xA96A,0xB8E3,0x8A78,0x9BF1,
    0x7387,0x620E,0x5095,0x411C,0x35A3,0x242A,0x16B1,0x0738,
    0xFFCF,0xEE46,0xDCDD,0xCD54,0xB9EB,0xA862,0x9AF9,0x8B70,
    0x8408,0x9581,0xA71A,0xB693,0xC22C,0xD3A5,0xE13E,0xF0B7,
    0x0840,0x19C9,0x2B52,0x3ADB,0x4E64,0x5FED,0x6D76,0x7CFF,
    0x9489,0x8500,0xB79B,0xA612,0xD2AD,0xC324,0xF1BF,0xE036,
    0x18C1,0x0948,0x3BD3,0x2A5A,0x5EE5,0x4F6C,0x7DF7,0x6C7E,
    0xA50A,0xB483,0x8618,0x9791,0xE32E,0xF2A7,0xC03C,0xD1B5,
    0x2942,0x38CB,0x0A50,0x1BD9,0x6F66,0x7EEF,0x4C74,0x5DFD,
    0xB58B,0xA402,0x9699,0x8710,0xF3AF,0xE226,0xD0BD,0xC134,
    0x39C3,0x284A,0x1AD1,0x0B58,0x7FE7,0x6E6E,0x5CF5,0x4D7C,
    0xC60C,0xD785,0xE51E,0xF497,0x8028,0x91A1,0xA33A,0xB2B3,
    0x4A44,0x5BCD,0x6956,0x78DF,0x0C60,0x1DE9,0x2F72,0x3EFB,
    0xD68D,0xC704,0xF59F,0xE416,0x90A9,0x8120,0xB3BB,0xA232,
    0x5AC5,0x4B4C,0x79D7,0x685E,0x1CE1,0x0D68,0x3FF3,0x2E7A,
    0xE70E,0xF687,0xC41C,0xD595,0xA12A,0xB0A3,0x8238,0x93B1,
    0x6B46,0x7ACF,0x4854,0x59DD,0x2D62,0x3CEB,0x0E70,0x1FF9,
    0xF78F,0xE606,0xD49D,0xC514,0xB1AB,0xA022,0x92B9,0x8330,
    0x7BC7,0x6A4E,0x58D5,0x495C,0x3DE3,0x2C6A,0x1EF1,0x0F78,
]

def crc16(data: bytes) -> int:
    crc = 0x3692
    for b in data:
        crc = CRC16_TABLE[(crc & 0xFF) ^ b] ^ (crc >> 8)
    return crc
```

---

## 3. 设备地址

### 3.1 device_type (3 bits)

| 值 | 含义 |
|----|------|
| 0  | 设备 (ESC/电调等) |
| 1  | PC / 地面端 |
| 2  | 无线链路 2 |
| 3  | 雷达 |
| 6  | 主控 / FC (抓包观察到, ESC 固件路由表未列出) |
| 7  | Agent / HMS |

### 3.2 device_id (5 bits)

| id | 含义 | 路由目标 |
|----|------|----------|
| 0x00 | 默认 (通配) | link_host (本地处理) |
| 0x02 | App / 遥控器 | link_cb (CAN) |
| 0x03 | 飞控 (FC) | link_cb (CAN) |
| 0x05 | 中心板 (Center) | link_cb (CAN) |
| 0x08 | 图传 (1860/OFDM) | link_cb (CAN) |
| 0x0A | PC 助手 | link_cb (CAN), 可被 link_pc 动态接管 |
| 0x11 | 自动测试 (type=3) | link_cb (CAN) |
| 0x13 | RDTT 协商从机 | link_cb (CAN) |
| 0x14 | RDTT 协商主机 (type=1) | link_cb (CAN) |
| 0x18 | **本机 ESC (self)** | link_host (本地处理) |
| 0x1D | HMS 健康管理 | link_cb (CAN) |
| 0x1F | 广播 | link_host (本地处理) |

接收方 id=0x18 且 type=0 的帧由本机处理; 其他组合按路由表转发到 CAN 总线。

---

## 4. 命令表

### 4.1 cmd_set = 0x00 (通用)

| cmd_id | 名称 | 说明 |
|--------|------|------|
| 0x00 | echo | 回显, payload 原样返回 |
| 0x01 | get_device_info | 获取设备信息 |
| 0x07 | upgrade | 固件升级入口 |
| 0x0B | reboot | 重启设备 |
| 0x0C | heartbeat | 心跳 |
| 0x32 | activation | 设备激活 |
| 0x50 | push_stub | Push (桩, 未实现) |
| 0x51 | product_sn_get | 获取产品序列号 |
| 0xFF | get_cfg_info | 获取配置信息 |

### 4.2 cmd_set = 0x03 (产品信息)

| cmd_id | 名称 | 说明 |
|--------|------|------|
| 0x73 | product_sn_set | 写入 SN (桩) |
| 0x74 | product_sn_read | 读取 SN |

### 4.3 cmd_set = 0x10 (RDTT)

| cmd_id | 名称 | 说明 |
|--------|------|------|
| 0x80 | rdtt_link_negotiate | RDTT 链路协商 |
| 0x82 | rdtt_send_test | RDTT 发送测试 |

### 4.4 cmd_set = 0x1E (ESC / 电机 / 雷达)

ESC 固件实现的入站 handler (app/雷达 → ESC):

| cmd_id | 名称 | 说明 |
|--------|------|------|
| 0x01 | motor_pwm_control | 电机 PWM 直接控制 |
| 0x07 | motor_pid | 电机 PID 参数设置 |
| 0x09 | motor_enable | 电机使能 |
| 0x0A | motor_disable | 电机禁用 |
| 0x0D | angle_sync_ack | 角度同步应答 |
| 0x21 | flyinfo | 飞行信息处理 |
| 0x80 | motor_speed_set | 电机转速设定 (2B payload) |
| 0x81 | motor_position | 电机位置控制 |
| 0x82 | motor_stop | 电机停止 |
| 0x83 | hms_switch | HMS 开关, 需要 ACK |
| 0xE5 | esc_e5_stub | ESC 命令 (桩) |
| 0xE6 | ar1021x_flash_read | AR1021x Flash 读取 |
| 0xEA | esc_ea_stub | ESC 命令 (桩) |
| 0xEB | esc_config_set | ESC 配置写入 |
| 0xF9 | state2_toggle | 周期状态切换 (fire-and-forget) |
| 0xFE | rdtt_status_req | RDTT 状态查询 |

ESC → app 实际观察到的周期推送 (抓包确认, 不走入站 handler 表):

| cmd_id | 长度 | 频率 | 内容 |
|--------|------|------|------|
| 0x0F | 44B | 4.3 Hz | bottom_data_pack (电机角度 / 状态) |
| 0x0B | 26B | 4.3 Hz | radar_imu_inter_pack (IMU / 振动) |
| 0xEA | 2B  | 0.5 Hz | keepalive `aa aa` |
| 0xF9 | 1B  | 配对  | state2_toggle 的 CAN 回环 (非真实回复) |

---

## 5. 解析流程

### 5.1 接收状态机

```
等待 SOF (0x55)
  │
  ├─ 收到 0x55 → 继续收 byte[1..2]
  │
  ├─ validate_header:
  │   1. byte[0] == 0x55 ?
  │   2. CRC8(byte[0..3]) == 0 ?
  │   3. length >= 13 ?
  │   4. version == 1 ? (byte[2] >> 2)
  │   全部通过 → 标记 header_valid, 继续收剩余字节
  │   任一失败 → 丢弃, 指针+1, 重新找 SOF
  │
  ├─ 收满 total_length 字节
  │
  ├─ validate_crc16:
  │   CRC16(byte[0..L-1]) == 0 ?
  │   失败 → 丢弃整帧
  │
  ├─ parse_frame: 提取各字段到结构体
  │   sender_id    = byte[4] & 0x1F
  │   sender_type  = byte[4] >> 5
  │   receiver_id  = byte[5] & 0x1F
  │   receiver_type= byte[5] >> 5
  │   seq          = byte[6] | (byte[7] << 8)
  │   flags        = byte[8]
  │   cmd_set      = byte[9]
  │   cmd_id       = byte[10]
  │   payload_ptr  = &byte[11]
  │   payload_len  = total_length - 13
  │
  └─ 交给路由/分发
```

### 5.2 路由分发

```
parse_frame 输出
  │
  ├─ 计算路由键: (receiver_type << 24) | (receiver_id << 16) | (receiver_type << 8)
  │
  ├─ 查路由表: 键 & mask == entry.key ?
  │   ├─ 匹配 link_host → 本地 cmd_dispatch
  │   ├─ 匹配 link_cb   → 转发到 CAN
  │   └─ 匹配 link_radar → 转发到 USART0
  │
  └─ cmd_dispatch (仅 link_host):
      遍历 handler 表, 匹配 cmd_set + cmd_id
      → 调用对应 handler
      → handler 构造 ACK 帧原路返回
```

### 5.3 构造发送帧 (伪代码)

```python
def build_frame(src_type, src_id, dst_type, dst_id, seq,
                cmd_set, cmd_id, payload=b"", need_ack=True):
    total = 11 + len(payload) + 2
    hdr = bytes([0x55, total & 0xFF, ((total >> 8) & 3) | 0x04])
    c8 = crc8(hdr)
    flags = 0x40 if need_ack else 0x00
    frame = hdr + bytes([
        c8,
        (src_type << 5) | src_id,
        (dst_type << 5) | dst_id,
        seq & 0xFF, (seq >> 8) & 0xFF,
        flags, cmd_set, cmd_id
    ]) + payload
    c16 = crc16(frame)
    frame += bytes([c16 & 0xFF, (c16 >> 8) & 0xFF])
    return frame
```

---

## 6. 物理层参数

| 链路 | 接口 | 参数 | 协议 |
|------|------|------|------|
| link_pc | UART3 (PC10/PC11) | 921600 8N1 | DUML V1 |
| link_cb | CAN0 | 1 Mbps, BS1=5Tq BS2=4Tq BRP=6 | DUML V1 |
| link_radar | USART0 (PB6/PB7) | 921600 8N1 | DUML V1 |
| console | UART4 (PC12/PD2) | 921600 8N1 | ASCII Shell |

### 6.1 link_cb 的 CAN 封装

基于 `can_link_open` / `can_ioctl_handler` / `sub_801F700` / `sub_801FEB0` / `sub_801FFC8` 的逆向，`link_cb` 在 CAN 上不是额外套一层包头，而是把 DUML 原始字节流直接切成 8 字节 CAN payload：

- `link_cb` 绑定的虚拟链路 id 为 `0x520518A2`
- 解码方式：
  - `bus = (link_id >> 22) & 0x3 = 0`
  - `gd32_tx_id = (link_id >> 11) & 0x7FF = 0x0A3`
  - `gd32_rx_id = link_id & 0x7FF = 0x0A2`
- 从固件视角：
  - GD32 发送使用标准帧 id `0x0A3`
  - GD32 接收使用标准帧 id `0x0A2`
- 从主机视角：
  - 主机应发送到 `0x0A2`
  - 主机应监听 `0x0A3`
- 字节流封装：
  - 发送端把完整 DUML 帧按顺序切成 `min(8, remaining)` 字节的多个 CAN 帧
  - 每个 CAN 帧只有 payload 数据，没有额外序号/长度/首尾标志
- 接收端：
  - 接收端只按对应 rx CAN id 收集 payload
  - 将多个 CAN 帧 payload 顺序拼接成字节流
  - 再由 DUML 状态机在字节流中寻找 `0x55`、长度、CRC8、CRC16 完整帧

因此主机侧用 `socketcan` 实现时，只需要：

1. 发送时把 DUML 帧按 8 字节切片发到 `0x0A2`
2. 接收时把 `0x0A3` 上的 payload 依次拼接
3. 在拼接后的字节流上跑 DUML V1 解析状态机

---

## 7. 示例帧

### Echo 请求 (发给本机 ESC, 带 payload 'A')

```
55 0E 04 66 26 18 01 00 40 00 00 41 F1 D0
```

| 字段 | 值 | 解释 |
|------|----|------|
| SOF | 55 | |
| LEN+VER | 0E 04 | length=14, version=1 |
| CRC8 | 66 | CRC8([55 0E 04]) |
| SENDER | 26 | type=1(PC), id=6 |
| RECEIVER | 18 | type=0(设备), id=24(ESC) |
| SEQ | 01 00 | seq=1 |
| FLAGS | 40 | need_ack=1 |
| CMD_SET | 00 | 通用 |
| CMD_ID | 00 | echo |
| PAYLOAD | 41 | 'A' |
| CRC16 | F1 D0 | CRC16(byte[0..11]) |

### get_device_info 请求 (无 payload)

```
55 0D 04 33 26 18 02 00 40 00 01 33 9D
```

---

## 8. C 参考实现

```c
#include <stdint.h>
#include <stddef.h>

/* Tables omitted for brevity - see duml_gen.py for full tables */
extern const uint8_t  crc8_table[256];
extern const uint16_t crc16_table[256];

uint8_t duml_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x77;
    for (size_t i = 0; i < len; i++)
        crc = crc8_table[crc ^ data[i]];
    return crc;
}

uint16_t duml_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0x3692;
    for (size_t i = 0; i < len; i++)
        crc = crc16_table[(crc & 0xFF) ^ data[i]] ^ (crc >> 8);
    return crc;
}

/* Validate a received frame. Returns 0 on success. */
int duml_validate(const uint8_t *frame, size_t len) {
    if (len < 13 || frame[0] != 0x55)
        return -1;
    /* Check CRC8 over first 4 bytes */
    if (duml_crc8(frame, 4) != 0)
        return -2;
    /* Extract total length */
    uint16_t total = ((frame[2] & 0x03) << 8) | frame[1];
    if (total != len || total < 13)
        return -3;
    /* Check version */
    if ((frame[2] >> 2) != 1)
        return -4;
    /* Check CRC16 over entire frame */
    if (duml_crc16(frame, total) != 0)
        return -5;
    return 0;
}

/* Parse validated frame into struct */
typedef struct {
    uint8_t  sender_type, sender_id;
    uint8_t  receiver_type, receiver_id;
    uint16_t seq;
    uint8_t  flags, cmd_set, cmd_id;
    const uint8_t *payload;
    uint16_t payload_len;
} duml_frame_t;

void duml_parse(const uint8_t *frame, duml_frame_t *f) {
    uint16_t total = ((frame[2] & 0x03) << 8) | frame[1];
    f->sender_id     = frame[4] & 0x1F;
    f->sender_type   = frame[4] >> 5;
    f->receiver_id   = frame[5] & 0x1F;
    f->receiver_type = frame[5] >> 5;
    f->seq           = frame[6] | (frame[7] << 8);
    f->flags         = frame[8];
    f->cmd_set       = frame[9];
    f->cmd_id        = frame[10];
    f->payload       = (total > 13) ? &frame[11] : NULL;
    f->payload_len   = total - 13;
}
```
