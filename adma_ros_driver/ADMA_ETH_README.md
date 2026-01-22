# ADMA ETH 驱动说明 (ROS1)

本文档说明 ETH (UDP) 驱动的流程与输出，涉及：
- `adma_ros_driver/src/adma_driver.cpp`
- `adma_ros_driver/src/parser/*`

内容包括配置参数、UDP 接收、解析逻辑、发布话题与示例。

---

## 1) 配置参数 (launch)
驱动从 `ADMA_pub_Ethernet.launch` 读取参数，且参数路径需要位于
`adma_driver/` 命名空间下。

必需参数：
- `destination_ip` (string)：本机网卡 IP，驱动会绑定到该 IP 接收 UDP。
- `destination_port` (int)：本机绑定端口。
- `protocol_version` (string)：`v3.3.3` / `v3.3.4` / `v3.3.5`。

可选参数：
- `mode` (string)：`default` / `record` / `replay`（当前代码中不影响解析逻辑）。
- `frame_id_navsatfix` / `frame_id_imu` / `frame_id_adma` / `frame_id_adma_status`
  / `frame_id_data_raw`：各话题的 frame_id。
- `truth_rate_hz` (double)：限制 `adma/truth` 发布频率，`0` 表示不限制。
- `use_system_time` (bool)：true 使用系统时间，false 使用 ADMA GNSS 时间。
- `publish_truth` (bool)：true 发布 `adma/truth`，false 禁用该话题。
- `truth_log` (bool)：true 时在终端打印 `adma/truth` 数值。
- `truth_log_hz` (double)：终端打印频率（Hz）。
- `log_gsdb` (bool)：true 时启动 `adma_tools_cpp/bag2gsdb_converter`。
- `record_rosbag` (bool)：true 时启动 `rosbag record -a`。
- `log_path` (string)：gsdb/rosbag 输出目录。

重要：`destination_ip` 应该是本机网卡 IP。使用广播地址
`192.168.88.255` 通常会 bind 失败。

示例 (本机网卡 = 192.168.88.10，ADMA 发送到 192.168.88.10:11021):
```xml
<include file="$(find adma_ros_driver)/launch/ADMA_pub_Ethernet.launch">
  <arg name="destination_ip" value="192.168.88.10"/>
  <arg name="destination_port" value="11021"/>
  <arg name="protocol_version" value="v3.3.4"/>
  <arg name="mode" value="default"/>
  <arg name="log_gsdb" value="false"/>
  <arg name="record_rosbag" value="false"/>
</include>
```

---

## 2) UDP 接收流程 (`adma_driver.cpp`)

### 2.1 初始化
- 通过 `ros::param::get("adma_driver/...", ...)` 读取参数。
- 根据 `protocol_version` 创建对应的 ROS publisher。
- 无论版本都发布 `adma/data_raw`。
- 创建解析器 `ADMA2ROSParser`。
- 打开 UDP socket 并进入接收循环。

### 2.2 UDP socket 创建
核心步骤：
1) 设置 `addrinfo` (UDP)。
2) 端口转字符串。
3) 填充 `sockaddr_in` (IP+port)。
4) `getaddrinfo()` 做地址解析。
5) `socket()` 创建 UDP socket。
6) `bind()` 绑定到本机 IP/端口。

若 `bind()` 失败，直接报错退出：
```
Could not bind UDP socket with: "<ip>:<port>"
```

### 2.3 接收循环
- `select()` 最多等待 1 秒。
- `recv()` 读取 856 字节到 `recv_buf`。
- 长度不是 856 就丢弃。
- 进入 `parseData(recv_buf)`。

---

## 3) 解析流程 (按协议版本)

### 3.1 调度入口
`parseData()` 根据 `protocol_version` 调用：
- `parseV333()` (v3.3.3)
- `parseV334()` (v3.3.4)
- `parseV335()` (v3.3.5)

### 3.2 解析器模块
`adma_ros_driver/src/parser/`：
- `adma2ros_parser.cpp`：封装 v333/v334/v335 三套解析器入口。
- `adma2ros_parser_v333.cpp`：`AdmaDataV333` -> `Adma`。
- `adma2ros_parser_v334.cpp`：`AdmaDataV334` -> `AdmaDataScaled` + `AdmaStatus`。
- `adma2ros_parser_v335.cpp`：`AdmaDataV335` -> `AdmaDataScaled` + `AdmaStatus`。
- `parser_utils.cpp`：位操作等工具函数。

协议版本必须匹配设备输出，否则字段会错位。

---

## 4) 发布话题与类型 (ETH)

通用 (所有版本)：
- `adma/data_raw` (`adma_ros_driver_msgs/AdmaDataRaw`)：原始字节流。

v3.3.3：
- `adma/data` (`adma_ros_driver_msgs/Adma`)

v3.3.4 / v3.3.5：
- `adma/data_scaled` (`adma_ros_driver_msgs/AdmaDataScaled`)
- `adma/status` (`adma_ros_driver_msgs/AdmaStatus`)
- `adma/heading` (`std_msgs/Float64`)：`ins_yaw`
- `adma/velocity` (`std_msgs/Float64`)：`sqrt(vx^2 + vy^2) * 3.6` (km/h)
- `adma/fix` (`sensor_msgs/NavSatFix`)
- `adma/imu` (`sensor_msgs/Imu`)

常见单位：
- `ins_vel_frame`：m/s
- `adma/velocity`：km/h
- `rate_hor`：deg/s
- `acc_hor`：g

---

## 5) 真值变量提取示例

### 5.1 来源字段 (ETH)
来自 `adma/data_scaled`：
- `actual_spd = sqrt(ins_vel_frame.x^2 + ins_vel_frame.y^2)` (m/s)
- `yaw_rate = rate_hor.z` (deg/s)
- `long_accel = acc_hor.x` (g)
- `lat_accel = acc_hor.y` (g)
- `yaw_rate_sign = (yaw_rate < 0) ? 1 : 0`

也可直接用 `adma/velocity` (km/h) 作为速度来源。

### 5.2 坐标系与符号说明
- `ins_vel_frame` 和 `acc_hor/rate_hor` 都在 ADMA 定义的坐标系中。
- 通常 `acc_hor.x`=纵向、`acc_hor.y`=横向，但必须以 ADMA 文档为准。
- 如果你的车辆坐标与 ADMA 相反，则符号会翻转。

### 5.2.1 单位与符号
- `acc_hor.*` 单位为 g (1 g = 9.80665 m/s^2)。
- `rate_hor.*` 单位为 deg/s。
- `actual_spd` 是速度模长，不受坐标轴符号影响。

### 5.2.2 yaw_rate_sign
- 当前逻辑：`yaw_rate < 0` 时 `yaw_rate_sign = 1`，否则 0。
- 若你需要相反规则，请自行改判断条件。

### 5.2.3 轴向/符号调整示例 (仅说明，不改驱动)
```
# 示例1：ADMA 的 Y 指向左，但你的车辆 Y 指向右
lat_accel = -acc_hor.y
yaw_rate  = -rate_hor.z
yaw_rate_sign = (yaw_rate < 0) ? 1 : 0

# 示例2：ADMA 的 X 指向前，而你的车辆 X 指向后
long_accel = -acc_hor.x

# 示例3：ADMA X/Y 轴对调
long_accel = acc_hor.y
lat_accel  = acc_hor.x
```
可在下游节点或自定义 truth 话题里做这些变换。

### 5.3 便捷真值话题 (已添加)
发布 `adma/truth`：
- 类型：`adma_ros_driver_msgs/AdmaTruth`
- 字段：`actual_spd`、`yaw_rate`、`lat_accel`、`long_accel`、`yaw_rate_sign`
- header：`frame_id = frame_id_adma`，`stamp = system time`

该话题每收到一帧 UDP 数据就发布一次，频率与 ADMA 输出频率一致。

### 5.4 降采样到固定频率 (例如 20 Hz)
方式 A：驱动内限频（推荐，直接设置参数）
- 在 launch 中设置：`truth_rate_hz:=20.0`

方式 B：外部 throttle 节点
```bash
rosrun topic_tools throttle messages /adma/truth 20.0 /adma/truth_20hz
```
上面命令会启动一个节点，把 `/adma/truth` 以 20 Hz 重新发布到
`/adma/truth_20hz`。也可写进 launch：
```xml
<node pkg="topic_tools" type="throttle" name="adma_truth_throttle"
      args="messages /adma/truth 20.0 /adma/truth_20hz" />
```
这既适用于实时，也适用于回放。

### 5.5 终端打印与单位换算示例
打印 `adma/truth` 数值只需在 launch 里加一行：
```
truth_log:=true
```
可配合 `truth_log_hz` 控制打印频率（Hz）。

单位换算示例（基于当前代码字段）：
```
# 速度：m/s -> km/h
speed_kmh = actual_spd * 3.6

# 加速度：g -> m/s^2
accel_ms2 = lat_accel * 9.80665

# 角速度：deg/s -> rad/s
yaw_rate_rad = yaw_rate * 3.1415926 / 180.0
```

---

## 6) 常见问题

1) 使用广播 IP 无法 bind：
   - `destination_ip` 请设为本机网卡 IP。

2) 无设备/无数据时：
   - 会打印 `Waiting for ADMA data...`，且不发布数据。

3) 协议版本不匹配：
   - 字段解析会错位，数值不可信。
