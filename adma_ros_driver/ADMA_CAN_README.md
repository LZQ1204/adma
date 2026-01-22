# ADMA CAN 真值节点说明

本说明对应：
- `adma_ros_driver/src/adma_truth_can_node.py`
- `adma_ros_driver/launch/ADMA_truth_CAN.launch`

该节点通过 Kvaser CAN (canlib) + DBC 解码输出 `/adma/truth`。

---

## 1) 使用前提
1) 已安装并配置 Kvaser 驱动与 canlib。
2) 已安装 Python 依赖：
   ```bash
   python3 -m pip install --user cantools
   ```
3) 已编译工作区并 source：
   ```bash
   catkin_make
   source devel/setup.bash
   ```

---

## 2) Launch 启动
编辑或直接覆盖参数：
```bash
roslaunch adma_ros_driver ADMA_truth_CAN.launch \
  dbc_file:=/ABS/PATH/TO/ADMA.dbc \
  channel:=0 \
  can_fd:=true \
  bitrate:=500000 \
  bitrate_fd:=2000000 \
  publish_on_frame:=1 \
  publish_hz:=20.0 \
  max_age_sec:=0.2 \
  require_all:=true \
  speed_can_id:=0x100 \
  yaw_rate_can_id:=0x101 \
  lat_accel_can_id:=0x102 \
  long_accel_can_id:=0x103 \
  speed_signal:=veh_speed \
  yaw_rate_signal:=yaw_rate \
  lat_accel_signal:=lat_accel \
  long_accel_signal:=long_accel
```

---

## 3) 参数说明
- `dbc_file`：DBC 路径（必须）。
- `channel`：Kvaser 通道号。
- `can_fd`：true = CAN-FD，false = 标准 CAN。
- `bitrate`：标准 CAN 波特率。
- `bitrate_fd`：CAN-FD 数据段波特率。
- `speed_can_id/yaw_rate_can_id/lat_accel_can_id/long_accel_can_id`：对应 CAN ID。
- `speed_signal/yaw_rate_signal/lat_accel_signal/long_accel_signal`：DBC 信号名。
- `publish_on_frame`：1=有帧就发；0=按 `publish_hz` 定时发。
- `publish_hz`：定时发布频率（`publish_on_frame=0` 时生效）。
- `max_age_sec`：新鲜度阈值（秒）。
- `require_all`：true=任一字段过期则整条不发；false=过期字段用 NaN 但仍发布。
- `use_freshness`：true=启用新鲜度判断；false=不判断，直接用缓存旧值。
- `truth_log`：true 时在终端打印 `adma/truth` 数值。
- `truth_log_hz`：终端打印频率（Hz）。

---

## 4) 话题输出
输出话题：`/adma/truth`
- 类型：`adma_ros_driver_msgs/AdmaTruth`
- 字段：
  - `actual_spd`
  - `yaw_rate`
  - `lat_accel`
  - `long_accel`
  - `yaw_rate_sign`

时间戳：系统时间 `rospy.Time.now()`。

---

## 8) 终端打印与单位换算示例
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

## 5) 新鲜度逻辑
- 每个信号都有独立的更新时间。
- 发布时用当前时间减去各自更新时间进行 `max_age_sec` 判断。
- `require_all=true`：任一信号过期 → 整条不发。
- `require_all=false`：过期字段写 `NaN`，其余照常发布。

---

## 6) 坐标系与符号
CAN 解码得到的数值仍然遵循 ADMA 的坐标定义。
如果你的车辆坐标系与 ADMA 不一致，需要自行做符号或轴向调整。
示例：
```
# ADMA Y 向左，车辆 Y 向右
lat_accel = -lat_accel
yaw_rate = -yaw_rate
yaw_rate_sign = (yaw_rate < 0) ? 1 : 0
```
```
# ADMA X 与车辆 X 方向相反
long_accel = -long_accel
```

---

## 7) 常见问题
1) 节点找不到：
   - 确保 `catkin_make` + `source devel/setup.bash`。
2) DBC 路径错误：
   - `dbc_file` 必须存在，否则节点退出。
3) 信号名不匹配：
   - DBC 内 signal 名必须与参数一致，否则字段一直是 NaN。
