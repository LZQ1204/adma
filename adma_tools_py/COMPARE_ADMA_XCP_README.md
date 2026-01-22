# ADMA vs XCP 对比（rosbag）

该脚本用于在同一个 rosbag 内，对比 ADMA 真值话题与 XCP 话题的数值。
支持时间戳对齐、单位转换和 CSV 导出。

文件：
- `adma_tools_py/scripts/compare_adma_xcp.py`

---

## 1) 前置条件
1) 已 source ROS 环境：
   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash
   source devel/setup.bash
   ```
2) rosbag 中包含 ADMA 与 XCP 两个话题。

---

## 2) 基本用法
```bash
rosrun adma_tools_py compare_adma_xcp.py \
  --bag /path/to/data.bag \
  --adma-topic /adma/truth \
  --adma-field actual_spd \
  --xcp-topic /xcp/vehicle \
  --xcp-field speed
```

输出指标：
- 平均误差（Mean error）
- MAE
- RMSE
- 最大绝对误差（Max abs error）

---

## 3) 一次对比多个字段
字段列表用逗号分隔（两边数量必须一致）：
```bash
rosrun adma_tools_py compare_adma_xcp.py \
  --bag /path/to/data.bag \
  --adma-topic /adma/truth \
  --adma-field actual_spd,yaw_rate,lat_accel,long_accel \
  --xcp-topic /xcp/vehicle \
  --xcp-field speed,yaw_rate,lat_accel,long_accel
```

### 带单位转换的例子
```bash
rosrun adma_tools_py compare_adma_xcp.py \
  --bag /path/to/data.bag \
  --adma-topic /adma/truth \
  --adma-field actual_spd,yaw_rate,lat_accel,long_accel \
  --xcp-topic /xcp/vehicle \
  --xcp-field speed,yaw_rate,lat_accel,long_accel \
  --adma-scale 1,1,1,1 --adma-offset 0,0,0,0 \
  --xcp-scale 0.277777,1,1,1 --xcp-offset 0,0,0,0
```
说明：
- `adma-field` 与 `xcp-field` 一一对应。
- `xcp-scale` 第一个值把速度从 km/h 转成 m/s（0.277777）。
- 其余字段单位一致，所以 scale=1，offset=0。

---

## 4) 时间对齐
对齐方式：
- `nearest`（默认）：每个 XCP 点找最近的 ADMA 点。
- `linear`：在两个 ADMA 点之间做线性插值。

例子（最近邻 + 最大时间差 0.02s）：
```bash
rosrun adma_tools_py compare_adma_xcp.py \
  --bag /path/to/data.bag \
  --adma-topic /adma/truth \
  --adma-field yaw_rate \
  --xcp-topic /xcp/vehicle \
  --xcp-field yaw_rate \
  --method nearest \
  --max-dt 0.02
```

---

## 5) 时间戳来源
默认优先使用 `header.stamp`，否则使用 bag 时间。
可强制指定：
```bash
--time-source header
--time-source bag
--time-source header_or_bag
```

---

## 6) 单位转换
用 scale/offset 来统一单位（支持单值或列表）：
```bash
# 例：ADMA 速度为 m/s，XCP 为 km/h
--adma-scale 1.0 --adma-offset 0.0 \
--xcp-scale 0.277777 --xcp-offset 0.0

# 例：4 个字段对应的缩放/偏移
--adma-scale 1,1,1,1 --adma-offset 0,0,0,0 \
--xcp-scale 0.277777,1,1,1 --xcp-offset 0,0,0,0
```

---

## 7) 导出 CSV
```bash
rosrun adma_tools_py compare_adma_xcp.py \
  --bag /path/to/data.bag \
  --adma-topic /adma/truth \
  --adma-field actual_spd \
  --xcp-topic /xcp/vehicle \
  --xcp-field speed \
  --output-csv /path/to/compare.csv
```

CSV 列：
- `adma_field`, `xcp_field`, `t_xcp`, `t_adma`, `adma`, `xcp`, `error`, `dt`

---

## 8) 备注
- 字段数量必须一致。
- 某字段缺失时，该字段对会被跳过。

---

## 9) 脚本内部流程与函数说明（含例子）
整体流程：解析参数 → 读取 bag → 按字段缓存 (t, value) → 时间对齐 → 误差统计 → 输出 CSV。

### 关键函数
1) `parse_args()`：解析命令行参数（话题、字段列表、对齐方式、时间戳来源、单位转换等）。

2) `split_list(value)`：把逗号分隔字段拆成列表。  
例：`"actual_spd,yaw_rate"` → `["actual_spd", "yaw_rate"]`。

3) `parse_float_list(value, count, name)`：把 `scale/offset` 解析成列表并对齐字段数量。  
例：`"0.277777"` + `count=4` → `[0.277777, 0.277777, 0.277777, 0.277777]`。

4) `get_msg_time(msg, bag_time, source)`：决定时间戳来源。  
例：`header_or_bag` 时优先用 `header.stamp`，否则用 bag 时间。

5) `get_field(msg, path)`：根据“字段路径字符串”取值，支持嵌套与数组。  
例：  
- `path="angular_velocity.x"` → 等价于 `msg.angular_velocity.x`  
- `path="covariance[3]"` → 等价于 `msg.covariance[3]`

6) `apply_scale(value, scale, offset)`：单位转换，执行 `value * scale + offset`。

7) `align_nearest(...)`：最近邻对齐。  
例：XCP 时间 0.015，ADMA 有 0.01 和 0.02，选更近的一个。

8) `align_linear(...)`：线性插值对齐。  
例：ADMA (0.00,10)、(0.02,14)，XCP 0.01 → 插值 12。

### main 流程例子
假设命令：
```bash
rosrun adma_tools_py compare_adma_xcp.py \
  --bag demo.bag \
  --adma-topic /adma/truth \
  --adma-field actual_spd,yaw_rate \
  --xcp-topic /xcp/vehicle \
  --xcp-field speed,yaw_rate \
  --method nearest \
  --max-dt 0.02
```

假设数据：
- ADMA actual_spd：`(0.00,10) (0.02,12) (0.04,14)`
- XCP speed：`(0.015,11.5) (0.035,13.5)`
- ADMA yaw_rate：`(0.00,0.5) (0.02,0.6)`
- XCP yaw_rate：`(0.01,0.45) (0.03,0.55)`

处理结果：
- `actual_spd vs speed` 对齐后匹配两对 → 统计 Mean/MAE/RMSE/Max。
- `yaw_rate vs yaw_rate` 同理对齐并统计。
- CSV 输出每一对匹配样本（每行含时间、ADMA、XCP、误差、dt）。

补充说明：  
对齐前会执行 `adma_list.sort(key=lambda x: x[0])`、`xcp_list.sort(key=lambda x: x[0])`，  
确保按时间戳升序排序，便于二分查找与插值对齐。
