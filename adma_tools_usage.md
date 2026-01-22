# ADMA 工具使用说明（adma_tools_cpp / adma_tools_py）

## 前置准备
- 已将 `adma_ros_driver`、`adma_ros_driver_msgs`、`adma_tools_cpp`、`adma_tools_py` 放入工作区并通过 `catkin_make` 编译，终端已 `source devel/setup.bash`。
- ETH 驱动：已运行 `<include file="$(find adma_ros_driver)/launch/ADMA_pub_Ethernet.launch">` 并发布 `/adma/data_raw`、`/adma/data_scaled`、`/adma/status`。
- 数据存放目录存在且可写（下文示例使用 `/home/$USER/adma_logs`，可自行替换）。

---

## adma_tools_cpp
### 1) 从 rosbag 生成 .gsdb
用途：将 ADMA 原始 UDP 数据（`/adma/data_raw`）转成官方 Post-Processing 工具使用的 `.gsdb`。

步骤：
1. 若尚未录制 bag，先录制原始数据：
   ```bash
   rosbag record -O /home/$USER/adma_logs/adma_raw.bag /adma/data_raw
   ```
2. 运行转换：
   ```bash
   roslaunch adma_tools_cpp bag2gsdb.launch rosbag_file:=/home/$USER/adma_logs/adma_raw.bag
   ```
   - 内部会播放 rosbag 并启动 `bag2gsdb_converter` 订阅 `adma/data_raw`。
   - `.gsdb` 文件会生成在同目录。

### 2) 回放 .gsdb
用途：把已有 `.gsdb` 按 ADMA UDP 协议回放，用于仿真/复现。

命令：
```bash
roslaunch adma_tools_cpp gsdb_replay.launch \
  gsdb_file:=/home/$USER/adma_logs/adma_raw.gsdb \
  protocol_version:=v3.3.4 \
  port:=1040 ip_address:=localhost frequency:=100
```
- `gsdb_server` 会按设定频率通过 UDP 发送数据；同一 launch 会 include ADMA ETH 驱动监听 `localhost:1040`，你可在 `/adma/*` 话题看到回放数据。
- 如需同时录制回放到 rosbag，可在 `gsdb_replay.launch` 的 include 里把 `record_rosbag` 设为 `true`。

---

## adma_tools_py
### 导出 CSV
用途：将解析后的 `/adma/data_scaled` 与 `/adma/status` 同步写入 CSV 便于分析。

命令：
```bash
rosrun adma_tools_py ros2csv_converter.py
```
- 运行后在当前终端目录生成 `recorded_data.csv`，包含全部字段（含 POI 与状态位）。
- 如需自定义文件名，可在脚本 `ros2csv_converter.py` 中修改 `self.filename`。

---

## 快速检查
- ETH 驱动运行：`rostopic echo /adma/data_scaled` 观察数据刷新与时间戳。
- gsdb 回放：命令启动后 `rostopic echo /adma/status` 应有数据流动。
- CSV 导出：脚本运行后应在终端目录出现/增长 `recorded_data.csv`。
