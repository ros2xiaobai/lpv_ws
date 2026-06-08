# GPS Spoofing Attack Simulation - Usage Guide

## 概述

本工具集用于在Gazebo/PX4环境中模拟GPS欺骗攻击，并生成定量评估报告。

---

## 快速开始

### 方法1：使用便捷脚本（推荐）

```bash
cd /home/lxx/LPV_ws
source devel/setup.bash

# 运行moderate场景，自动生成指标和视频
./src/lpv_attack_sim/scripts/run_gps_spoof_attack.sh --scenario moderate --all

# 只运行仿真（不生成后处理）
./src/lpv_attack_sim/scripts/run_gps_spoof_attack.sh --scenario severe

# 查看帮助
./src/lpv_attack_sim/scripts/run_gps_spoof_attack.sh --help
```

### 方法2：手动步骤

```bash
# 1. 更新配置
python3 src/lpv_attack_sim/scripts/update_gps_spoof_config.py --scenario moderate

# 2. 运行仿真
roslaunch lpv_attack_sim gps_spoof_deviation.launch

# 3. 生成指标（仿真完成后）
python3 src/lpv_attack_sim/scripts/generate_attack_metrics.py \
    src/lpv_attack_sim/results/gps_spoof_attack_YYYYMMDD_HHMMSS.csv

# 4. 生成3D视频
python3 src/lpv_attack_sim/scripts/make_gps_spoof_video_3d.py \
    --csv src/lpv_attack_sim/results/gps_spoof_attack_YYYYMMDD_HHMMSS.csv
```

---

## 预定义攻击场景

在 `config/gps_spoof_params.yaml` 中定义了4个场景：

| 场景 | 偏置 (m) | 漂移 (m/s) | 攻击时间 (s) | 特点 |
|------|---------|-----------|-------------|------|
| **mild** | (1.0, 0.5) | (0.01, 0) | 30-50 | 轻度攻击，偏离约1-1.5m |
| **moderate** | (2.0, 1.5) | (0.03, 0) | 30-65 | 中度攻击，偏离约3m |
| **severe** | (3.5, 2.5) | (0.05, 0) | 25-70 | 重度攻击，偏离约5-6m |
| **asymmetric** | (1.5, 3.0) | (0.02, -0.04) | 30-65 | 非对称攻击，Y向偏离更大 |

---

## 自定义攻击参数

### 编辑YAML配置文件

编辑 `src/lpv_attack_sim/config/gps_spoof_params.yaml`：

```yaml
attack:
  offset_x: 2.5        # X方向偏置 (m)
  offset_y: 1.8        # Y方向偏置 (m)
  offset_z: 0.0        # Z方向偏置 (m)
  drift_x: 0.04        # X方向漂移率 (m/s)
  drift_y: 0.0         # Y方向漂移率 (m/s)
  start_time: 28.0     # 起飞后开始时间 (s)
  end_time: 68.0       # 起飞后结束时间 (s)
  smooth_duration: 4.0 # 平滑过渡时长 (s)
  takeoff_z_threshold: 1.5  # 起飞检测高度 (m)
```

保存后，运行配置更新脚本：

```bash
python3 src/lpv_attack_sim/scripts/update_gps_spoof_config.py
```

---

## 批量场景对比

自动运行多个场景并生成对比表格：

```bash
cd /home/lxx/LPV_ws
source devel/setup.bash
./src/lpv_attack_sim/scripts/batch_attack_scenarios.sh
```

输出示例：
```
==========================================================================================
GPS SPOOFING ATTACK - SCENARIO COMPARISON TABLE
==========================================================================================

Metric                              |         Mild |     Moderate |       Severe
------------------------------------------------------------------------------------------
Baseline mean error (m)             |        0.180 |        3.578 |        0.182
Attack duration (s)                 |        20.00 |        34.96 |        45.00
Max deviation (m)                   |        1.523 |        3.151 |        5.832
Mean deviation (m)                  |        1.124 |        2.200 |        4.256
Final deviation (m)                 |        1.487 |        3.126 |        5.621
Max acceleration (m/s²)             |        0.185 |        0.262 |        0.421
Recovery time (s)                   |         6.23 |        12.01 |        18.45
Final error after recovery (m)      |        0.321 |        0.495 |        0.687
Attack effectiveness (×baseline)    |         8.46 |         0.88 |        32.05
==========================================================================================
```

---

## 输出文件说明

每次仿真会生成以下文件（位于 `src/lpv_attack_sim/results/`）：

1. **CSV数据日志**：`gps_spoof_attack_YYYYMMDD_HHMMSS.csv`
   - 包含时间序列数据：位置、速度、姿态、误差等

2. **定量指标JSON**：`*_metrics.json`
   - 机器可读的结构化指标

3. **定量指标报告**：`*_metrics.txt`
   - 人类可读的文本报告

4. **3D轨迹视频**：`*_3d_deviation.mp4`
   - 三维可视化动画

---

## 定量指标说明

### Baseline Phase（基准阶段）
- `mean_tracking_error_m`: 未受攻击时的平均跟踪误差
- `max_tracking_error_m`: 未受攻击时的最大跟踪误差

### Attack Phase（攻击阶段）
- `max_deviation_m`: 最大偏离距离
- `mean_deviation_m`: 平均偏离距离
- `final_deviation_m`: 攻击结束时的偏离距离
- `max_acceleration_m_s2`: 最大加速度（轨迹平滑度指标）

### Post-Attack Recovery（攻击后恢复）
- `recovery_time_s`: 恢复到误差<0.5m所需时间
- `final_error_m`: 恢复阶段结束时的误差

### Overall Summary（总体摘要）
- `attack_effectiveness`: 攻击有效性（攻击偏离 / 基准误差）

---

## 故障排查

### 问题1：仿真卡住或无人机不起飞
```bash
# 手动清理残留进程
pkill -9 -f "gzserver|gzclient|px4|mavros|roslaunch|rosmaster"
sleep 3

# 重新启动
./src/lpv_attack_sim/scripts/run_gps_spoof_attack.sh --scenario moderate
```

### 问题2：CSV文件未生成
检查Python脚本是否正常运行：
```bash
rosnode list | grep gps_spoof_attack_demo
rostopic echo /iris_0/mavros/local_position/pose
```

### 问题3：SDF参数未更新
手动检查SDF文件：
```bash
grep "gpsSpoofOffset" /home/lxx/PX4_Firmware/Tools/sitl_gazebo/models/gps_spoof/gps_spoof.sdf
```

确保在启动仿真前运行了配置更新脚本。

---

## 工作原理

1. **GPS插件层注入**：修改后的 `libgazebo_gps_plugin.so` 在传感器层直接污染GPS数据
2. **起飞检测触发**：攻击时间窗口锚定在"起飞后N秒"，与实际飞行阶段对齐
3. **平滑过渡**：使用cubic多项式实现渐入/渐出，避免EKF跳变
4. **坐标系一致性**：所有日志统一使用本地ENU坐标（原点=出生点）

---

## 引用

如果在论文中使用本工具，请引用：
```
[你的论文标题]
[作者]
[会议/期刊]
```

---

## 技术支持

- 问题反馈：[GitHub Issues链接]
- 文档：本README
- 配置文件：`config/gps_spoof_params.yaml`
