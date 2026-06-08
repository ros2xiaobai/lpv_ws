# LPV Attack Simulation

本工作区用于在本机 `PX4 + ROS + XTDrone` 环境下进行单机虚假数据注入攻击轨迹偏离仿真。

当前版本实现的是“控制指令/执行器等效 FDI 攻击”。其思想是对发送给 PX4 的位置期望值叠加攻击项：

```text
p_ref_attack(k) = p_ref_nominal(k) + a_u(k)
```

攻击前，无人机跟踪名义参考轨迹；攻击窗口内，节点发布被篡改的位置指令，因此真实轨迹会向被攻击指令偏移，从而体现 FDI 攻击导致无人机偏离原任务轨迹的现象。

节点会记录以下数据到 CSV：

- 名义参考轨迹；
- 被攻击后的参考轨迹；
- 无人机实际轨迹；
- 攻击信号；
- 速度、姿态角；
- 实际位置相对名义轨迹和受攻击轨迹的误差。

## Build

```bash
cd ~/LPV_ws
catkin_make
source devel/setup.bash
```

## Run

推荐直接使用工作区根目录下的脚本：

```bash
cd ~/LPV_ws
./run_attack_sim.sh
```

等价命令为：

```bash
roslaunch lpv_attack_sim single_attack_deviation.launch gui:=true record_bag:=true
```

如果你已经单独启动了 PX4/Gazebo/MAVROS，只运行攻击仿真节点：

```bash
cd ~/LPV_ws
./run_attack_node_only.sh
```

等价命令为：

```bash
roslaunch lpv_attack_sim single_attack_deviation.launch start_px4:=false record_bag:=true
```

## Plot

仿真结束后画图：

```bash
cd ~/LPV_ws
./plot_attack_results.sh
```

等价命令为：

```bash
source ~/LPV_ws/devel/setup.bash
rosrun lpv_attack_sim plot_attack_results.py
```

默认读取 `results/` 中最新的 CSV 文件，并生成：

- 三维轨迹对比图；
- x/y/z 位置响应图；
- 攻击信号与轨迹误差图。

## Attack Parameters

攻击参数位于：

```text
~/LPV_ws/src/lpv_attack_sim/config/setpoint_attack.yaml
```

默认设置为：

```text
0 s -- 4 s: 起飞/暖机
4 s -- 22 s: 无攻击名义轨迹
22 s -- 57 s: 注入 FDI 攻击
57 s -- 69 s: 攻击后观察
```

默认攻击形式为：

```text
a_x(t) = 1.2 + 0.015(t - t_attack)
a_y(t) = 0.8
a_z(t) = 0
```

这对应论文中“攻击变化率有界”的执行器/控制输入等效攻击场景。
