# DWA Drone Avoidance

A ROS2 implementation of the Dynamic Window Approach (DWA) for obstacle avoidance in multirotor drones, featuring two different implementations for different use cases.

## Key Features

- ✅ **Two DWA implementations**: 
  - `dwa_vx_vy.py`: Full implementation with lateral movement (vx, vy) and angular velocity (w)
  - `dwa_vx_only.py`: Simplified version with only forward velocity (vx) and angular velocity (w)
- ✅ **True side-flight capability**: Multirotors can move laterally (left/right) without changing heading direction
- ✅ **FLU coordinate frame implementation**: Uses Forward-Left-Up body frame for intuitive velocity control
- ✅ **Obstacle avoidance with Lidar**: Real-time obstacle detection and avoidance
- ✅ **Dynamic mode switching**: Automatically enters/exits DWA mode based on obstacle distance
- ✅ **PX4 integration**: Works with PX4 flight stack for drone control

## Project Structure

```
dwa-drone-avoidance/
├── dwa_vx_vy.py          # Full DWA implementation with lateral movement (vx, vy)
├── dwa_vx_only.py        # Simplified DWA implementation (only vx, w)
├── README.md
└── ...
```

## Why Two Implementations?

| Feature | `dwa_vx_vy.py` | `dwa_vx_only.py` |
|---------|----------------|------------------|
| **Lateral Movement** | ✅ Yes | ❌ No |
| **Computational Load** | Higher | Lower (~50% less) |
| **Avoidance Flexibility** | High (can move left/right) | Limited (only forward/backward) |
| **Use Case** | Complex environments, full functionality | Simple environments, resource-constrained systems |
| **Default Target** | [10.0, -15.0, 5.0] | [10.0, -10.0, 5.0] |

## How It Works

The system operates in the drone's body coordinate frame (FLU: Forward-Left-Up), where:
- `vx` = forward velocity (along drone's nose)
- `vy` = lateral velocity (to the left of drone)
- `w` = angular velocity (yaw rate)

Unlike fixed-wing aircraft, multirotors can control `vx` and `vy` independently without changing heading, enabling true side-flight capability.

## Requirements

- ROS2 
- PX4 SITL (Software-in-the-Loop) with drone model
- Lidar simulation 
``



## Configuration

Both implementations use similar parameters but with slight differences:

```python
# Key parameters for both implementations
safe_dist = 3.0  # Safety distance (triggers DWA when obstacle < this)
max_speed = 2.0  # Max linear speed (m/s)
max_w = math.pi / 2  # Max angular speed (rad/s)
v_samples = 10  # Velocity samples
w_samples = 20  # Angular velocity samples
alpha, beta, gamma = 0.2, 0.2, 0.6  # Scoring weights
```

For `dwa_vx_only.py`, the parameters are slightly adjusted:
```python
# Simplified version parameters
safe_dist = 3.5  # Slightly higher safety distance
max_v = 2.0  # Max forward speed
alpha, beta, gamma = 0.5, 0.2, 0.3  # Different scoring weights
```

## Key Algorithms

- **Dynamic Window Approach (DWA)**: For local trajectory optimization
- **FLU Coordinate Transformation**: For intuitive body-frame velocity control
- **KDTree Obstacle Query**: For efficient collision checking
- **Obstacle-Aware Rotation**: When no valid trajectory is found

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.



# 项目简介

## 中文版

这是一个基于ROS2的无人机避障系统，包含两种DWA（动态窗口法）实现，适用于不同场景需求。

**项目特点**
- ✅ **两种实现方式**：
  - `dwa_vx_vy.py`：完整版，支持前进速度(vx)和侧向速度(vy)的避障
  - `dwa_vx_only.py`：简化版，仅考虑前进速度(vx)和角速度(w)
- ✅ **真正的侧向移动能力**：多旋翼无人机可以在不改变航向的情况下横向移动
- ✅ **FLU坐标系实现**：使用前进-左-上坐标系进行直观的速度控制
- ✅ **障碍物检测与避障**：实时激光雷达障碍物检测
- ✅ **动态模式切换**：根据障碍物距离智能进入/退出DWA模式
- ✅ **PX4集成**：与PX4飞行栈配合工作



## 为什么有两种实现方式？

| 特性 | `dwa_vx_vy.py` | `dwa_vx_only.py` |
|------|----------------|------------------|
| **侧向移动** | ✅ 支持 | ❌ 不支持 |
| **计算负载** | 较高 | 较低（约减少50%） |
| **避障灵活性** | 高（可向左右移动） | 有限（仅能前进/后退） |
| **适用场景** | 复杂环境，需要完整功能 | 简单环境，计算资源受限的设备 |
| **默认目标点** | [10.0, -15.0, 5.0] | [10.0, -10.0, 5.0] |

## 工作原理

系统在无人机的机体坐标系（FLU：前进-左-上）中运行：
- `vx` = 前进速度（沿无人机机头方向）
- `vy` = 侧向速度（无人机左侧方向）
- `w` = 角速度（偏航率）

与固定翼飞机不同，多旋翼无人机可以独立控制`vx`和`vy`而不改变航向，从而实现真正的侧向移动能力。

## 依赖要求

- ROS2
- PX4 SITL（软件在环）带无人机模型
- 激光雷达仿真
- 



## 配置

两种实现使用类似的参数，但有细微差别：

```python
# 两种实现共用的关键参数
safe_dist = 3.0  # 安全距离（障碍物距离小于此值触发DWA）
max_speed = 2.0  # 最大线速度（m/s）
max_w = math.pi / 2  # 最大角速度（rad/s）
v_samples = 10  # 速度采样数
w_samples = 20  # 角速度采样数
alpha, beta, gamma = 0.2, 0.2, 0.6  # 评分权重
```

对于`dwa_vx_only.py`，参数略有调整：
```python
# 简化版参数
safe_dist = 3.5  # 稍高的安全距离
max_v = 2.0  # 最大前进速度
alpha, beta, gamma = 0.5, 0.2, 0.3  # 不同的评分权重
```

## 关键算法

- **动态窗口法（DWA）**：用于局部轨迹优化
- **FLU坐标变换**：用于直观的机体速度控制
- **KDTree障碍物查询**：用于高效的碰撞检查
- **障碍物感知旋转**：当没有有效轨迹时

## 许可证
'''
================================================================================
作者/Author: 刘永学/Liu Yongxue
邮箱/Email: 805110687@qq.com
QQ群：1080856708

版权声明/Copyright Notice:
© All rights reserved. 保留所有权利。

使用许可/Usage License:
仅供个人使用，禁止商业用途。
For personal use only. Commercial use is prohibited.
================================================================================
'''


本项目根据Apache 2.0许可证发布，请参阅[LICENSE](LICENSE)文件获取详情。


**作者**  
刘永学 (Liu Yongxue)  
邮箱：805110687@qq.com
