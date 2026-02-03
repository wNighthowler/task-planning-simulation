import pybullet as p
import pybullet_data
import numpy as np
import time

# 初始化仿真
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载场景和机器人
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 0.5])

# 设置轨迹 (例如圆周轨迹)
trajectory = []
num_points = 50
radius = 2
for angle in np.linspace(0, 2 * np.pi, num_points):
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0.5  # 固定高度
    trajectory.append((x, y, z))

# 保存影像的物理表现: 透明复制
shadow_objects = []

# 仿真主循环
for idx, point in enumerate(trajectory):
    # 移动机器人到轨迹点
    position = point
    orientation = [0, 0, 0, 1]  # 固定方向
    p.resetBasePositionAndOrientation(robot_id, position, orientation)
    
    # 每隔一定步长留影（例如每5步）
    if idx % 5 == 0:
        # 复制机器人并变为透明影像
        shadow_id = p.loadURDF("r2d2.urdf", position, orientation)
        p.changeVisualShape(shadow_id, -1, rgbaColor=[1, 0, 0, 0.3])  # 设置透明
        shadow_objects.append(shadow_id)
    
    # 小等待以便观察
    time.sleep(0.1)

# 保持场景显示
while True:
    time.sleep(1)
