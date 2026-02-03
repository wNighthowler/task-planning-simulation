import numpy
import pybullet as p
import pybullet_data
import time

# 连接到 PyBullet 仿真
p.connect(p.GUI)  # 使用图形界面
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 设置搜索路径

# 加载平面
plane_id = p.loadURDF("plane.urdf")

# 设置重力
p.setGravity(0, 0, -9.8)

# 加载一个机器人 (例如KUKA手臂)
robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# 获取机器人关节数量
num_joints = p.getNumJoints(robot_id)
print(f"机器人有 {num_joints} 个关节")

# # 让机器人移动：设置第一个关节的目标位置
for t in range(1000):
    target_position = 0.5 * numpy.sin(0.01 * t)  # 简单的正弦运动
    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=0,  # 控制第一个关节
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_position,
        force=500
    )
    p.stepSimulation()  # 仿真一步
    time.sleep(1.0 / 240.0)  # 保持仿真步长
# 断开连接
p.disconnect()
