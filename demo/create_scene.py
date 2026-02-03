import numpy
import pybullet as p
import pybullet_data
import time

from pybullet_tools.utils import create_box, set_point, wait_if_gui

# 连接到 PyBullet 仿真
p.connect(p.GUI)  # 使用图形界面
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 设置搜索路径

# 加载平面
plane_id = p.loadURDF("plane.urdf")

# 设置重力
p.setGravity(0, 0, -9.8)

h = 3
wall_h = 3

# 房间的墙
# 外墙
wall1 = create_box(0.2, 6, wall_h)
set_point(wall1, (0, 3, h/2))

wall2 = create_box(6, 0.2, wall_h)
set_point(wall2, (3, 0, h/2))

wall3 = create_box(10, 0.2, wall_h)
set_point(wall3, (11, 0, h/2))

wall4 = create_box(8, 0.2, wall_h)
set_point(wall4, (20, 0, h/2))

wall5 = create_box(0.2, 8, wall_h)
set_point(wall5, (24, 4, h/2))

wall6 = create_box(4, 0.2, wall_h)
set_point(wall6, (22, 8, h/2))

wall7 = create_box(0.2, 4, wall_h)
set_point(wall7, (20, 10, h/2))

wall8 = create_box(25, 0.2, wall_h)
set_point(wall8, (7.5, 12, h/2))

wall9 = create_box(0.2, 6, wall_h)
set_point(wall9, (-5, 9, h/2))

wall10 = create_box(5, 0.2, wall_h)
set_point(wall10, (-2.5, 6, h/2))
# 内墙
tmp = 2
wall11 = create_box(0.2, tmp, wall_h)
set_point(wall11, (0, 11, h/2))

wall12 = create_box(0.2, tmp, wall_h)
set_point(wall12, (0, 7, h/2))

wall13 = create_box(0.2, tmp, wall_h)
set_point(wall13, (11, 11, h/2))

wall14 = create_box(0.2, tmp, wall_h)
set_point(wall14, (11, 7, h/2))

wall15 = create_box(2, 0.2, wall_h)
set_point(wall15, (1, 6, h/2))

wall16 = create_box(3.5, 0.2, wall_h)
set_point(wall16, (5.75, 6, h/2))

wall17 = create_box(3, 0.2, wall_h)
set_point(wall17, (11, 6, h/2))

wall18 = create_box(9.5, 0.2, wall_h)
set_point(wall18, (19.25, 6, h/2))

wall19 = create_box(0.2, 2, wall_h)
set_point(wall19, (6, 5, h/2))

wall20 = create_box(0.2, 2, wall_h)
set_point(wall20, (6, 1, h/2))

wall21 = create_box(0.2, 2, wall_h)
set_point(wall21, (16, 5, h/2))

wall22 = create_box(0.2, 2, wall_h)
set_point(wall22, (16, 1, h/2))

# wall_lists = []
# wall_lists.append(wall1)
# wall_lists.append(wall2)
# wall_lists.append(wall3)
# wall_lists.append(wall4)
# wall_lists.append(wall5)
# wall_lists.append(wall6)
# wall_lists.append(wall7)
# wall_lists.append(wall8)
# wall_lists.append(wall9)
# wall_lists.append(wall10)
# wall_lists.append(wall11)
# wall_lists.append(wall12)
# wall_lists.append(wall13)
# wall_lists.append(wall14)
# wall_lists.append(wall15)
# wall_lists.append(wall16)
# wall_lists.append(wall17)
# wall_lists.append(wall18)
# wall_lists.append(wall19)
# wall_lists.append(wall20)
# wall_lists.append(wall21)
# wall_lists.append(wall22)

# filenames = []
# for i in range(len(wall_lists)):
#     filenames.append('wall'+str(i)+'.obj')
wait_if_gui()
# 断开连接
p.disconnect()

