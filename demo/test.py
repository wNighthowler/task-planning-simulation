import pybullet as p
import pybullet_data
import numpy as np

# 启动PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 加载默认数据路径

# 加载已有模型 (可以是任何URDF文件)
model_id = p.loadURDF("r2d2.urdf")  # 使用PyBullet默认路径下的r2d2模型

# 获取模型的顶点数据
mesh_data = p.getMeshData(model_id, 0)  # 第一个子模型，索引为0

# 提取顶点和面片数据
vertices = np.array(mesh_data[0])  # 顶点坐标
indices = np.array(mesh_data[1])   # 面片索引

# 将数据转换成.obj格式
def save_obj(vertices, indices, filename="model.obj"):
    with open(filename, "w") as obj_file:
        # 写入顶点
        for vertex in vertices:
            obj_file.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")
        
        # 写入面片
        for index in indices:
            obj_file.write(f"f {index[0]+1} {index[1]+1} {index[2]+1}\n")

# 保存为obj文件
save_obj(vertices, indices)

# 关闭PyBullet连接
p.disconnect()
