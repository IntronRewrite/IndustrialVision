'''
Author: He Weijie 129854778+IntronRewrite@users.noreply.github.com
Date: 2024-11-24 14:21:57
LastEditors: IntronRewrite weijiehe@sdust.edu.com
LastEditTime: 2024-11-25 03:34:05
FilePath: \IndustrialVision\plan1\1.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import time
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


def compute_average_distance(pcd):
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    return avg_dist


pcd_src = o3d.io.read_point_cloud(r'..\src_model.ply')
print('读取src点云数据成功')
pcd_ref = o3d.io.read_point_cloud(r'..\ref_model.ply')
print('读取ref点云数据成功')

start_time = time.time()
#对src聚类
# avg_dist = compute_average_distance(pcd_src)
avg_dist = 1.8126511386464113
print(f"点云的平均距离: {avg_dist}")

# 去除pcd_src的离群点
# 欧式聚类pcd_src
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd_src.cluster_dbscan(eps=avg_dist*4.2, min_points=1, print_progress=True))
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")

# 为每个聚类分配颜色
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd_src.colors = o3d.utility.Vector3dVector(colors[:, :3])
print('欧式聚类成功')
# # 可视化聚类结果
# o3d.visualization.draw_geometries([pcd_src])
# print('聚类结果可视化成功')


# 只保留点最多的一个聚类
largest_cluster_idx = np.argmax(np.bincount(labels[labels >= 0]))
pcd_src = pcd_src.select_by_index(np.where(labels == largest_cluster_idx)[0])
print('保留点最多的一个聚类成功')
# # 可视化点云
# o3d.visualization.draw_geometries([pcd_src])
# print('点云可视化成功')
end_time = time.time()
print('聚类时间{:.2f}秒'.format(end_time - start_time))

#粗配准
start_time = time.time()
src_oriented_bounding_box = pcd_src.get_oriented_bounding_box()
src_oriented_bounding_box.color = (0, 1, 0)
print('获取src定向边界框成功')
# o3d.visualization.draw_geometries([pcd_src, src_oriented_bounding_box])
print('src可视化成功')
ref_oriented_bounding_box = pcd_ref.get_oriented_bounding_box()
ref_oriented_bounding_box.color = (1, 0, 0)
print('获取ref定向边界框成功')
# o3d.visualization.draw_geometries([pcd_ref, ref_oriented_bounding_box])
# print('ref可视化成功')

# Set pcd_ref color to a single color (e.g., red)
pcd_ref.paint_uniform_color([1, 0, 0])

# 获取src定向边界框的中心
src_center = src_oriented_bounding_box.get_center()
# 获取ref定向边界框的中心
ref_center = ref_oriented_bounding_box.get_center()


# 将src点云和定向边界框移动到原点
pcd_src.translate(-src_center)
src_oriented_bounding_box.translate(-src_center)
# 将ref点云和定向边界框移动到原点
pcd_ref.translate(-ref_center)
ref_oriented_bounding_box.translate(-ref_center)


o3d.visualization.draw_geometries([pcd_src, src_oriented_bounding_box,pcd_ref, ref_oriented_bounding_box])
print('ref点云和定向边界框已移动到原点')
print('src点云和定向边界框已移动到原点')



# 计算src和ref定向边界框的旋转矩阵
src_rotation = src_oriented_bounding_box.R
ref_rotation = ref_oriented_bounding_box.R
print(src_rotation)
print(ref_rotation)


# 将src点云和定向边界框旋转到与ref对齐
pcd_src.rotate(ref_rotation @ np.linalg.inv(src_rotation), center=(0, 0, 0))
src_oriented_bounding_box.rotate(ref_rotation @ np.linalg.inv(src_rotation), center=(0, 0, 0))
end_time = time.time()
print('聚类时间{:.2f}秒'.format(end_time - start_time))

o3d.visualization.draw_geometries([pcd_src, src_oriented_bounding_box, pcd_ref, ref_oriented_bounding_box])
print('src点云和定向边界框已旋转对齐ref')


# 精配准src和ref点云
start_time = time.time()
threshold = 1 # 配准距离阈值
trans_init = np.eye(4)  # 初始变换矩阵

# 使用ICP算法进行精配准
reg_p2p = o3d.pipelines.registration.registration_icp(
    pcd_src, pcd_ref, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

# 获取精配准后的变换矩阵
transformation_icp = reg_p2p.transformation
print("ICP 变换矩阵:")
print(transformation_icp)

# 将src点云应用精配准后的变换矩阵
pcd_src.transform(transformation_icp)
end_time = time.time()
print('聚类时间{:.2f}秒'.format(end_time - start_time))

# 可视化精配准结果
o3d.visualization.draw_geometries([pcd_src, pcd_ref])
print('src点云和ref点云精配准成功')
