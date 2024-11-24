'''
Author: He Weijie 129854778+IntronRewrite@users.noreply.github.com
Date: 2024-11-24 14:34:34
LastEditors: He Weijie 129854778+IntronRewrite@users.noreply.github.com
LastEditTime: 2024-11-24 16:00:28
FilePath: \cloud\2.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# def compute_average_distance(pcd):
#     distances = pcd.compute_nearest_neighbor_distance()
#     avg_dist = np.mean(distances)
#     return avg_dist


pcd_src = o3d.io.read_point_cloud(r'..\src_model.ply')
print('读取src点云数据成功')
# pcd_ref = o3d.io.read_point_cloud(r'..\ref_model.ply')
# print('读取ref点云数据成功')

#对src聚类
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
# 可视化聚类结果
o3d.visualization.draw_geometries([pcd_src])
print('聚类结果可视化成功')


# 只保留点最多的一个聚类
largest_cluster_idx = np.argmax(np.bincount(labels[labels >= 0]))
pcd_src = pcd_src.select_by_index(np.where(labels == largest_cluster_idx)[0])
print('保留点最多的一个聚类成功')
# 可视化点云
o3d.visualization.draw_geometries([pcd_src])
print('点云可视化成功')