import target_detect
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

def fix_points(points_array):

    try:
        # 确保 points_array 是一个二维数组
        if not points_array:
            print("Error: points_array is empty!")
            return
        
        # 转换为 NumPy 数组，确保它是二维的
        points = np.array(points_array)

        if points.ndim != 2 or points.shape[1] != 2:
            print("Error: points_array should be a list of 2D coordinates.")
            return

        # 使用 DBSCAN 聚类
        db = DBSCAN(eps=1.0, min_samples=4).fit(points)

        # 获取聚类标签
        labels = db.labels_

        # 将噪声点（标签为 -1）过滤掉
        filtered_points = points[labels != -1]

        # 绘制聚类结果
        '''
        plt.scatter(points[:, 0], points[:, 1], c=labels, cmap='rainbow')
        plt.scatter(filtered_points[:, 0], filtered_points[:, 1], color='black', marker='x')  # 去噪后的点
        plt.title('DBSCAN Clustering')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()
        '''
        # 输出新的聚类点
        min_cluster_size = 3  # 可以根据需求调整阈值
        new_points = []
        for label in set(labels):
            if label != -1:  # 排除噪声点
                cluster_points = points[labels == label]
                if len(cluster_points) >= min_cluster_size:  # 只保留簇大小大于阈值的簇
                    new_point = np.mean(cluster_points, axis=0)
                    new_points.append(new_point)
        '''
        print("聚类后的新点：")
        for point in new_points:
            print(f"x: {point[0]}, y: {point[1]}")
        '''
        return new_points
    except:
        return