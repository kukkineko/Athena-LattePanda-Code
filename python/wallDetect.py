import numpy as np
from sklearn.cluster import KMeans

class WallDetect:
    def __init__(self, num_walls=2):
        self.num_walls = num_walls

    def detect_walls_in_pointcloud(self, point_cloud):
        kmeans = KMeans(n_clusters=self.num_walls)
        kmeans.fit(point_cloud)

        x_walls = [[] for _ in range(self.num_walls)]
        y_walls = [[] for _ in range(self.num_walls)]

        for i in range(self.num_walls):
            cluster_points = point_cloud[kmeans.labels_ == i]
            if len(cluster_points) > 0:
                wall_center = np.mean(cluster_points, axis=0)
                wall_points = cluster_points - wall_center
                _, _, v = np.linalg.svd(wall_points.T)
                wall_direction = v[0]
                wall_direction /= np.linalg.norm(wall_direction)

                # Find two points to represent the wall line
                p1 = wall_center - wall_direction[:, np.newaxis] * 1000
                p2 = wall_center + wall_direction[:, np.newaxis] * 1000

                x_walls[i] = [p1[0, 0], p2[0, 0]]
                y_walls[i] = [p1[1, 0], p2[1, 0]]

        print(x_walls, y_walls)
        return x_walls, y_walls
