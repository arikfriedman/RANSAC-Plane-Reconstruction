
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd
import matplotlib.pyplot as plt
import random
import math
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d


def distance_from_plane(a, b, c, d, x, y, z):
    return math.fabs(a*x + b*y + c*z + d)/math.sqrt(a*a + b*b + c*c)


class RansacPlane:
    def __init__(self, pcl, iterations, distance_threshold):
        self.pcl = pcl
        self.iterations = iterations
        self.distance_threshold = distance_threshold
        self.best_plane_params = [0, 0, 0, 0]  # a, b, c, d

    def run_ransac_algorithm(self):
        inliers, outliers = self.ransac_algorithm()
        print("plane equation: ")
        print(str(self.best_plane_params[0]) + "X + " + str(self.best_plane_params[1]) + "Y + " +
              str(self.best_plane_params[2]) + "Z + " + str(self.best_plane_params[3]) + " = 0")
        fig = plt.figure()
        axis = Axes3D(fig)
        axis.scatter(inliers.X, inliers.Y,  inliers.Z,  c="green")
        axis.scatter(outliers.X, outliers.Y, outliers.Z, c="red")
        plt.show()

    def ransac_algorithm(self):
        max_inliers = []
        while self.iterations > 0:
            self.iterations -= 1

            # adding 3 random pcl points to create a plane:
            random.seed()
            inliers = []
            for i in range(3):
                idx = random.randint(0, len(self.pcl.X)-1)
                inliers.append(idx)

            x1, y1, z1 = self.pcl.loc[inliers[0]]
            x2, y2, z2 = self.pcl.loc[inliers[1]]
            x3, y3, z3 = self.pcl.loc[inliers[2]]

            # Plane Equation: ax + by + cz +d = 0
            # calculating the plane constants:
            a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
            b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
            c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
            d = -(a*x1 + b*y1 + c*z1)

            # for the specific plane choice, iterating through all the other points and checking if they are inliers
            # or outliers:
            for point in self.pcl.iterrows():
                idx = point[0]  # first value in iterrows is the index
                x, y, z = point[1]  # second value in iterrows is the data
                # if the distance of the point from the plane is smaller than the threshold, count it as an inlier:
                if distance_from_plane(a, b, c, d, x, y, z) <= self.distance_threshold:
                    inliers.append(idx)
            # update the "best plane" and the inliers of that plane
            if len(inliers) > len(max_inliers):
                max_inliers.clear()
                max_inliers = inliers
                self.best_plane_params = [a, b, c, d]

        # differentiate between inliers and outliers for visulalization purposes:
        outlier_df = pd.DataFrame(columns={"X", "Y", "Z"})
        inlier_df = pd.DataFrame(columns={"X", "Y", "Z"})
        for point in self.pcl.iterrows():
            if point[0] in max_inliers:
                inlier_df = inlier_df.append({"X": point[1]["X"],
                                                      "Y": point[1]["Y"],
                                                      "Z": point[1]["Z"]}, ignore_index=True)
            else:
                outlier_df = outlier_df.append({"X": point[1]["X"],
                                                        "Y": point[1]["Y"],
                                                        "Z": point[1]["Z"]}, ignore_index=True)

        return inlier_df, outlier_df


if __name__ == "__main__":
    ITERATIONS = 20
    DISTANCE_THRESHOLD = 0.8
    # read point cloud:
    pcl_raw = o3d.io.read_point_cloud(r"pcl_sample.xyz")
    # sample point cloud data for faster computation:
    pcl_raw = pcl_raw.random_down_sample(sampling_ratio=0.01)
    pcl_df = pd.DataFrame(pcl_raw.points, columns={"X","Y","Z"})
    # run ransac plane recunstruction algorithm:
    RansacPlane(pcl_df, ITERATIONS, DISTANCE_THRESHOLD).run_ransac_algorithm()
