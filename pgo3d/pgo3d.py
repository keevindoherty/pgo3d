import sys
import gtsam
import numpy as np
import open3d as o3d
from evo.tools import file_interface
from evo.core.trajectory import PoseTrajectory3D

def g2o_to_traj(values, is3D):
    """Convert a set of gtsam.Values of type gtsam.pose{D} (D = 2 or 3) to a
    trajectory that can parsed by evo.

    `values`: a set of gtsam.Values of type gtsam.Pose{D} where D = 2 or 3
    `is3D`: true or false (0/1). Set to true of `values` contains gtsam.Pose3

    returns PosePath3D object for processing with evo
    """
    traj = []  # a List[np.ndarray] where each item is a 4x4 SE(3) matrix
    for k in values.keys():
        if is3D:
            pose = values.atPose3(k).matrix()
            traj.append(pose)
        else:
            pose2 = values.atPose2(k).matrix()
            traj.append(se2_to_se3(pose2))

    return PoseTrajectory3D(poses_se3 = traj,
                            timestamps = np.array(range(len(values.keys()))).astype(np.float64))

def g2o_to_o3d(graph, traj, odom_color=[0,0,1], lc_color=[1,0,0], remove_outlier_lc=True):
    """
    An o3d LineSet takes as argument `points` and `lines`
    `points`: a list of 3D points (as Vector3dVector)
    `lines`: a list of tuples containing indices of connected points
    (as Vector2iVector)
    e.g.:
    points = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1],
              [0, 1, 1], [1, 1, 1]]
    lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
             [0, 4], [1, 5], [2, 6], [3, 7]]
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([line_set])
    """
    points = []
    lines = []
    colors = []
    # all_keys = [key for key in values.keys()]
    # for i in range(len(all_keys)):
        # pt = values.atPose3(all_keys[i]).translation()
        # points.append([pt[0], pt[1], pt[2]])
    points = traj.positions_xyz

    for k in range(graph.size()):
        factor = graph.at(k)
        keys = factor.keys()
        idx1 = keys[0] #all_keys.index(keys[0])
        idx2 = keys[1] #all_keys.index(keys[1])
        if abs(idx2 - idx1) > 1:
            if remove_outlier_lc:
                delta = np.array(points[idx1]) - np.array(points[idx2])
                if np.linalg.norm(delta) < 50:
                    lines.append([idx1, idx2])
                    colors.append(lc_color)
        else:
            lines.append([idx1, idx2])
            colors.append(odom_color)
        # t1 = values.atPose3(keys[0]).translation()
        # t2 = values.atPose3(keys[1]).translation()
        # points = np.array([[t1[0], t1[1], t1[2]],
        #                    [t2[0], t2[1], t2[2]]])


    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} [.g2o file]")
        sys.exit()

    graph, values = gtsam.readG2o(sys.argv[1], True)

    traj_est = g2o_to_traj(values, True)

    if len(sys.argv) > 2:
        print("Got trajectory file.")
        traj_est = file_interface.read_kitti_poses_file(sys.argv[2])

    line_set = g2o_to_o3d(graph, traj_est)
    o3d.visualization.draw_geometries([line_set])
