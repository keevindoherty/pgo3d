import sys
import gtsam
import pgo3d.pgo3d as pgo3d
import open3d as o3d

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} [.g2o file]")
        sys.exit()

    graph, values = gtsam.readG2o(sys.argv[1], True)

    traj_est = pgo3d.g2o_to_traj(values, True)

    line_set = pgo3d.g2o_to_o3d(graph, traj_est)
    o3d.visualization.webrtc_server.enable_webrtc()
    o3d.visualization.draw([line_set])
