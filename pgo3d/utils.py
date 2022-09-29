import numpy as np
from typing import List, Tuple, Union
from collections import namedtuple

# Define RelativePoseMeasurement container
RelativePoseMeasurement = namedtuple('RelativePoseMeasurement',
                                     ['i', 'j', 't', 'R', 'kappa', 'rot_info', 'tau', 'tran_info'])

def read_g2o_file(filename: str) -> Tuple[List[RelativePoseMeasurement], int]:
    """
    Parses the file with path `filename`, interpreting it as a g2o file and
    returning the set of measurements it contains and the number of poses
    Args:
        filename (str): Path to the g2o file.
    Returns:
        Tuple[List[RelativePoseMeasurement], int]: A tuple containing the list
        of measurements and the number of poses in the file.
    """
    measurements = []
    with open(filename, 'r') as infile:
        num_poses = 0
        for line in infile:
            parsed_line = line.split(' ', )

            # Clean up output
            while '\n' in parsed_line:
                parsed_line.remove('\n')
            while '' in parsed_line:
                parsed_line.remove('')

            if len(parsed_line) == 0:
                continue

            token = parsed_line[0]
            if (token == "EDGE_SE3:QUAT"):
                # This is a 3D pose measurements
                """The g2o format specifies a 3D relative pose measurement in the
                following form:
                EDGE_SE3:QUAT id1 id2 dx dy dz dqx dqy dqz dqw
                I11 I12 I13 I14 I15 I16
                    I22 I23 I24 I25 I26
                        I33 I34 I35 I36
                            I44 I45 I46
                                I55 I56
                                    I66
                """
                # Extract formatted output
                i, j, dx, dy, dz, dqx, dqy, dqz, dqw, I11, I12, I13, I14, I15, I16, I22, I23, I24, I25, I26, I33, I34, I35, I36, I44, I45, I46, I55, I56, I66 = map(float, parsed_line[1:])

                # Fill in elements of this measurement

                # Clean up pose ids
                i = int(i)
                j = int(j)

                # Raw measurements
                t = np.array([dx, dy, dz])

                # Reconstruct quaternion for relative measurement
                q = np.array([dqw, dqx, dqy, dqz])
                q = q / np.linalg.norm(q)

                R = quat2rot(q)

                meas_info = np.array([[I11, I12, I13, I14, I15, I16],
                                       [I12, I22, I23, I24, I25, I26],
                                      [I13, I23, I33, I34, I35, I36],
                                      [I14, I24, I34, I44, I45, I46],
                                      [I15, I25, I35, I45, I55, I56],
                                      [I16, I26, I36, I46, I56, I66]])

                tau = 3.0 / np.trace(np.linalg.inv(meas_info[0:2, 0:2]))
                kappa = 3.0 / (2.0 * np.trace(np.linalg.inv(meas_info[3:5, 3:5])))

                measurement = RelativePoseMeasurement(i=i,
                                                      j=j,
                                                      t=t,
                                                      R=R,
                                                      tau=tau,
                                                      tran_info=meas_info[0:2, 0:2],
                                                      kappa=kappa,
                                                      rot_info=meas_info[3:6, 3:6])
                max_pair = max(i, j)
                num_poses = max(num_poses, max_pair)

                measurements.append(measurement)
            if (token == "EDGE_SE2"):
                # This is a 2D pose measurement
                """ The g2o format specifies a 2D relative pose measurement in the following
                form:
                EDGE_SE2 id1 id2 dx dy dtheta I11 I12 I13 I22 I23 I33
                """

                # Extract formatted output
                i, j, dx, dy, dtheta, I11, I12, I13, I22, I23, I33 = map(
                    float, parsed_line[1:])

                # Fill in elements of this measurement

                # Clean up pose ids
                i = int(i)
                j = int(j)

                # Raw measurements
                t = np.array([dx, dy])
                R = rot2D_from_theta(dtheta)

                tran_info = np.array([[I11, I12], [I12, I22]])
                tau = 2.0 / np.trace(np.linalg.inv(tran_info))
                kappa = I33
                measurement = RelativePoseMeasurement(i=i,
                                                      j=j,
                                                      t=t,
                                                      R=R,
                                                      tau=tau,
                                                      tran_info=tran_info,
                                                      kappa=kappa,
                                                      rot_info=I33)
                max_pair = max(i, j)
                num_poses = max(num_poses, max_pair)

                measurements.append(measurement)

    # Account for zero-based indexing
    num_poses = num_poses + 1

    return measurements, num_poses
