import numpy as np
from transforms3d.quaternions import quat2mat

input_file = "Log/keyframe_pose_optimized.txt"
output_file = "PCD/keyframe_pose_removert.txt"


def pose7d_to_matrix(x, y, z, qw, qx, qy, qz):
    quaternion = np.array([qw, qx, qy, qz])
    rotation_matrix = quat2mat(quaternion)
    translation_vector = np.array([x, y, z])

    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation_vector
    return transform_matrix


with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
    for line in infile:
        if line.startswith('#'):
            # outfile.write(line)
            pass
        else:
            parts = line.strip().split()
            timestamp = float(parts[0])
            tx, ty, tz, qx, qy, qz, qw = map(float, parts[1:])
            pose_mat = pose7d_to_matrix(tx, ty, tz, qw, qx, qy, qz)
            # print(pose_mat)

            # 在这里你可以执行任何你想要的操作，比如转换数据、修改数据等
            outfile.write(f"{pose_mat[0,0]:.6f} {pose_mat[0,1]:.6f} {pose_mat[0,2]:.6f} {pose_mat[0,3]:.6f} "
                          f"{pose_mat[1,0]:.6f} {pose_mat[1,1]:.6f} {pose_mat[1,2]:.6f} {pose_mat[1,3]:.6f} "
                          f"{pose_mat[2,0]:.6f} {pose_mat[2,1]:.6f} {pose_mat[2,2]:.6f} {pose_mat[2,3]:.6f}\n")

print(f"位姿数据已从 '{input_file}' 读取并写入 '{output_file}'")
