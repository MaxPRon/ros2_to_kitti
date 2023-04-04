import numpy as np

bin_path = '/home/maxronecker/Code/ros2_to_kitti/kitti/kitti_000008.bin'


test = np.fromfile(bin_path, dtype=np.float32)