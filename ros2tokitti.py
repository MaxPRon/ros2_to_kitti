import rosbag_api as bag
import numpy as np
import os
from tqdm import tqdm
import sensor_msgs_py.point_cloud2 as plc2
from pathlib import Path   

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import sensor_msgs_py.point_cloud2 as plc2

ROSBAG_PATH ='/home/maxronecker/Code/radar-dynamic-grids/data/vif_radar/01_oncoming_vehicle_15/01_oncoming_vehicle_15_0.db3'
LIDAR_TOPIC = '/sensing/lidar/top/points_downsampled'

RESULT_FOLDER = '/kitti_format'

def main():

    result_dir = os.path.split(ROSBAG_PATH)[0] + RESULT_FOLDER
    
    Path(result_dir).mkdir(parents=True, exist_ok=True)

    conn, c = bag.connect(ROSBAG_PATH)
    # get all topics names and types
    topic_names = bag.getAllTopicsNames(c, print_out=False)
    topic_types = bag.getAllMsgsTypes(c, print_out=False)
    # Create a map for quicker lookup
    type_map = {topic_names[i]: topic_types[i] for i in range(len(topic_types))}

    lidar_timestamps, lidar_msgs = bag.getAllMessagesInTopic(c, LIDAR_TOPIC, print_out=False)
    lidar_msg_type = get_message(type_map[LIDAR_TOPIC])

    all_deserialized_lidar_msgs = [deserialize_message(lidar_msgs[i], lidar_msg_type) for i in range(len(lidar_msgs))]
    points = plc2.read_points_list(all_deserialized_lidar_msgs[0])
    for counter, lidar_msg in tqdm(enumerate(all_deserialized_lidar_msgs)):
        points = plc2.read_points_list(lidar_msg)
        x = np.array([point.x for point in points])
        y = np.array([point.y for point in points])
        z = np.array([point.z for point in points])
        intensity = np.ones(x.shape[0]) # Currently our recorded rosbag has an issue with the intensity
        save_array = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)

        save_array[::4] = x
        save_array[1::4] = y
        save_array[2::4] = z
        save_array[3::4] = intensity

        filename = str(counter) + '.bin'
        save_array.astype('float32').tofile(os.path.join(result_dir,filename))

if __name__ == '__main__':
    main()



