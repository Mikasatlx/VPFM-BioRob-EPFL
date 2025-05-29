import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from mocap4r2_msgs.msg import Markers
from assistive_furniture_interfaces.msg import Furniture
from assistive_furniture_interfaces.msg import FurnitureList


import yaml
import numpy as np
from scipy.spatial.transform import Rotation 

import matplotlib.pyplot as plt


np.set_printoptions(precision = 6, suppress=True)


##### GLOBAL VARIABLES #####
calibration_data_file = './src/furniture_tracker/config/calibration_data_file.yaml'
marker_types_G = ['regular', 'origin', 'robot']
calibration_counter_G = 50
alpha_EMA = 0.5

##### UTILS FUNCTIONS #####                          
def calculate_transformation(markers):
    if markers.ndim == 1: # 1 marker
        translation = markers.copy()
        R_wb = np.eye(3)

    elif markers.shape[0] == 2: # 2 markers: center = 1st, (2nd - 1st) = heading
        translation = markers[0,:].copy()

        yaw_vec = np.array([0,0,1])

        heading = markers[1,:] - markers[0,:]
        heading /= np.linalg.norm(heading)

        pitch_vec = np.cross(yaw_vec, heading)
        pitch_vec /= np.linalg.norm(pitch_vec)

        roll_vec = np.cross(pitch_vec,yaw_vec)
        roll_vec /= np.linalg.norm(roll_vec)

        R_wb = np.stack([roll_vec, pitch_vec, yaw_vec],axis=1)
        rotation = Rotation.from_matrix(R_wb)

    else: # n markers: center = mean, 1st-center = heading, z=normal plane 3 first markers
        translation = np.mean(markers, axis=0)

        x_vec = markers[0,:] - translation
        x_vec /= np.linalg.norm(x_vec)

        z_vec = np.cross(markers[1,:]-markers[0,:], markers[2,:]-markers[0,:])
        z_vec /= np.linalg.norm(z_vec)

        y_vec = np.cross(z_vec, x_vec)
        y_vec /= np.linalg.norm(y_vec)

        R_wb = np.stack([x_vec, y_vec, z_vec],axis=1)
        rotation = Rotation.from_matrix(R_wb)
    
    return rotation, translation

def create_homogenous_transformation_matrix(rotation=np.eye(3), translation=np.zeros(3)):
    T = np.eye((4))
    T[0:3,0:3] = rotation
    T[0:3,3] = translation
    return T
    
##### CLASS #####
class FurnitureDescription:
    def __init__(self, name):
        self.furniture_name = name

        self.position = np.zeros(3)
        self.orientation = np.zeros(4)
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        self.is_occluded = True
        self.last_update = None

        self.filtered_markers = Filter_EMA(alpha_EMA)

class Filter_EMA:
    def __init__(self, alpha):
        self.alpha = alpha
        self.data_old = None

    def __call__(self, data):
        if self.data_old is None:
            self.data_old = data
        else:
            self.data_old = self.alpha * data + (1 - self.alpha) * self.data_old
        return self.data_old

##### NODE #####
class FurnitureTracker(Node):
    def __init__(self):
        super().__init__('furniture_tracker')
        self.group = MutuallyExclusiveCallbackGroup()
        self.furniture_dict = {}

        self.declare_parameter('calibration', False)
        self.declare_parameter('tracking_system', 'vicon')
        (self.param_calibration, self.param_tracking_system) = self.get_parameters(['calibration', 'tracking_system'])

        if self.param_tracking_system.value == 'vicon':
            try:
                with open(calibration_data_file, 'r') as file:
                    self.calibration_data = yaml.safe_load(file)
                    if self.calibration_data is None:
                        self.calibration_data = {}  
            except FileNotFoundError:
                self.calibration_data = {}

            if self.param_calibration.value==True:
                _ = self.create_subscription(Markers, 'markers', self.mocap_calibration_callback, 1)
            else:
                _ = self.create_subscription(Markers, 'markers', self.mocap_regular_callback, 1, callback_group=self.group)

                self.freq_publisher = 10
                _ = self.create_timer(1/self.freq_publisher, self.timer_tracker_publisher_callback, callback_group=self.group)
                self.tracker_publisher = self.create_publisher(FurnitureList, 'tracker', 1)

        self.get_logger().info('Node running ...')

    def timer_tracker_publisher_callback(self):
        msg = FurnitureList()
        timestamp = self.get_clock().now().to_msg()
        for furniture_name in self.furniture_dict:
            furniture_msg = Furniture()
            furniture_msg.header.stamp = timestamp
            furniture_msg.header.frame_id = furniture_name
            self.numpy_2_msg(furniture_msg, furniture_name)
            msg.furniture.append(furniture_msg)

        self.tracker_publisher.publish(msg)

    def mocap_regular_callback(self, msg):
        marker_dict, timestamp = self.extract_info_mocap(msg, m_type = 'regular')

        # Calculate spatial transformation for all furniture and update values in dictionary
        for furniture_name in marker_dict:
            if furniture_name not in self.furniture_dict:
                self.furniture_dict[furniture_name] = FurnitureDescription(furniture_name)
                if furniture_name not in self.calibration_data:
                    self.calibration_data[furniture_name]={}
                for marker_type in marker_types_G[1:]:
                    if marker_type not in self.calibration_data[furniture_name]:
                        self.get_logger().warning(f'Missing calibration data of \'{furniture_name}_{marker_type}\', using identity matrices')
                        self.calibration_data[furniture_name][marker_type] = np.eye(4)

            markers = marker_dict[furniture_name]['regular']
            if markers.all():                  
                # Filtering
                markers_filtered = self.furniture_dict[furniture_name].filtered_markers(markers)

                # Pose
                r_reg, t_reg = calculate_transformation(markers_filtered)
                T_reg = create_homogenous_transformation_matrix(r_reg.as_matrix(), t_reg)
                T_furniture = T_reg @ self.calibration_data[furniture_name]['origin']

                # Velocities
                if  not self.furniture_dict[furniture_name].is_occluded:
                    dt = timestamp.sec - self.furniture_dict[furniture_name].last_update.sec + (timestamp.nanosec - self.furniture_dict[furniture_name].last_update.nanosec)*1e-9

                    T_furniture_old = create_homogenous_transformation_matrix(Rotation.from_quat(self.furniture_dict[furniture_name].orientation).as_matrix(), self.furniture_dict[furniture_name].position)
                    dT_furniture = np.linalg.inv(T_furniture_old) @ T_furniture

                    self.furniture_dict[furniture_name].linear_velocity = T_furniture_old[0:3,0:3] @ (dT_furniture[0:3,3] / dt) # global frame
                    self.furniture_dict[furniture_name].angular_velocity = np.linalg.inv(dT_furniture[0:3,0:3]) @ (Rotation.from_matrix(dT_furniture[0:3,0:3]).as_rotvec() / dt)               

                self.furniture_dict[furniture_name].position = T_furniture[0:3,3]
                self.furniture_dict[furniture_name].orientation = Rotation.from_matrix(T_furniture[0:3,0:3]).as_quat()

                # Other attributes
                self.furniture_dict[furniture_name].last_update = timestamp
                if self.furniture_dict[furniture_name].is_occluded:
                    self.furniture_dict[furniture_name].is_occluded = False
                    self.get_logger().info(f'{furniture_name} is tracked')

            else:
                # TODO: think about what to do when it occurs; depending on time?
                if self.furniture_dict[furniture_name].is_occluded == False:
                    self.furniture_dict[furniture_name].is_occluded = True
                    self.get_logger().warning(f'{furniture_name} is occluded')
                
    def mocap_calibration_callback(self, msg):
        marker_dict, _ = self.extract_info_mocap(msg, m_type='all')
        
        # stack markers for filtering
        for furniture_name, markers in marker_dict.items():
            if furniture_name not in self.furniture_dict:
                self.furniture_dict[furniture_name] = {}
            for marker_type, values in markers.items():
                if values.all():
                    values = np.expand_dims(values, axis=0)
                    if marker_type not in self.furniture_dict[furniture_name]:
                        self.furniture_dict[furniture_name][marker_type] = values
                    else:
                        self.furniture_dict[furniture_name][marker_type]= np.vstack((self.furniture_dict[furniture_name][marker_type],
                                                                                    values))

        # check if enough data for each marker type
        ready_for_saving = True
        for _, markers in self.furniture_dict.items():
            for val in markers.values():
                if len(val)<calibration_counter_G:
                    ready_for_saving = False   
   
        if ready_for_saving:
            with open(calibration_data_file, 'w') as file:
                for furniture_name in self.furniture_dict:
                    markers_reg = self.furniture_dict[furniture_name]['regular'].mean(axis=0)
                    r_reg, t_reg = calculate_transformation(markers_reg)
                    T_reg = create_homogenous_transformation_matrix(r_reg.as_matrix(), t_reg)
                    T_reg_inv = np.linalg.inv(T_reg)

                    for marker_type in [m for m in self.furniture_dict[furniture_name] if m!='regular']:
                        markers = self.furniture_dict[furniture_name][marker_type].mean(axis=0)

                        r_calib, t_calib = calculate_transformation(markers)
                        T_calib = create_homogenous_transformation_matrix(r_calib.as_matrix(), t_calib)

                        T_reg_calib = T_reg_inv @ T_calib

                        self.get_logger().warning(f'Saving calibration of \'{furniture_name}_{marker_type}\'\n {T_reg_calib}')
                        if furniture_name not in self.calibration_data:
                            self.calibration_data[furniture_name] = {}
                        self.calibration_data[furniture_name][marker_type]=(T_reg_calib.tolist())

                yaml.safe_dump(self.calibration_data, file)
            raise SystemExit  

    def extract_info_mocap(self, msg, f_name = ['all'], m_type = ['all']):
        marker_dict = {}
        timestamp = msg.header.stamp

        if 'all' in f_name:
            any_furniture = True
        else:
            any_furniture = False

        if 'all' in m_type:
            any_marker = True
        else:
            any_marker = False

        
        for marker in msg.markers:

            marker_name = marker.marker_name
            name_parts = marker_name.split('_')

            if len(name_parts) == 4:
                # Extract info
                furniture_name = name_parts[0]+'_'+name_parts[1]
                marker_type = name_parts[2]
                marker_id = int(name_parts[3]) - 1 # Vicon start labeling at 1, Python at 0
                marker_position = np.array([marker.translation.x,marker.translation.y, marker.translation.z])
                
                if (furniture_name in f_name or any_furniture) and (marker_type in m_type or any_marker):
                    if furniture_name not in marker_dict:   
                        marker_dict[furniture_name] = {}
                    if marker_type not in marker_dict[furniture_name]:
                        marker_dict[furniture_name][marker_type]={}
                    marker_dict[furniture_name][marker_type][marker_id]=marker_position
                else:
                    continue
            else:
                print(f'The naming convention of marker \'{marker_name}\' is not correct; should be \'FurnitureType_FurnitureID_MarkerType_MarkerID\'')

        for furniture_name in marker_dict:
            for marker_type in marker_dict[furniture_name]:
                markers = np.vstack([pos for _, pos in sorted(marker_dict[furniture_name][marker_type].items())])
                marker_dict[furniture_name][marker_type] = markers    
        return marker_dict, timestamp

    def numpy_2_msg(self, furniture_msg, furniture_name):
        furniture_msg.pose.position.x = self.furniture_dict[furniture_name].position[0]
        furniture_msg.pose.position.y = self.furniture_dict[furniture_name].position[1]
        furniture_msg.pose.position.z = self.furniture_dict[furniture_name].position[2]
        furniture_msg.pose.orientation.x = self.furniture_dict[furniture_name].orientation[0]
        furniture_msg.pose.orientation.y = self.furniture_dict[furniture_name].orientation[1]
        furniture_msg.pose.orientation.z = self.furniture_dict[furniture_name].orientation[2]
        furniture_msg.pose.orientation.w = self.furniture_dict[furniture_name].orientation[3]
        furniture_msg.twist.linear.x = self.furniture_dict[furniture_name].linear_velocity[0]
        furniture_msg.twist.linear.y = self.furniture_dict[furniture_name].linear_velocity[1]
        furniture_msg.twist.linear.z = self.furniture_dict[furniture_name].linear_velocity[2]
        furniture_msg.twist.angular.x = self.furniture_dict[furniture_name].angular_velocity[0]
        furniture_msg.twist.angular.y = self.furniture_dict[furniture_name].angular_velocity[1]
        furniture_msg.twist.angular.z = self.furniture_dict[furniture_name].angular_velocity[2]
        
    
##### MAIN #####
def main(args=None):
    rclpy.init(args=args)

    try:
        furniture_tracker = FurnitureTracker()
        try:
            rclpy.spin(furniture_tracker)
        finally:
            furniture_tracker.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
