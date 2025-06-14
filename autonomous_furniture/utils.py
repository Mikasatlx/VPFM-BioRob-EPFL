import numpy as np
import math
from math import sin, cos, pi
from scipy.spatial.transform import Rotation 
from geometry_msgs.msg import Quaternion

np.set_printoptions(precision = 6, suppress=True)

def relative2global(relative_pos, obstacle):
    angle = obstacle.orientation
    obs_pos = obstacle.center_position
    global_pos = np.zeros_like(relative_pos)
    rot = np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])

    for i in range(relative_pos.shape[0]):
        rot_rel_pos = np.dot(rot, relative_pos[i, :])
        global_pos[i, :] = obs_pos + rot_rel_pos

    return global_pos

def global2relative(global_pos, obstacle):
    angle = -1 * obstacle.orientation
    obs_pos = obstacle.center_position
    relative_pos = np.zeros_like(global_pos)
    rot = np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])

    for i in range(global_pos.shape[0]):
        rel_pos_pre_rot = global_pos[i, :] - obs_pos
        relative_pos[i, :] = np.dot(rot, rel_pos_pre_rot)

    return relative_pos

def calculate_relative_position(num_agent, max_ax, min_ax):
    div = max_ax / (num_agent + 1)
    radius = np.sqrt(((min_ax / 2) ** 2) + (div**2))
    rel_agent_pos = np.zeros((num_agent, 2))

    for i in range(num_agent):
        rel_agent_pos[i, 0] = (div * (i + 1)) - (max_ax / 2)

    return rel_agent_pos, radius

##### Vicon Tracker UTILS FUNCTIONS #####                          
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

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Returns geometry_msgs-Quaternion based on roll-pitch-yaw input."""
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(
        pitch / 2
    ) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(
        pitch / 2
    ) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def euler_from_quaternion(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians