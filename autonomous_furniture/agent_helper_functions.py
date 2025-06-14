import numpy as np
from numpy import linalg as LA
from dynamic_obstacle_avoidance.avoidance import obs_avoidance_interpolation_moving
import yaml
from dynamic_obstacle_avoidance.visualization import plot_obstacles
import matplotlib.pyplot as plt


def apply_linear_and_angular_acceleration_constraints(
    linear_velocity_old,
    angular_velocity_old,
    linear_velocity,
    angular_velocity,
    maximum_linear_acceleration,
    maximum_angular_acceleration,
    time_step,
):
    # This function checks whether the difference in new computed kinematics and old kinematics exceeds the acceleration limits and adapts the kinematics in case th elimits are exceeded
    linear_velocity_difference = linear_velocity - linear_velocity_old
    angular_velocity_difference = angular_velocity - angular_velocity_old
    linear_velocity_difference_allowed = maximum_linear_acceleration * time_step
    angular_velocity_difference_allowed = maximum_angular_acceleration * time_step

    if LA.norm(linear_velocity_difference) > linear_velocity_difference_allowed:
        vel_correction = (
            linear_velocity_difference
            / LA.norm(linear_velocity_difference)
            * linear_velocity_difference_allowed
        )
        linear_velocity = linear_velocity_old + vel_correction

    if LA.norm(angular_velocity_difference) > angular_velocity_difference_allowed:
        angular_velocity = (
            angular_velocity_old
            + angular_velocity_difference
            / LA.norm(angular_velocity_difference)
            * angular_velocity_difference_allowed
        )

    return linear_velocity, angular_velocity


def compute_ctr_point_vel_from_obs_avoidance(
    number_ctrpt,
    goal_pos_ctr_pts,
    actual_pos_ctr_pts,
    environment_without_me,
    priority,
    sensing_range,
    mode
):
    # This function calculates all the control point velocities using DSM
    velocities = np.zeros((2, number_ctrpt))
    for i in range(number_ctrpt):
        # define direction as initial velocities
        ctr_pt_i = np.array(
            [actual_pos_ctr_pts[0][i], actual_pos_ctr_pts[1][i]]
        )  # extract i-th actual control points position
        ctr_pt_i_goal = np.array(
            [goal_pos_ctr_pts[0][i], goal_pos_ctr_pts[1][i]]
        )  # extract i-th goal control points position
        initial_velocity = ctr_pt_i_goal - ctr_pt_i
        if mode == 3:
            velocities[:, i] = initial_velocity
        elif mode == 4:
            velocities[:, i] = obs_avoidance_interpolation_moving(
                position=ctr_pt_i,
                initial_velocity=initial_velocity,
                obs=environment_without_me,
                self_priority=priority,
                cut_off_gamma = 0.5*sensing_range,
                repulsive_obstacle = True
            )
        else:
            velocities[:, i] = obs_avoidance_interpolation_moving(
                position=ctr_pt_i,
                initial_velocity=initial_velocity,
                obs=environment_without_me,
                self_priority=priority,
                cut_off_gamma = 0.5*sensing_range
            )

    return velocities

def agent_kinematics_from_ctr_point_vel(
    velocities, weights, global_control_points, ctrpt_number, global_reference_position
):
    # CALCULATE FINAL LINEAR AND ANGULAT VELOCITY OF AGENT GIVEN THE LINEAR VELOCITY OF EACH CONTROL POINT ###
    cotrol_points_relative_global = []
    for i in range(ctrpt_number):
        cotrol_points_relative_global.append(
            global_control_points[:, i] - global_reference_position
        )

    N = 2 * ctrpt_number
    A = np.zeros((N, 3))
    b = np.zeros((N))
    w_diag = np.zeros((N))

    for i in range(ctrpt_number):
        A[2 * i : 2 * i + 2, 0:2] = np.eye(2)
        A[2 * i : 2 * i + 2, 2] = np.array(
            [-cotrol_points_relative_global[i][1], cotrol_points_relative_global[i][0]]
        )

        b[2 * i : 2 * i + 2] = velocities[:, i]
        w_diag[2 * i : 2 * i + 2] = np.array([weights[i], weights[i]])
    W = np.diag(w_diag)
    Aw = np.dot(W, A)
    bw = np.dot(b, W)
    x = np.linalg.lstsq(Aw, bw, rcond=None)

    linear_velocity = x[0][0:2]
    angular_velocity = x[0][2]

    return linear_velocity, angular_velocity


def evaluate_safety_repulsion(
    gamma_values,
    number_ctrpt,
    environment_without_me,
    global_control_points: np.ndarray,
    obs_idx,
    velocities,
    sensing_range,
    converged
):
    # This function takes the control point velocities and checkes wether any of the control point velocities need to be modulated using the safety module and modulates those
    
    receive_repulsion = False
    for i in range(number_ctrpt):
        obs = environment_without_me[obs_idx[i]]
        gamma = gamma_values[i]
        if gamma <= 1:
            gamma = 1.001
        
        if gamma > obs.repulsion_range:
            continue
        if obs.object_type == "user":
            if converged:
                continue

        receive_repulsion = True
        normal, _ = obs.get_normal_direction_with_distance(
            global_control_points[:, i],
            in_global_frame=True,
        )

        b = 1 / ((sensing_range - 1)*(gamma - 1))
        velocities[:, i] += obs.repulsion_strength_regulator * b * normal

    return velocities, receive_repulsion

def evaluate_safety_repulsion_old(
    number_ctrpt,
    environment_without_me,
    global_control_points: np.ndarray,
    obs_idx,
    velocities,
    sensing_range,
    converged,
    distance_to_goal
):
    # This function takes the control point velocities and checkes wether any of the control point velocities need to be modulated using the safety module and modulates those
    
    gamma_min = 1
    gamma_max = sensing_range
    th_coverge = 0.05
    
    if distance_to_goal < sensing_range:
        curr_sensing_range = 1 + (distance_to_goal-th_coverge) * (gamma_max - gamma_min) / gamma_max
    else:
        curr_sensing_range = gamma_max

    receive_repulsion = False
    for i in range(number_ctrpt):
        obs = environment_without_me[obs_idx[i]]
        gamma = obs.get_gamma(global_control_points[:, i], in_global_frame=True)
        if gamma < 1:
            gamma = 1.001
       
        if gamma > curr_sensing_range:
            continue

        receive_repulsion = True
        normal, _ = obs.get_normal_direction_with_distance(
            global_control_points[:, i],
            in_global_frame=True,
        )
        b = 0.3 / ((curr_sensing_range - 1) * (gamma - 1))
        velocities[:, i] += 1 * b * normal

    return velocities, receive_repulsion

def apply_velocity_constraints(
    linear_velocity, angular_velocity, maximum_linear_velocity, maximum_angular_velocity
):
    # This function check wether the velocity constraints are resepcted and adapts the linear and angular velocity in case
    if (
        LA.norm(linear_velocity) > maximum_linear_velocity
    ):  # resize speed if it passes maximum speed
        linear_velocity *= maximum_linear_velocity / LA.norm(linear_velocity)

    if (
        LA.norm(angular_velocity) > maximum_angular_velocity
    ):  # resize speed if it passes maximum speed
        angular_velocity = (
            angular_velocity / LA.norm(angular_velocity) * maximum_angular_velocity
        )

    return linear_velocity, angular_velocity

def get_gamma_product_crowd(position, env, use_margin):
    # This fuction gives back the smallest gamma value and its index for one control point

    if not len(env):
        # Very large number
        return 0, 1e20

    gamma_list = np.zeros(len(env))
    for ii, obs in enumerate(env):
        if use_margin:
            gamma_list[ii] = obs.get_gamma(position, in_global_frame=True, margin_absolut=obs.margin_absolut)
            if gamma_list[ii] <= 1:
                gamma_list[ii] = 1.001
        else:
            gamma_list[ii] = obs.get_gamma(position, in_global_frame=True, margin_absolut=0)
            if gamma_list[ii] <= 1:
                return 0, 0

    gamma = np.min(gamma_list)
    index = int(np.argmin(gamma_list))

    return index, gamma

def get_weight_of_control_points(
    gamma_values, control_points, environment_without_me, gamma_stop, cutoff_gamma
):
    # This function calculates the weights of each control point regarding the smallest gamma value of each point
    num_ctp = control_points.shape[1]

    ctl_point_weight = np.zeros(num_ctp)
    ind_nonzero = gamma_values < cutoff_gamma
    if not any(ind_nonzero):
        ctl_point_weight = np.full(num_ctp, 1 / num_ctp)

    ctl_point_weight[ind_nonzero] = (cutoff_gamma - gamma_stop)/(gamma_values[ind_nonzero] - gamma_stop) - 1
    ctl_point_weight_sum = np.sum(ctl_point_weight)
    ctl_point_weight = ctl_point_weight / ctl_point_weight_sum

    return ctl_point_weight


def get_params_from_file(
    agent,
    parameter_file,
    mode,
    static_attractor,
    maximum_linear_velocity,
    maximum_angular_velocity,
    maximum_linear_acceleration,
    maximum_angular_acceleration,
    repulsion_strength_regulator,
    sensing_range,
    gamma_stop,
    static,
    name,
    priority_value,
):
    """This function checks wether any variable already is assigned and if not assigns the value from the parameter file"""

    with open(parameter_file, "r") as openfile:
        yaml_object = yaml.safe_load(openfile)

    if mode == None:
        agent.mode = yaml_object["mode"]
    else:
        agent.mode = mode
    
    if static_attractor == None:
        agent.static_attractor = yaml_object["static attractor"]
    else:
        agent.static_attractor = static_attractor

    if maximum_linear_velocity == None:
        agent.maximum_linear_velocity = yaml_object["maximum linear velocity"]
    else:
        agent.maximum_linear_velocity = maximum_linear_velocity

    if maximum_angular_velocity == None:
        agent.maximum_angular_velocity = yaml_object["maximum angular velocity"]
    else:
        agent.maximum_angular_velocity = maximum_angular_velocity

    if maximum_linear_acceleration == None:
        agent.maximum_linear_acceleration = yaml_object["maximum linear acceleration"]
    else:
        agent.maximum_linear_acceleration = maximum_linear_acceleration

    if maximum_angular_acceleration == None:
        agent.maximum_angular_acceleration = yaml_object["maximum angular acceleration"]
    else:
        agent.maximum_angular_acceleration = maximum_angular_acceleration

    agent.repulsion_strength_regulator = 1

    if (
        sensing_range == None
    ):  
        agent.sensing_range = yaml_object["sensing range"]
    else:
        agent.sensing_range = sensing_range

    agent.gamma_stop = 1  # agent should stop when a ctrpoint reaches a gamma value under this threshold

    if static == None:
        agent.static = yaml_object["static"]
    else:
        agent.static = static

    if name == "no_name":
        agent.name = yaml_object["name"]
    else:
        agent.name = name

    if priority_value == None:
        agent.priority = 1
    else:
        agent.priority = priority_value

    return agent
