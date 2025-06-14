"""
Autonomous two-dimensional agents which navigate in unstructured environments.
"""
import warnings
from abc import ABC, abstractmethod
from typing import Optional
from enum import Enum, auto
import math
import copy
import random

from scipy.optimize import minimize, Bounds

from asyncio import get_running_loop

import numpy as np
from numpy import linalg as LA
import numpy.typing as npt

import matplotlib.pyplot as plt

from vartools.states import Pose
from vartools.dynamical_systems import LinearSystem

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles.ellipse_xd import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.containers.obstacle_container import ObstacleContainer
from dynamic_obstacle_avoidance.avoidance import obs_avoidance_interpolation_moving


from autonomous_furniture.agent_helper_functions import (
    compute_ctr_point_vel_from_obs_avoidance,
    agent_kinematics_from_ctr_point_vel,
    apply_velocity_constraints,
    apply_linear_and_angular_acceleration_constraints,
    evaluate_safety_repulsion,
    evaluate_safety_repulsion_old,
    get_gamma_product_crowd,
    get_weight_of_control_points,
    get_params_from_file,
)

class BaseAgent(ABC):

    def __init__(
        self,
        shape: Obstacle,
        axes_occupancy,
        obstacle_environment: ObstacleContainer,
        control_points: Optional[np.ndarray],
        goal_pose: Pose,
        parameter_file: str,
        priority_value: float = None,
        name: str = "no_name",
        static: bool = None,
        object_type: str = "other",
        mode: bool = None,
        static_attractor = None,
        sensing_range: float = None,
        gamma_stop: float = None,
        repulsion_strength_regulator: float = None,
        maximum_linear_velocity: float = None,  # m/s
        maximum_angular_velocity: float = None,  # rad/s
        maximum_linear_acceleration: float = None,  # m/s^2
        maximum_angular_acceleration: float = None,  # rad/s^2
    ) -> None:
    
        super().__init__()

        self._shape = shape
        self.axes_occupancy = axes_occupancy
        self.object_type = object_type
        self._shape.object_type = object_type
        self._obstacle_environment = obstacle_environment
        self._control_points = control_points
        self._goal_pose = goal_pose
        self._parking_pose = copy.deepcopy(goal_pose)
        
        # Adding the current shape of the agent to the list of
        # obstacle_env so as to be visible to other agents
        self._obstacle_environment.append(self._shape)

        self = get_params_from_file(
            agent = self,
            parameter_file = parameter_file,
            maximum_linear_velocity = maximum_linear_velocity,
            maximum_angular_velocity = maximum_angular_velocity,
            maximum_linear_acceleration = maximum_linear_acceleration,
            maximum_angular_acceleration = maximum_angular_acceleration,
            repulsion_strength_regulator = repulsion_strength_regulator,
            sensing_range = sensing_range,
            gamma_stop = gamma_stop,
            mode = mode,
            static_attractor = static_attractor,
            static = static,
            name = name,
            priority_value = priority_value,
        )

        #self.maximum_linear_velocity = random.uniform(0.5, 1.0)
        #print(self.maximum_linear_velocity)

        self.converged: bool = False

        self.ctr_pt_number = self._control_points.shape[0]
        self.ctr_pt_dim = self._control_points.shape[1]

        self.linear_velocity = np.array([0.0, 0.0])
        self.angular_velocity = 0
        self.attractor_linear_velocity = np.array([0.0, 0.0])
        self.attractor_angular_velocity = 0
        self.unit_vel_avoid = None

        self.gamma_values = np.zeros(self.ctr_pt_number)  # Store the min Gamma of each control point
        self.obs_idx = [None] * self.ctr_pt_number  # Idx of the obstacle in the environment where the Gamma is calculated from
        self.gamma_values_check_collision = np.zeros(self.ctr_pt_number)
        self.state = "regrouping"

        # for safety module
        self.receive_repulsion = False
        self.repulsion_range = self.sensing_range
        self._shape.repulsion_range = self.repulsion_range

        # metrics
        self.direct_distance = LA.norm(self._parking_pose.position - self.position)
        self.total_distance = 0
        self.time_conv = 0
        self.time_conv_direct = 0
        self.list_gamma_min = []
        self.list_prox_min = []

        # low pass filter for smoothening traj
        self.filter_coeff = 0.75


    @property
    def pose(self):
        """Returns numpy-array position."""
        return self._shape.pose

    @property
    def position(self):
        """Returns numpy-array position."""
        return self._shape.pose.position

    @property
    def orientation(self) -> float:
        """Returns a (float) orientation (since uniquely 2d)"""
        if self._shape.pose.orientation is None:
            return 0
        else:
            return self._shape.pose.orientation

    @property
    def dimension(self):
        return self._shape.pose.dimension

    @property
    def linear_velocity(self):
        return self._shape.twist.linear

    @linear_velocity.setter
    def linear_velocity(self, value):
        self._shape.twist.linear = value

    @property
    def angular_velocity(self):
        return self._shape.twist.angular

    @angular_velocity.setter
    def angular_velocity(self, value):
        self._shape.twist.angular = value

    @property
    def priority(self):
        return self._shape.reactivity

    @priority.setter
    def priority(self, value):
        self._shape.reactivity = value

    @property
    def sensing_range(self):
        return self._sensing_range

    @sensing_range.setter
    def sensing_range(self, value):
        self._sensing_range = value

    @property
    def name(self):
        return self._shape.name

    @name.setter
    def name(self, name):
        self._shape.name = name

    def do_velocity_step(self, dt):
        return self._shape.do_velocity_step(dt)

    def get_global_control_points(self):
        return np.array(
            [
                self._shape.pose.transform_position_from_relative(ctp)
                for ctp in self._control_points
            ]
        ).T

    def get_goal_control_points(self):
        """Get goal-control-points in global frame."""
        return np.array(
            [
                self._goal_pose.transform_position_from_relative(ctp)
                for ctp in self._control_points
            ]
        ).T

    def get_veloctity_in_global_frame(self, velocity: npt.ArrayLike) -> np.ndarray:
        """Returns the transform of the velocity from relative to global frame."""
        return self._shape.pose.transform_direction_from_relative(np.array(velocity))

    def get_velocity_in_local_frame(self, velocity):
        """Returns the transform of the velocity from global to relative frame."""
        return self._shape.pose.transform_direction_to_relative(velocity)

    def get_obstacles_without_me(self):
        return [obs for obs in self._obstacle_environment if not obs == self._shape]


class AssFurniture(BaseAgent):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._dynamics = LinearSystem(
            attractor_position=self._goal_pose.position,
            maximum_velocity=self.maximum_linear_velocity,
        )

        # Metrics
        self.time_conv_direct = self.direct_distance / self._dynamics.maximum_velocity

    @property
    def margin_absolut(self):
        return self._shape._margin_absolut

    def apply_kinematic_constraints(self):
        linear_velocity = np.copy(self.linear_velocity)
        angular_velocity = self.angular_velocity

        linear_velocity, angular_velocity = apply_velocity_constraints(
            linear_velocity,
            angular_velocity,
            maximum_linear_velocity=self.maximum_linear_velocity,
            maximum_angular_velocity=self.maximum_angular_velocity,
        )

        linear_velocity, angular_velocity = apply_linear_and_angular_acceleration_constraints(
            self.linear_velocity_old,
            self.angular_velocity_old,
            linear_velocity,
            angular_velocity,
            maximum_linear_acceleration=self.maximum_linear_acceleration,
            maximum_angular_acceleration=self.maximum_angular_acceleration,
            time_step=self.time_step,
        )
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
    
    def check_status(self, rtol_pos=0.03, rtol_ang = 0.03):

        margin = self._shape._margin_absolut

        if self.static:
            self.converged = True
            self.repulsion_range = 1.05
            self._shape.repulsion_range = self.repulsion_range
            self.repulsion_strength_regulator = margin / (self.sensing_range - 1)
            self._shape.repulsion_strength_regulator = self.repulsion_strength_regulator
            return
        
        diff_position = LA.norm(self._goal_pose.position - self.position)
        diff_orientation = abs(self._goal_pose.orientation - self.orientation)

        if diff_position > self.sensing_range - 1:
            self.repulsion_strength_regulator = 1
            self.repulsion_range = self.sensing_range
        elif diff_position > margin:
            self.repulsion_strength_regulator = diff_position / (self.sensing_range - 1)
            self.repulsion_range = self.sensing_range
        else:
            self.repulsion_strength_regulator = margin / (self.sensing_range - 1)
            self.repulsion_range = 1.01
        self._shape.repulsion_strength_regulator = self.repulsion_strength_regulator
        self._shape.repulsion_range = self.repulsion_range

        num_cycle = math.floor(diff_orientation/(2*math.pi))
        angle_regulated = diff_orientation - 2*math.pi*num_cycle

        geometry_coverged = (diff_position < rtol_pos) and (angle_regulated < rtol_ang or angle_regulated > math.pi - rtol_ang)
        state_converged = (self.state != "avoiding") and (self.receive_repulsion == False)

        if geometry_coverged and state_converged:
            self.converged = True
        else:
            self.converged = False

        return

    def get_gamma_values(self, environment_without_me, global_control_points):

        for ii in range(self.ctr_pt_number):
            self.obs_idx[ii], self.gamma_values[ii] = get_gamma_product_crowd( 
                position = global_control_points[:, ii], 
                env = environment_without_me, 
                use_margin = True,
            )

            _, self.gamma_values_check_collision[ii] = get_gamma_product_crowd( 
                position = global_control_points[:, ii], 
                env = environment_without_me, 
                use_margin = False,
            )

    def check_collision(self):
        if self.static:
            self.angular_velocity = 0.
            self.linear_velocity = np.array([0.0, 0.0])
            return False
        if any(x <= self.gamma_stop for x in self.gamma_values_check_collision):
            print("EMERGENCY STOP")
            self.angular_velocity = 0.
            self.linear_velocity = np.array([0.0, 0.0])
            return True

        return False

    def get_attractor_dynamic(self, user_id_list, environment_without_me):

        gamma_range_user = 0.5*(self.sensing_range-1)+1
        gamma_range_surrounding = 0.25*(self.sensing_range-1)+1
        attractor_vel_list = []

        for user_id in user_id_list:
            
            obs_user = self._obstacle_environment[user_id]
            gamma_user_agent = obs_user.get_gamma(
                self._goal_pose.position,
                in_global_frame=True,
            )
            if gamma_user_agent > gamma_range_user:
                self.state = "regrouping"
                attractor_vel_list.append(np.zeros(self.ctr_pt_dim))
                continue

            vel_user = obs_user.linear_velocity
            vel_user_norm = LA.norm(vel_user)
            if vel_user_norm < 0.2:
                self.state = "regrouping"
                attractor_vel_list.append(np.zeros(self.ctr_pt_dim))
                continue
            unit_vel_user = vel_user / vel_user_norm

            dir_user_goal = self._goal_pose.position - obs_user.position
            unit_dir_user_goal = dir_user_goal/LA.norm(dir_user_goal)
            cos_dir_user_goal = np.dot(unit_vel_user, unit_dir_user_goal)
            if cos_dir_user_goal < -0.15:
                self.state = "regrouping"
                attractor_vel_list.append(np.zeros(self.ctr_pt_dim))
                continue

            theta_dir_user_goal = math.acos(cos_dir_user_goal)
            projected_collision_len = 0.5*max(self.axes_occupancy) + obs_user.margin_absolut
            projected_gap = LA.norm(dir_user_goal)*math.sin(theta_dir_user_goal) - projected_collision_len

            if projected_gap >= 0:
                attractor_vel_list.append(np.zeros(self.ctr_pt_dim))
                continue
            else:
                self.state = "avoiding"
                repulsion_weight = 10*np.dot(unit_dir_user_goal, vel_user)/gamma_user_agent

                if LA.norm(self.attractor_linear_velocity) != 0:
                    attractor_vel = repulsion_weight*self.attractor_linear_velocity/LA.norm(self.attractor_linear_velocity)
                else:
                    normal_user,_ = obs_user.get_normal_direction_with_distance(
                        self._goal_pose.position,
                        in_global_frame=True,
                    )

                    unit_vel_user_3D = np.zeros(3)
                    unit_vel_user_3D[0:2] = unit_vel_user
                    normal_user_3D = np.zeros(3)
                    normal_user_3D[0:2] = normal_user
                    vec_avoid_3D = np.cross(np.cross(unit_vel_user_3D, normal_user_3D), unit_vel_user_3D)
                    vec_avoid = vec_avoid_3D[0:2]
                    if LA.norm(vec_avoid) == 0:
                        unit_vec_avoid = np.zeros(2)
                    else:
                        unit_vec_avoid = vec_avoid/LA.norm(vec_avoid)

                    gamma_list = []
                    normal_list = []

                    gamma_min = gamma_range_surrounding

                    for obs in environment_without_me:
                        gamma_obs = obs.get_gamma(self._goal_pose.position, in_global_frame=True)
                        if gamma_obs >= gamma_range_surrounding:
                            continue
                        normal_obs,_ = obs.get_normal_direction_with_distance(
                            self._goal_pose.position,
                            in_global_frame=True,
                        )
                        gamma_list.append(gamma_obs)
                        normal_list.append(normal_obs)
                        if gamma_min > gamma_obs:
                            gamma_min = gamma_obs

                    weight = np.zeros(len(gamma_list))
                    weight = (gamma_range_surrounding - self.gamma_stop) / (np.array(gamma_list) - self.gamma_stop) - 1

                    if np.sum(weight) == 0:
                        unit_vec_side = np.zeros(2)
                    else:
                        vec_side = np.average(np.array(normal_list), axis=0, weights=weight)
                        unit_vec_side = vec_side/LA.norm(vec_side)
                
                    attractor_vel = repulsion_weight*unit_vec_avoid+unit_vec_side

                attractor_vel_list.append(attractor_vel)
        
        if self.static_attractor == 1:
            return np.zeros(self.ctr_pt_dim)
        else:
            return attractor_vel_list

    def update_attractor_velocity(self, user_id_list):

        if self.state == "following":
            return

        environment_without_me = self.get_obstacles_without_me()
        attractor_vel_list = self.get_attractor_dynamic(user_id_list, environment_without_me)
        init_attractor_vel = sum(attractor_vel_list)/len(attractor_vel_list)
        self.attractor_linear_velocity = init_attractor_vel
        self.attractor_angular_velocity = 0
 
    def update_attractor(self, time_step):
        if self.state == "following":
            new_goal_ori = math.pi/2
        elif self.state == "regrouping":
            new_goal_pos = self._parking_pose.position
            new_goal_ori = self._parking_pose.orientation
        else:
            new_goal_pos = self.attractor_linear_velocity * time_step + self._goal_pose.position
            new_goal_ori = -(1 * self.attractor_angular_velocity * time_step) + self._goal_pose.orientation

        self._goal_pose.position = new_goal_pos
        self._goal_pose.orientation = new_goal_ori
        self._dynamics.attractor_position = new_goal_pos

    def update_velocity(
        self,
        time_step: float = 0.1,
    ) -> None:

        global_control_points = self.get_global_control_points()
        goal_control_points = self.get_goal_control_points()
        environment_without_me = self.get_obstacles_without_me()
        self.get_gamma_values(environment_without_me, np.copy(global_control_points))

        self.time_step = time_step
        self.linear_velocity_old = self.linear_velocity
        if self.angular_velocity == None:
            self.angular_velocity = 0.0
        self.angular_velocity_old = self.angular_velocity

        if bool(environment_without_me):  # if there are other objects to take care of
            weights = get_weight_of_control_points(
                self.gamma_values,
                global_control_points,
                environment_without_me,
                gamma_stop=self.gamma_stop,
                cutoff_gamma=self.sensing_range
            )
        else:
            weights = np.ones(self.ctr_pt_number) / self.ctr_pt_number
            (
                linear_velocity,
                angular_velocity,
            ) = agent_kinematics_from_ctr_point_vel(
                velocities,
                weights,
                global_control_points=np.copy(global_control_points),
                ctrpt_number=self.ctr_pt_number,
                global_reference_position=self.position,
            )
            self.linear_velocity = linear_velocity
            self.angular_velocity = angular_velocity
            return

        velocities = compute_ctr_point_vel_from_obs_avoidance(
            number_ctrpt=self.ctr_pt_number,
            goal_pos_ctr_pts=goal_control_points,
            actual_pos_ctr_pts=np.copy(global_control_points),
            environment_without_me=environment_without_me,
            priority=self.priority,
            sensing_range=self.sensing_range,
            mode = self.mode
        )
            
        if (self.mode == 1) or (self.mode == 3):
            velocities, self.receive_repulsion = evaluate_safety_repulsion(
                gamma_values = self.gamma_values,
                number_ctrpt=self.ctr_pt_number,
                environment_without_me=environment_without_me,
                global_control_points=np.copy(global_control_points),
                obs_idx=self.obs_idx,
                velocities=velocities,
                sensing_range=self.sensing_range,
                converged=self.converged
            )
        elif self.mode == 2:
            velocities, self.receive_repulsion = evaluate_safety_repulsion_old(
                number_ctrpt=self.ctr_pt_number,
                environment_without_me=environment_without_me,
                global_control_points=np.copy(global_control_points),
                obs_idx=self.obs_idx,
                velocities=velocities,
                sensing_range=self.sensing_range,
                converged=self.converged,
                distance_to_goal=LA.norm(self.position - self._goal_pose.position)
            )

        (
            self.linear_velocity,
            self.angular_velocity,
        ) = agent_kinematics_from_ctr_point_vel(
            velocities,
            weights,
            global_control_points=np.copy(global_control_points),
            ctrpt_number=self.ctr_pt_number,
            global_reference_position=self.position,
        )

        self.apply_kinematic_constraints()
        self.linear_velocity = self.filter_coeff*self.linear_velocity + (1 - self.filter_coeff)*self.linear_velocity_old
        self.angular_velocity = self.filter_coeff*self.angular_velocity + (1 - self.filter_coeff)*self.angular_velocity_old

        self.check_status()
        if self.converged:
            self.linear_velocity = np.array([0.0, 0.0])
            self.angular_velocity = 0.0

    def compute_metrics(self, dt):
        if not self.converged:
            self.total_distance += LA.norm(self.linear_velocity) * dt
            self.time_conv += dt
        
        self.list_gamma_min.append(min(self.gamma_values))

        distance = []
        for obs in self.get_obstacles_without_me():
            distance.append(LA.norm(obs.position - self.position))
        self.list_prox_min.append(min(distance))

class Person(BaseAgent):
    def __init__(
        self,
        obstacle_environment,
        priority_value: float = 1,
        center_position: Optional[np.ndarray] = None,
        velocity = np.array([-0.3, 0]),
        radius: float = 0.25,
        margin: float = 1,
        **kwargs,
    ) -> None:
        _shape = Ellipse(
            center_position=np.array(center_position),
            margin_absolut=margin,
            orientation=0,
            tail_effect=False,
            repulsion_coeff=1,
            axes_length=np.array([2*radius, 2*radius]),
            linear_velocity = velocity
        )

        self.user_id = len(obstacle_environment)
        self.velocity = velocity
        self.state = "regrouping"

        super().__init__(
            shape=_shape,
            axes_occupancy=np.array([2*radius, 2*radius]),
            priority_value=priority_value,
            control_points=np.array([[0, 0]]),
            object_type="user",
            obstacle_environment = obstacle_environment,
            **kwargs,
        ) 

    @property
    def margin_absolut(self):
        return self._shape.margin_absolut

    def update_velocity(self, time_step, **kwargs):
        environment_without_me = self.get_obstacles_without_me()
        global_control_points = self.get_global_control_points()
        self.get_gamma_values(environment_without_me, np.copy(global_control_points))

        self.check_status()
        if self.converged:
            self.linear_velocity = np.array([0.0, 0.0])
        else:
            self.linear_velocity = self.velocity

    def compute_metrics(self, delta_t):
        pass

    def get_gamma_values(self, environment_without_me, global_control_points):
        for ii in range(self.ctr_pt_number):
            
            self.obs_idx[ii], self.gamma_values[ii] = get_gamma_product_crowd(  # TODO: Done elsewhere, for efficiency maybe will need to be delete
                position = global_control_points[:, ii], 
                env = environment_without_me,
                use_margin = True
            )

            _, self.gamma_values_check_collision[ii] = get_gamma_product_crowd(  # TODO: Done elsewhere, for efficiency maybe will need to be delete
                position = global_control_points[:, ii], 
                env = environment_without_me, 
                use_margin = False,
            )

    def check_collision(self):
        if any(x <= self.gamma_stop for x in self.gamma_values_check_collision):
            print("EMERGENCY STOP")
            self.angular_velocity = 0.
            self.linear_velocity = np.array([0.0, 0.0])
            return True

        return False
    
    def check_status(self, rtol_pos=0.05, rtol_ang = 0.1):
        
        diff_position = LA.norm(self._goal_pose.position - self.position)

        if diff_position > self.sensing_range - 1:
            self.repulsion_strength_regulator = 1
            self.repulsion_range = self.sensing_range
        elif diff_position > rtol_pos:
            self.repulsion_strength_regulator = diff_position / (self.sensing_range - 1)
            self.repulsion_range = self.sensing_range
        else:
            self.repulsion_strength_regulator = rtol_pos / (self.sensing_range - 1)
            self.repulsion_range = 1.01
        self._shape.repulsion_strength_regulator = self.repulsion_strength_regulator
        self._shape.repulsion_range = self.repulsion_range

        geometry_coverged = (diff_position < rtol_pos)
        state_converged = (self.state != "avoiding") and (self.receive_repulsion == False)

        if geometry_coverged and state_converged:
            self.converged = True
        else:
            self.converged = False

        return
