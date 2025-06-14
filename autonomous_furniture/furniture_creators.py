"""
Furniture Factories
"""
# Author: Lukas Huber
# Github: hubernikus
# Created: 2023-02-28

from typing import Optional

import math
import yaml
import numpy as np

from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid
from dynamic_obstacle_avoidance.obstacles import Polygon
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.containers import ObstacleContainer

from autonomous_furniture.agent import BaseAgent
from autonomous_furniture.agent import AssFurniture

from vartools.states import Pose
import copy

def create_chair(
    parameter_file: str,
    obstacle_environment: ObstacleContainer,
    start_pose: Pose,
    goal_pose: Optional[Pose] = None,
    name: str = "",
    margin_absolut: float = 0.2,
    axes_length = np.array([0.7, 0.6])
):
    if not len(name):
        name = f"obstacle{len(obstacle_environment)}"

    if goal_pose is None:
        goal_pose = start_pose

    # Small difference of control points to allow changing the orientation
    # TODO: somehow the re-orientation does not work
    # control_points = np.array([[-0.25*axes_length[0], 0], [0.25*axes_length[0], 0]])
    # control_points = np.array([[-0.5*axes_length[0], -0.5*axes_length[1]],
    #                            [-0.5*axes_length[0], 0.5*axes_length[1]],
    #                            [0.5*axes_length[0], 0.5*axes_length[1]],
    #                            [0.5*axes_length[0], -0.5*axes_length[1]]])
    #control_points = np.array([[0.5*axes_length[0], 0.5*axes_length[1]], [-0.5*axes_length[0], 0.5*axes_length[1]],
    #                           [-0.5*axes_length[0], -0.5*axes_length[1]], [0.5*axes_length[0], -0.5*axes_length[1]]])

    ratio_ax_len = 0.5
    control_points = np.array([[-ratio_ax_len*axes_length[0], ratio_ax_len*axes_length[1]],
                                [0, ratio_ax_len*axes_length[1]],
                                [ratio_ax_len*axes_length[0], ratio_ax_len*axes_length[1]],
                                [ratio_ax_len*axes_length[0], 0],
                                [ratio_ax_len*axes_length[0], -ratio_ax_len*axes_length[1]],
                                [0, -ratio_ax_len*axes_length[1]],
                                [-ratio_ax_len*axes_length[0], -ratio_ax_len*axes_length[1]],
                                [-ratio_ax_len*axes_length[0], 0]])

    shape_ = Polygon(
        edge_points=control_points.T,
        center_position=start_pose.position,
        margin_absolut=margin_absolut,
        orientation=start_pose.orientation,
        tail_effect=False,
        repulsion_coeff=1
    )

    new_furniture = AssFurniture(
        parameter_file = parameter_file,
        shape=shape_,
        axes_occupancy=axes_length,
        obstacle_environment=obstacle_environment,
        control_points=control_points,
        goal_pose=goal_pose,
        priority_value=1.0,
        name=name,
        object_type="chair",
    )

    return new_furniture

def create_regular_polygon(
    parameter_file: str,
    obstacle_environment: ObstacleContainer,
    radius: float,
    num_vertex: int,
    start_pose: Pose,
    goal_pose: Optional[Pose] = None,
    name: str = "",
    margin_absolut: float = 0.2,
):
    if not len(name):
        name = f"obstacle{len(obstacle_environment)}"

    if goal_pose is None:
        goal_pose = start_pose

    edge_points = np.zeros((num_vertex,2))
    control_points = np.zeros((2*num_vertex,2))
    delta_angle = 2*np.pi/num_vertex

    for idx in range(num_vertex):
        vertex = np.array([radius*math.cos(idx*delta_angle), radius*math.sin(idx*delta_angle)])
        edge_points[idx] = vertex
        control_points[2*idx] = vertex
    
    for idx in range(num_vertex):
        mid_point = 0.5*(edge_points[idx] + edge_points[(idx+1)%num_vertex])
        control_points[2*idx+1] = mid_point
    
    if num_vertex <= 5:
        final_points = control_points
    else:
        final_points = edge_points

    shape_ = Polygon(
        edge_points=edge_points.T,
        center_position=start_pose.position,
        margin_absolut=margin_absolut,
        orientation=start_pose.orientation,
        tail_effect=False,
        repulsion_coeff=1
    )

    new_furniture = AssFurniture(
        parameter_file = parameter_file,
        shape=shape_,
        axes_occupancy=[2*radius,2*radius],
        obstacle_environment=obstacle_environment,
        control_points=final_points,
        goal_pose=goal_pose,
        priority_value=1.0,
        name=name,
        object_type="polygon",
    )

    return new_furniture

def create_polygon(
    parameter_file: str,
    obstacle_environment: ObstacleContainer,
    start_pose: Pose,
    vertices,
    goal_pose: Optional[Pose] = None,
    name: str = "",
    margin_absolut: float = 0.2,
):
    if not len(name):
        name = f"obstacle{len(obstacle_environment)}"

    if goal_pose is None:
        goal_pose = start_pose

    edge_points = np.array([[0, 0.3],
                            [0.05, 0.05],
                            [0.3, 0],
                            [0.05, -0.05],
                            [0, -0.3],
                            [-0.05, -0.05],
                            [-0.3, 0],
                            [-0.05, 0.05]])

    shape_ = Polygon(
        edge_points=vertices.T,
        center_position=start_pose.position,
        margin_absolut=margin_absolut,
        orientation=start_pose.orientation,
        tail_effect=False,
        repulsion_coeff=1
    )

    new_furniture = AssFurniture(
        parameter_file = parameter_file,
        shape=shape_,
        axes_occupancy=[0.6,0.6],
        obstacle_environment=obstacle_environment,
        control_points=vertices,
        goal_pose=goal_pose,
        priority_value=1.0,
        name=name,
        object_type="chair",
    )

    return new_furniture

def create_circle(
    parameter_file: str,
    obstacle_environment: ObstacleContainer,
    start_pose: Pose,
    goal_pose: Optional[Pose] = None,
    name: str = "",
    margin_absolut: float = 0.2,
    radius: float = 0.2,
):
    if not len(name):
        name = f"obstacle{len(obstacle_environment)}"

    if goal_pose is None:
        goal_pose = start_pose

    axes_occupancy = np.array([2*radius, 2*radius])
    num_ctrpt = 8
    control_points = init_position = np.zeros((num_ctrpt, 2))
    delta_angle = 2*np.pi/num_ctrpt

    for idx in range(num_ctrpt):
        control_points[idx] = [radius*math.cos(idx*delta_angle), radius*math.sin(idx*delta_angle)]

    shape_ = Ellipse(
        center_position=start_pose.position,
        margin_absolut=margin_absolut,
        orientation=0,
        tail_effect=False,
        repulsion_coeff=1,
        axes_length=axes_occupancy,
        name=name
    )

    new_circle = AssFurniture(
        parameter_file = parameter_file,
        shape=shape_,
        axes_occupancy=axes_occupancy,
        obstacle_environment=obstacle_environment,
        control_points=control_points,
        goal_pose=goal_pose,
        priority_value=1.0,
        name=name,
        object_type="circle",
    )

    return new_circle

def create_table(
    parameter_file: str,
    obstacle_environment: ObstacleContainer,
    start_pose: Pose,
    goal_pose: Optional[Pose] = None,
    name: str = "",
    margin_absolut: float = 0.2,
    axes_length=np.array([1.8, 0.8]),
    static: bool = False
) -> AssFurniture:
    if not len(name):
        name = f"obstacle{len(obstacle_environment)}"

    if goal_pose is None:
        goal_pose = start_pose

    ratio_ax_len = 0.5
    control_points = np.array([[-ratio_ax_len*axes_length[0], ratio_ax_len*axes_length[1]],
                                [0, ratio_ax_len*axes_length[1]],
                                [ratio_ax_len*axes_length[0], ratio_ax_len*axes_length[1]],
                                [ratio_ax_len*axes_length[0], 0],
                                [ratio_ax_len*axes_length[0], -ratio_ax_len*axes_length[1]],
                                [0, -ratio_ax_len*axes_length[1]],
                                [-ratio_ax_len*axes_length[0], -ratio_ax_len*axes_length[1]],
                                [-ratio_ax_len*axes_length[0], 0]])

    if static:
        repulsion_coeff = 1
    else:
        repulsion_coeff = 1

    shape_ = Polygon(
        edge_points=control_points.T,
        center_position=start_pose.position,
        margin_absolut=margin_absolut,
        orientation=start_pose.orientation,
        tail_effect=False,
        repulsion_coeff=1
    )

    new_furniture = AssFurniture(
        parameter_file = parameter_file,
        shape=shape_,
        axes_occupancy=axes_length,
        obstacle_environment=obstacle_environment,
        control_points=control_points,
        goal_pose=goal_pose,
        priority_value=1.0,
        name=name,
        object_type="table",
        static = static
    )

    return new_furniture

def add_walls(
    parameter_file, obstacle_environment, x_lim, y_lim, wall_width=0.5, wall_margin=0.5
):
    # adds walls to the simulation
    displacement_center = wall_width / 2
    center_left = [x_lim[0] - displacement_center, np.average(y_lim)]
    center_down = [np.average(x_lim), y_lim[0] - displacement_center]
    center_right = [x_lim[1] + displacement_center, np.average(y_lim)]
    center_up = [np.average(x_lim), y_lim[1] + displacement_center]
    center_array = [center_left, center_down, center_right, center_up]
    horizontal_wall_axis = np.array([x_lim[1] - x_lim[0], wall_width])
    vertical_wall_axis = np.array([wall_width, y_lim[1] - y_lim[0]])
    axis_array = [
        vertical_wall_axis,
        horizontal_wall_axis,
        vertical_wall_axis,
        horizontal_wall_axis,
    ]

    agent_list = []

    for i in range(4):
        wall_pose = Pose(
            position=center_array[i], orientation=0
        )

        '''
        wall_shape = Cuboid(
            axes_length=axis_array[i],
            center_position=wall_pose.position,
            margin_absolut=wall_margin,
            orientation=wall_pose.orientation,
            tail_effect=False,
            repulsion_coeff=1,
            linear_velocity=np.array([0.0, 0.0]),
            is_boundary=False,
        )
        '''

        control_points = np.array([[-axis_array[i][0] / 2, axis_array[i][1] / 2], 
                                   [axis_array[i][0] / 2, axis_array[i][1] / 2], 
                                   [axis_array[i][0] / 2, -axis_array[i][1] / 2], 
                                   [-axis_array[i][0] / 2, -axis_array[i][1] / 2]])

        wall_shape = Polygon(
            edge_points=control_points.T,
            center_position=wall_pose.position,
            margin_absolut=wall_margin,
            orientation=wall_pose.orientation,
            tail_effect=False,
            linear_velocity=np.array([0.0, 0.0]),
            is_boundary=False,
        )
        
        wall = AssFurniture(
            parameter_file=parameter_file,
            shape=wall_shape,
            axes_occupancy=axis_array[i],
            obstacle_environment=obstacle_environment,
            control_points=control_points,
            goal_pose=wall_pose,
            priority_value=1.0,
            name="wall",
            object_type="wall",
            static=True,
        )

        agent_list.append(wall)

    return agent_list