import pathlib

import math
import copy
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
import argparse
import random

from dynamic_obstacle_avoidance.containers import ObstacleContainer
from autonomous_furniture.agent import Person
from autonomous_furniture.dynamical_system_animation import DynamicalSystemAnimation
from vartools.states import Pose
from autonomous_furniture.furniture_creators import create_circle, add_walls, create_regular_polygon

def create_environment(args, obstacle_environment: ObstacleContainer, do_walls = True, do_person = True):

    parameter_file = (
        str(pathlib.Path(__file__).parent.resolve())
        + "/parameters/antipodal_position_switching.yaml"
    )

    x_lim: list[float] = [0, 10]
    y_lim: list[float] = [0, 10] 

    wall_width = 0.5
    wall_margin= 0.3
    radius = args.r
    agent_margin = 0.3
    num_agent = 10
    r_formation = 4

    init_position = np.zeros((num_agent, 2))
    goal_position = np.zeros((num_agent, 2))
    delta_angle = 2*np.pi/num_agent

    for idx in range(num_agent):
        init_position[idx] = [5 + r_formation*math.cos(idx*delta_angle),
                            5 + r_formation*math.sin(idx*delta_angle)]
        goal_position[idx] = [5 - r_formation*math.cos(idx*delta_angle),
                            5 - r_formation*math.sin(idx*delta_angle)]

    agent_list = []
    for idx in range(num_agent):
        agent = create_regular_polygon(
            parameter_file = parameter_file, 
            obstacle_environment = obstacle_environment,
            radius = radius,
            num_vertex = idx%int(num_agent/2)+3,
            start_pose = Pose(
                position = init_position[idx],
                orientation = np.pi - idx*delta_angle,
            ),
            goal_pose = Pose(
                position = goal_position[idx],
                orientation = idx*delta_angle,
            ),
            margin_absolut = agent_margin,
            name="agent_" + str(idx)
        )
        agent_list.append(agent)
    
    if do_walls:
        wall_list = add_walls(
            parameter_file = parameter_file,
            obstacle_environment = obstacle_environment,
            x_lim = x_lim,
            y_lim = y_lim,
            wall_width = wall_width,
            wall_margin = wall_margin
        )

        for wall in wall_list:
            agent_list.append(wall)
    else:
        wall_width = 0

    user_id_list = []
    #do_person = False
    if do_person:
        person = Person(
            parameter_file = parameter_file,
            center_position=[2.6, 0.0],
            velocity = np.array([-0.3, 0]),
            priority_value=1e3,
            goal_pose=Pose((-2.6, 0.0), 0.0),
            obstacle_environment=obstacle_environment,
            margin=0.2,
            name="user",
        )
        agent_list.append(person)
        user_id_list.append(person.user_id)

    return agent_list, user_id_list, x_lim, y_lim, wall_width, num_agent, radius

def full_animation(args, add_human = True, add_wall = True, no_clip = False, logs = False):

    my_animation = DynamicalSystemAnimation(
        it_max=6000,
        dt_simulation=0.1,
        dt_sleep=0.001,
        animation_name="_real_2D_animation",
        file_type=".gif",
    )

    obstacle_environment = ObstacleContainer()
    agent_list, user_id_list, x_lim, y_lim, wall_width, num_goals, radius = create_environment(args, obstacle_environment, do_walls=add_wall, do_person=add_human)
    
    total_enlargement = wall_width / 2
    x_lim_anim = [x_lim[0] - total_enlargement, x_lim[1] + total_enlargement]
    y_lim_anim = [y_lim[0] - total_enlargement, y_lim[1] + total_enlargement]
    
    if no_clip:
        my_animation.setup(
            obstacle_environment = obstacle_environment,
            agent = agent_list,
            user_id_list = user_id_list,
            num_goals = num_goals,
            x_lim=x_lim_anim,
            y_lim=y_lim_anim,
            anim=False
        )

        my_animation.run_no_clip(save_animation=False)

    else:
        plt.ion()
        # generate color list of animation
        n_agents = len(agent_list)
        if add_wall:
            if add_human:
                n_agents -= 5 
            else:
                n_agents -= 4
        cm = plt.get_cmap("gist_rainbow")
        color_list = [cm(1.0 * ii / n_agents) for ii in range(n_agents)]

        if add_wall:
            for i in range(4):
                color_list.append("grey")

        if add_human:
            color_list.append("red") # for human

        my_animation.setup(
            obstacle_environment=obstacle_environment,
            agent=agent_list,
            user_id_list = user_id_list,
            num_goals = num_goals,
            x_lim=x_lim_anim,
            y_lim=y_lim_anim,
            figsize=(10, 8),
            obstacle_colors=color_list
        )

        my_animation.run(save_animation=False)
    
    if logs:

        logs_dir = os.path.join(os.getcwd(), 'data/pure_collision_avoidance/dsm_new_speed', "r_" + str(radius))
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)
            logs_number = 0
        else:
            logs_number = len(os.listdir(logs_dir))
        logs_file = os.path.join(logs_dir, "r_" + str(radius) + f"_{logs_number:03}.yaml")

        my_animation.logs(logs_file)

if (__name__) == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--r", type=float, default=0.9)
    args = parser.parse_args()

    plt.close("all")
    full_animation(args, add_human = False, add_wall = False, logs=False, no_clip=False)
