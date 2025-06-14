import math
import random
import copy
import pathlib
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import argparse
import random
from numpy import linalg as LA
from dynamic_obstacle_avoidance.containers import ObstacleContainer
from autonomous_furniture.agent import Person
from autonomous_furniture.dynamical_system_animation import DynamicalSystemAnimation
from vartools.states import Pose
from autonomous_furniture.furniture_creators import create_chair, create_table, add_walls

def create_environment(obstacle_environment: ObstacleContainer, do_walls=True, do_person=True):

    parameter_file = (
        str(pathlib.Path(__file__).parent.resolve())
        + "/parameters/clear_way_for_user.yaml"
    )

    x_lim: list[float] = [-4.5, 5.0]
    y_lim: list[float] = [-2.0, 2.0]

    table_nb = 6
    table_size = [0.65,0.55]
    chair_nb = 12
    chair_size = [0.5,0.45] 
    big_table_nb = 0
    big_table_size = [1.2,0.6]

    agent_size_list = [table_size]*table_nb + [chair_size]*chair_nb + [big_table_size]*big_table_nb

    goal_setup = [Pose((-2.25, 0.0), 0.0),
                             Pose((-1.35,0.0), 0.0),
                             Pose((-0.45,0.0), 0.0),
                             Pose((0.45, 0.0), 0.0),
                             Pose((1.35, 0.0), 0.0),
                             Pose((2.25, 0.0), 0.0),
                             
                             Pose((-2.25, 0.9), -0.5*math.pi),
                             Pose((-1.35, 0.9), -0.5*math.pi),
                             Pose((-0.45,0.9), -0.5*math.pi),
                             Pose((0.45, 0.9), -0.5*math.pi),
                             Pose((1.35, 0.9), -0.5*math.pi),
                             Pose((2.25, 0.9), -0.5*math.pi),

                             Pose((-2.25, -0.9), 0.5*math.pi),
                             Pose((-1.35, -0.9), 0.5*math.pi),
                             Pose((-0.45,-0.9), 0.5*math.pi),
                             Pose((0.45,-0.9), 0.5*math.pi),
                             Pose((1.35,-0.9), 0.5*math.pi),
                             Pose((2.25,-0.9), 0.5*math.pi),]
    
    init_setup = copy.deepcopy(goal_setup)
    
    num_goals = len(goal_setup)
    mobile_agent_margin = 0.18
    wall_margin = 0.2
    wall_width = 0.5

    agent_list = []

    for i in range(table_nb+chair_nb+big_table_nb):
        if i < table_nb:
            agent = create_table(
                parameter_file = parameter_file, 
                obstacle_environment = obstacle_environment,
                start_pose = init_setup[i],
                goal_pose = goal_setup[i],
                margin_absolut = mobile_agent_margin,
                axes_length = agent_size_list[i],
                name="table_"+str(i+1)    
            )
        elif i < table_nb+chair_nb:
            agent = create_chair(
                parameter_file = parameter_file, 
                obstacle_environment = obstacle_environment,
                start_pose = init_setup[i],
                goal_pose = goal_setup[i],
                margin_absolut = mobile_agent_margin,
                axes_length = agent_size_list[i],
                name="chair_"+str(i+1)
            )       
        else:
            agent = create_table(
                parameter_file = parameter_file, 
                obstacle_environment = obstacle_environment,
                start_pose = init_setup[i],
                goal_pose = goal_setup[i],
                margin_absolut = mobile_agent_margin,
                axes_length = agent_size_list[i],
                name="table_"+str(i+1)    
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
    
    if do_person:

        start_position = np.array([-3.75, random.uniform(-1.3, 1.3)])
        end_position = np.array([3.75, random.uniform(-1.3, 1.3)])
        norm_dir = (end_position - start_position)/LA.norm(end_position - start_position)
        vel_value = 0.5
        vel = vel_value*norm_dir

        person = Person(
            parameter_file = parameter_file,
            center_position=start_position,
            velocity = vel,
            priority_value=1e3,
            goal_pose=Pose(end_position, 0.0),
            obstacle_environment=obstacle_environment,
            margin=0.25,
            name="user",
        )
        agent_list.append(person)
        user_id_list.append(person.user_id)

    return agent_list, user_id_list, x_lim, y_lim, wall_width, num_goals


def full_animation(add_human = True, add_wall = True, no_clip = False, logs = False):

    my_animation = DynamicalSystemAnimation(
        it_max=1000,
        dt_simulation=0.05,
        dt_sleep=0.001,
        animation_name="_regroup_2D_dense_animation",
        file_type=".gif",
    )

    plt.ion()

    obstacle_environment = ObstacleContainer()
    agent_list, user_id_list, x_lim, y_lim, wall_width, num_goals = create_environment(obstacle_environment, 
                                                                                do_walls=add_wall, 
                                                                                do_person=add_human)

    total_enlargement = wall_width / 2
    x_lim_anim = [x_lim[0] - total_enlargement, x_lim[1] + total_enlargement]
    y_lim_anim = [y_lim[0] - total_enlargement, y_lim[1] + total_enlargement]
    
    if no_clip:
        my_animation.setup(
            obstacle_environment=obstacle_environment,
            agent=agent_list,
            user_id_list=user_id_list,
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
            user_id_list=user_id_list,
            num_goals = num_goals,
            x_lim=x_lim_anim,
            y_lim=y_lim_anim,
            figsize=(10, 8),
            obstacle_colors=color_list
        )

        my_animation.run(save_animation=False)
    
    if logs:

        logs_dir = os.path.join(os.getcwd(), 'data', "clear_way/newavoid_safety" + str(num_goals))
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)
            logs_number = 0
        else:
            logs_number = len(os.listdir(logs_dir))
        logs_file = os.path.join(logs_dir, "newavoid_safety_" + str(num_goals) + f"_{logs_number:03}.yaml")

        my_animation.logs(logs_file)


if (__name__) == "__main__":

    plt.close("all")
    full_animation(add_human = True, logs = False, no_clip = False)