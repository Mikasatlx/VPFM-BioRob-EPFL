import math
import copy
import pathlib
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import argparse
import random
from dynamic_obstacle_avoidance.containers import ObstacleContainer
from autonomous_furniture.agent import Person
from autonomous_furniture.dynamical_system_animation import DynamicalSystemAnimation
from vartools.states import Pose
from autonomous_furniture.furniture_creators import add_walls, create_regular_polygon

def create_environment(args, obstacle_environment: ObstacleContainer, do_walls=True, do_person=True):

    parameter_file = (
        str(pathlib.Path(__file__).parent.resolve())
        + "/parameters/dense_reconfiguration_regular_hexegon.yaml"
    )

    x_lim: list[float] = [-3.0, 3.0]
    y_lim: list[float] = [-3.0, 3.0]

    num_hexegon = 19
    radius = args.r
    init_formation = []

    size_x = x_lim[1] - x_lim[0]
    size_y = y_lim[1] - y_lim[0]

    for idx in range(num_hexegon):
        i = int(idx/5)
        j = idx%5
        new_pose = Pose((x_lim[0] + size_x/10 + j*size_x/5, y_lim[1] - size_y/10 - i*size_y/5), 0.0)
        init_formation.append(new_pose)

    rand_idx_0 = random.randint(0, 18)
    rand_idx_1 = random.randint(0, 18)
    rand_idx_2 = random.randint(0, 18)
    rand_idx_3 = random.randint(0, 18)
    rand_idx_4 = random.randint(0, 18)
    rand_idx_5 = random.randint(0, 18)
    rand_idx_6 = random.randint(0, 18)
    rand_idx_7 = random.randint(0, 18)
    rand_idx_8 = random.randint(0, 18)
    rand_idx_9 = random.randint(0, 18)
    rand_idx_10 = random.randint(0, 18)
    rand_idx_11 = random.randint(0, 18)
    rand_idx_12 = random.randint(0, 18)
    rand_idx_13 = random.randint(0, 18)
    rand_idx_14 = random.randint(0, 18)
    rand_idx_15 = random.randint(0, 18)
    rand_idx_16 = random.randint(0, 18)
    rand_idx_17 = random.randint(0, 18)
    rand_idx_18 = random.randint(0, 18)
    tmp = init_formation[rand_idx_0]
    init_formation[rand_idx_0] = init_formation[rand_idx_1]
    init_formation[rand_idx_1] = init_formation[rand_idx_2]
    init_formation[rand_idx_2] = init_formation[rand_idx_3]
    init_formation[rand_idx_3] = init_formation[rand_idx_4]
    init_formation[rand_idx_4] = init_formation[rand_idx_5]
    init_formation[rand_idx_5] = init_formation[rand_idx_6]
    init_formation[rand_idx_6] = init_formation[rand_idx_7]
    init_formation[rand_idx_7] = init_formation[rand_idx_8]
    init_formation[rand_idx_8] = init_formation[rand_idx_9]
    init_formation[rand_idx_9] = init_formation[rand_idx_10]
    init_formation[rand_idx_10] = init_formation[rand_idx_11]
    init_formation[rand_idx_11] = init_formation[rand_idx_12]
    init_formation[rand_idx_12] = init_formation[rand_idx_13]
    init_formation[rand_idx_13] = init_formation[rand_idx_14]
    init_formation[rand_idx_14] = init_formation[rand_idx_15]
    init_formation[rand_idx_15] = init_formation[rand_idx_16]
    init_formation[rand_idx_16] = init_formation[rand_idx_17]
    init_formation[rand_idx_17] = init_formation[rand_idx_18]
    init_formation[rand_idx_18] = tmp

    x_offset_hexegon = 0
    
    goal_formation_hexegon = [Pose((-1+x_offset_hexegon, 1.732), np.pi/6),
                              Pose((0+x_offset_hexegon, 1.732), np.pi/6),
                              Pose((1+x_offset_hexegon, 1.732), np.pi/6),

                              Pose((-1.5+x_offset_hexegon, 0.866), np.pi/6),
                              Pose((-0.5+x_offset_hexegon, 0.866), np.pi/6),
                              Pose((0.5+x_offset_hexegon, 0.866), np.pi/6),
                              Pose((1.5+x_offset_hexegon, 0.866), np.pi/6),

                              Pose((-2.0+x_offset_hexegon, 0.0), np.pi/6),
                              Pose((-1.0+x_offset_hexegon, 0.0), np.pi/6),
                              Pose((0.0+x_offset_hexegon, 0.0), np.pi/6),
                              Pose((1.0+x_offset_hexegon,0.0), np.pi/6),
                              Pose((2.0+x_offset_hexegon, 0.0), np.pi/6),

                              Pose((-1.5+x_offset_hexegon, -0.866), np.pi/6),
                              Pose((-0.5+x_offset_hexegon, -0.866), np.pi/6),
                              Pose((0.5+x_offset_hexegon, -0.866), np.pi/6),
                              Pose((1.5+x_offset_hexegon, -0.866), np.pi/6),

                              Pose((-1+x_offset_hexegon, -1.732), np.pi/6),
                              Pose((0+x_offset_hexegon, -1.732), np.pi/6),
                              Pose((1+x_offset_hexegon, -1.732), np.pi/6)]
    
    mobile_agent_margin = 0.15
    wall_margin = 0.15
    wall_width = 0.5

    agent_list = []

    for idx in range(num_hexegon):
        agent = create_regular_polygon(
            parameter_file = parameter_file, 
            obstacle_environment = obstacle_environment,
            radius = radius,
            num_vertex = 6,
            start_pose = Pose(
                position = init_formation[idx].position,
                orientation = init_formation[idx].orientation,
            ),
            goal_pose = Pose(
                position = goal_formation_hexegon[idx].position,
                orientation = goal_formation_hexegon[idx].orientation,
            ),
            margin_absolut = mobile_agent_margin,
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
    
    if do_person:
        person = Person(
            parameter_file = parameter_file,
            center_position=[-3.1, 0],
            velocity = np.array([0.3, 0]),
            priority_value=1e3,
            goal_pose=Pose((3.1, 0.0), 0.0),
            obstacle_environment=obstacle_environment,
            margin=mobile_agent_margin,
            name="user",
        )
        agent_list.append(person)
        user_id_list.append(person.user_id)

    return agent_list, user_id_list, x_lim, y_lim, wall_width, num_hexegon

    

def full_animation(args, add_human = True, add_wall = True, no_clip = False, logs = False):

    my_animation = DynamicalSystemAnimation(
        it_max=2000,
        dt_simulation=0.05,
        dt_sleep=0.001,
        animation_name="_regroup_2D_dense_animation",
        file_type=".gif",
    )

    plt.ion()

    obstacle_environment = ObstacleContainer()
    agent_list, user_id_list, x_lim, y_lim, wall_width, num_goals = create_environment(args, 
                                                                                obstacle_environment, 
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

        logs_dir = os.path.join(os.getcwd(), "data/dense_reconfiguration_hexegon/", "r_" + str(args.r))
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)
            logs_number = 0
        else:
            logs_number = len(os.listdir(logs_dir))
        logs_file = os.path.join(logs_dir, "r_" + str(args.r) + f"_{logs_number:03}.yaml")

        my_animation.logs(logs_file)


if (__name__) == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--r", type=float, default=0.4)
    args = parser.parse_args()

    plt.close("all")
    full_animation(args, add_human = False, add_wall = False, no_clip = False, logs = False)