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
from autonomous_furniture.furniture_creators import add_walls, create_regular_polygon, create_polygon

def create_environment(args, obstacle_environment: ObstacleContainer, do_walls=True, do_person=True):

    parameter_file = (
        str(pathlib.Path(__file__).parent.resolve())
        + "/parameters/auto_assembly_irregular_polytopic_parts.yaml"
    )

    x_lim: list[float] = [-4.5, 4.5]
    y_lim: list[float] = [-4.5, 4.5]

    num_square = 15
    radius = args.r
    init_formation = []

    size_x = x_lim[1] - x_lim[0]
    size_y = y_lim[1] - y_lim[0]

    for idx in range(num_square):
        i = int(idx/4)
        j = idx%4
        new_pose = Pose((x_lim[0] + size_x/8 + j*size_x/4, y_lim[1] - size_y/8 - i*size_y/4), np.pi/4)
        init_formation.append(new_pose)

    rand_idx_0 = random.randint(0, 13)
    rand_idx_1 = random.randint(0, 13)
    rand_idx_2 = random.randint(0, 13)
    rand_idx_3 = random.randint(0, 13)
    rand_idx_4 = random.randint(0, 13)
    rand_idx_5 = random.randint(0, 13)
    rand_idx_6 = random.randint(0, 13)
    rand_idx_7 = random.randint(0, 13)
    rand_idx_8 = random.randint(0, 13)
    rand_idx_9 = random.randint(0, 13)
    rand_idx_10 = random.randint(0, 13)
    rand_idx_11 = random.randint(0, 13)
    rand_idx_12 = random.randint(0, 13)
    rand_idx_13 = random.randint(0, 13)
    rand_idx_14 = random.randint(0, 14)

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
    init_formation[rand_idx_14] = tmp

    tmp_goal_list = np.array([[-0.05079, 2.35866],
                                [-0.91127, 1.72204],
                                [-1.48024, 1.06579],
                                [-0.11207, 0.71586],
                                [0.53852, 1.61975],
                                [1.33676, 1.11766],
                                [-2.33464, 0.11357],
                                [-1.23159, -0.00372],
                                [0.04315, -0.63796],
                                [0.82118, -0.11648],
                                [2.03036, -0.17429],
                                [-1.78111, -0.57588],
                                [-1.28787, -1.18497],
                                [0.06355, -2.1203],
                                [1.29042, -1.54519]])

    vertices_shape_0 = 0.9*(np.array([[-0.77364, 2.22696],
                                            [-0.4, 2.6],
                                            [0, 3],
                                            [0.4, 2.6],
                                            [0.8, 2.2],
                                            [0.44247, 2.16489],
                                            [0.09185, 2.12545],
                                            [-0.04839, 1.81428],
                                            [-0.41967, 1.82797]]) - tmp_goal_list[0])
    
    vertices_shape_1 = (np.array([[-1.3, 1.7],
                                            [-1.11918, 1.88489],
                                            [-0.95, 2.05],
                                            [-0.76724, 1.85647],
                                            [-0.58, 1.66],
                                            [-0.73871, 1.40724],
                                            [-1.01603, 1.5542]]) - tmp_goal_list[1])
    
    vertices_shape_2 = (np.array([[-2.1, 0.9],
                                            [-1.8, 1.2],
                                            [-1.50091, 1.50178],
                                            [-1.20796, 1.35228],
                                            [-0.87874, 1.18454],
                                            [-1, 1],
                                            [-1.2719, 0.83032],
                                            [-1.6, 0.6],
                                            [-1.8417, 0.7504]]) - tmp_goal_list[2])
    
    vertices_shape_3 = 0.95*(np.array([[-1.2259, 0.55357],
                                            [-0.81005, 0.82997],
                                            [-0.57133, 1.20074],
                                            [-0.34347, 1.57543],
                                            [-0.06634, 1.58145],
                                            [0.04752, 1.14694],
                                            [0.54244, 1.05421],
                                            [0.63647, 0.78425],
                                            [0.75266, 0.46447],
                                            [0.53801, 0.19599],
                                            [0.35456, -0.06298],
                                            [-0.05329, 0.26504],
                                            [-0.5, 0.5],
                                            [-0.94281, 0.32155]]) - tmp_goal_list[3])
    
    vertices_shape_4 = 0.85*(np.array([[0.13568, 1.66088],
                                            [0.2584, 1.92823],
                                            [0.59946, 1.9895],
                                            [0.95, 2.05],
                                            [0.88512, 1.56885],
                                            [0.6211, 1.26087],
                                            [0.24963, 1.30589]]) - tmp_goal_list[4])

    vertices_shape_5 = (np.array([[0.75434, 1.06197],
                                            [0.93002, 1.27758],
                                            [1.0784, 1.47593],
                                            [1.12011, 1.87853],
                                            [1.55, 1.45],
                                            [1.9, 1.1],
                                            [1.6055, 0.72398],
                                            [1.29576, 0.63431],
                                            [0.98883, 0.54126],
                                            [0.87678, 0.79846]]) - tmp_goal_list[5])
    
    vertices_shape_6 = (np.array([[-3, 0],
                                            [-2.6, 0.4],
                                            [-2.25077, 0.74984],
                                            [-1.97854, 0.56392],
                                            [-1.7, 0.4],
                                            [-1.7, 0],
                                            [-1.9421, -0.09437],
                                            [-2.2, -0.2],
                                            [-2.20427, -0.47134],
                                            [-2.2, -0.8],
                                            [-2.6, -0.4]]) - tmp_goal_list[6])
    
    vertices_shape_7 = 0.85*(np.array([[-1.47632, 0.4107],
                                            [-1.18762, 0.22702],
                                            [-0.9446, 0.07377],
                                            [-0.95015, -0.21396],
                                            [-0.94987, -0.54217],
                                            [-1.2179, -0.32369],
                                            [-1.47076, -0.12134],
                                            [-1.47309, 0.14082]]) - tmp_goal_list[7])
    
    vertices_shape_8 = (np.array([[-0.8813, -0.93742],
                                            [-0.77428, -0.51468],
                                            [-0.77968, -0.17607],
                                            [-0.77853, 0.14188],
                                            [-0.48824, 0.25891],
                                            [0, 0],
                                            [0.33242, -0.32335],
                                            [0.69373, -0.48962],
                                            [1.05, -0.65],
                                            [1, -1.2],
                                            [0.4, -1.3],
                                            [0.08437, -1.1514],
                                            [-0.19109, -1.00515],
                                            [-0.60378, -1.25979]]) - tmp_goal_list[8])

    vertices_shape_9 = 0.85*(np.array([[0.53272, -0.208], 
                                            [0.72057, 0.05346],
                                            [0.92886, 0.35548],
                                            [1.01248, -0.06623],
                                            [1.09279, -0.45388],
                                            [0.7906, -0.3177]]) - tmp_goal_list[9])
    
    vertices_shape_10 = (np.array([[1.13762, 0.33169],
                                            [1.46951, 0.43101],
                                            [1.74189, 0.50682],
                                            [2.1, 0.9],
                                            [2.4, 0.6],
                                            [2.8, 0.2],
                                            [3, 0],
                                            [2.65, -0.35],
                                            [2.22268, -0.79931],
                                            [1.8, -1.2],
                                            [1.22956, -1.20099],
                                            [1.29542, -0.68384],
                                            [1.23967, -0.16969]]) - tmp_goal_list[10])
    
    vertices_shape_11 = 0.8*(np.array([[-2, -1],
                                            [-2.00623, -0.63095],
                                            [-2.00273, -0.35781],
                                            [-1.67309, -0.22626],
                                            [-1.46798, -0.39493],
                                            [-1.63598, -0.80168]]) - tmp_goal_list[11])
    
    vertices_shape_12 = (np.array([[-1.85, -1.15],
                                            [-1.66053, -1.03794],
                                            [-1.47788, -0.93603],
                                            [-1.39912, -0.72414],
                                            [-1.32529, -0.52726],
                                            [-1.05, -0.75],
                                            [-1.12387, -1.01881],
                                            [-0.91201, -1.2649],
                                            [-1.04005, -1.51621],
                                            [-1.2, -1.8],
                                            [-1.5, -1.5]]) - tmp_goal_list[12])
    
    vertices_shape_13 = (np.array([[-1, -2],
                                            [-0.74762, -1.4722],
                                            [-0.45624, -1.4555],
                                            [-0.17233, -1.26602],
                                            [0.31201, -1.54035],
                                            [0.80029, -1.474],
                                            [1, -2],
                                            [0.7, -2.3],
                                            [0.4, -2.6],
                                            [0, -3],
                                            [-0.3, -2.7],
                                            [-0.6, -2.4]]) - tmp_goal_list[13])
    
    vertices_shape_14 = 0.75*(np.array([[0.94252, -1.43401],
                                            [1.28167, -1.39087],
                                            [1.65508, -1.34598],
                                            [1.37479, -1.63306],
                                            [1.11328, -1.9087],
                                            [1.02335, -1.65877]]) - tmp_goal_list[14])


    goal_formation = [Pose((tmp_goal_list[0][0], tmp_goal_list[0][1]), 0),
                        Pose((tmp_goal_list[1][0], tmp_goal_list[1][1]), 0),
                        Pose((tmp_goal_list[2][0], tmp_goal_list[2][1]), 0),
                        Pose((tmp_goal_list[3][0], tmp_goal_list[3][1]), 0),

                        Pose((tmp_goal_list[4][0], tmp_goal_list[4][1]), 0),
                        Pose((tmp_goal_list[5][0], tmp_goal_list[5][1]), 0),
                        Pose((tmp_goal_list[6][0], tmp_goal_list[6][1]), 0),
                        Pose((tmp_goal_list[7][0], tmp_goal_list[7][1]), 0),
                        Pose((tmp_goal_list[8][0], tmp_goal_list[8][1]), 0),
                        Pose((tmp_goal_list[9][0], tmp_goal_list[9][1]), 0),
                        Pose((tmp_goal_list[10][0], tmp_goal_list[10][1]), 0),
                        Pose((tmp_goal_list[11][0], tmp_goal_list[11][1]), 0),
                        Pose((tmp_goal_list[12][0], tmp_goal_list[12][1]), 0),
                        Pose((tmp_goal_list[13][0], tmp_goal_list[13][1]), 0),
                        Pose((tmp_goal_list[14][0], tmp_goal_list[14][1]), 0),]
                             
    mobile_agent_margin = 0.18
    wall_margin = 0.16
    wall_width = 0.5

    agent_list = []

    agent_0 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[0].position,
            orientation = init_formation[0].orientation,
        ),
        vertices = vertices_shape_0,
        goal_pose = Pose(
            position = goal_formation[0].position,
            orientation = goal_formation[0].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_0)

    agent_1 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[1].position,
            orientation = init_formation[1].orientation,
        ),
        vertices = vertices_shape_1,
        goal_pose = Pose(
            position = goal_formation[1].position,
            orientation = goal_formation[1].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_1)

    agent_2 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[2].position,
            orientation = init_formation[2].orientation,
        ),
        vertices = vertices_shape_2,
        goal_pose = Pose(
            position = goal_formation[2].position,
            orientation = goal_formation[2].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_2)

    agent_3 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[3].position,
            orientation = init_formation[3].orientation,
        ),
        vertices = vertices_shape_3,
        goal_pose = Pose(
            position = goal_formation[3].position,
            orientation = goal_formation[3].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_3)

    agent_4 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[4].position,
            orientation = init_formation[4].orientation,
        ),
        vertices = vertices_shape_4,
        goal_pose = Pose(
            position = goal_formation[4].position,
            orientation = goal_formation[4].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_4)

    agent_5 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[5].position,
            orientation = init_formation[5].orientation,
        ),
        vertices = vertices_shape_5,
        goal_pose = Pose(
            position = goal_formation[5].position,
            orientation = goal_formation[5].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_5)

    agent_6 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[6].position,
            orientation = init_formation[6].orientation,
        ),
        vertices = vertices_shape_6,
        goal_pose = Pose(
            position = goal_formation[6].position,
            orientation = goal_formation[6].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_6)

    agent_7 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[7].position,
            orientation = init_formation[7].orientation,
        ),
        vertices = vertices_shape_7,
        goal_pose = Pose(
            position = goal_formation[7].position,
            orientation = goal_formation[7].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_7)

    agent_8 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[8].position,
            orientation = init_formation[8].orientation,
        ),
        vertices = vertices_shape_8,
        goal_pose = Pose(
            position = goal_formation[8].position,
            orientation = goal_formation[8].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_8)

    agent_9 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[9].position,
            orientation = init_formation[9].orientation,
        ),
        vertices = vertices_shape_9,
        goal_pose = Pose(
            position = goal_formation[9].position,
            orientation = goal_formation[9].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_9)

    agent_10 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[10].position,
            orientation = init_formation[10].orientation,
        ),
        vertices = vertices_shape_10,
        goal_pose = Pose(
            position = goal_formation[10].position,
            orientation = goal_formation[10].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_10)

    agent_11 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[11].position,
            orientation = init_formation[11].orientation,
        ),
        vertices = vertices_shape_11,
        goal_pose = Pose(
            position = goal_formation[11].position,
            orientation = goal_formation[11].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_11)

    agent_12 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[12].position,
            orientation = init_formation[12].orientation,
        ),
        vertices = vertices_shape_12,
        goal_pose = Pose(
            position = goal_formation[12].position,
            orientation = goal_formation[12].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_12)

    agent_13 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[13].position,
            orientation = init_formation[13].orientation,
        ),
        vertices = vertices_shape_13,
        goal_pose = Pose(
            position = goal_formation[13].position,
            orientation = goal_formation[13].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_13)

    agent_14 = create_polygon(
        parameter_file = parameter_file, 
        obstacle_environment = obstacle_environment,
        start_pose = Pose(
            position = init_formation[14].position,
            orientation = init_formation[14].orientation,
        ),
        vertices = vertices_shape_14,
        goal_pose = Pose(
            position = goal_formation[14].position,
            orientation = goal_formation[14].orientation,
        ),
        margin_absolut = mobile_agent_margin,
    )       
    agent_list.append(agent_14)
    
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

    return agent_list, user_id_list, x_lim, y_lim, wall_width, num_square

    

def full_animation(args, add_human = True, add_wall = True, no_clip = False, logs = False):

    my_animation = DynamicalSystemAnimation(
        it_max=3000,
        dt_simulation=0.05,
        dt_sleep=0.001,
        animation_name="_regroup_2D_dense_animation",
        file_type=".gif",
    )

    plt.ion()

    obstacle_environment = ObstacleContainer()
    agent_list, user_id_list, x_lim, y_lim, wall_width, num_goals = create_environment(args, obstacle_environment, 
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

        logs_dir = os.path.join(os.getcwd(), "data/dense_reconfiguration_non_convex/vpfm")
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)
            logs_number = 0
        else:
            logs_number = len(os.listdir(logs_dir))
        logs_file = os.path.join(logs_dir, f"_{logs_number:03}.yaml")

        my_animation.logs(logs_file)


if (__name__) == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--r", type=float, default=0.4)
    args = parser.parse_args()

    plt.close("all")
    full_animation(args, add_human = False, add_wall = False, no_clip = False, logs = False)