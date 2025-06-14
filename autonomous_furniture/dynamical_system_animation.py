from typing import Optional
import numpy as np
import json
import matplotlib.pyplot as plt
from numpy import linalg as LA
import time
from vartools.animator import Animator
from dynamic_obstacle_avoidance.visualization import plot_obstacles

from autonomous_furniture.agent import BaseAgent


class DynamicalSystemAnimation(Animator):
    def __init__(
        self,
        it_max: int = 100,
        iterator: Optional[int] = None,
        dt_simulation: float = 0.1,
        dt_sleep: float = 0.1,
        animation_name: str = "",
        file_type=".mp4",
    ) -> None:
        super().__init__(
            it_max, iterator, dt_simulation, dt_sleep, animation_name, file_type
        )

        self.it_final = it_max - 1 # By default set to it_max, value is changed in metrics method
        self.check_collision = True  # check collision or not
        self.collision_stop = False # stop the animation due to collision
        self.check_deadlock = True # check for deadlock or not
        self.deadlock_count = 2400 # number of steps to check for a deadlock 
        self.deadlock_stop = False # stop the animation due to deadlock
        self.execution_time_list = []

    def setup(
        self,
        obstacle_environment,
        agent,
        user_id_list,
        num_goals,
        x_lim=None,
        y_lim=None,
        anim: bool = True,
        check_convergence: bool = True,
        obstacle_colors=["orange", "blue", "red"],
        figsize=(3.0, 2.5)
    ):
        self.check_convergence = check_convergence # check converge or not

        self.number_agent = len(agent)

        if y_lim is None:
            y_lim = [0.0, 10]
        if x_lim is None:
            x_lim = [0, 10]

        self.agent = agent
        self.user_id_list = user_id_list
        self.x_lim = x_lim
        self.y_lim = y_lim

        self.agent_pos_saver = []
        for i in range(self.number_agent):
            self.agent_pos_saver.append([])
        for i in range(self.number_agent):
            self.agent_pos_saver[i].append(self.agent[i].position)

        self.obstacle_environment = obstacle_environment
        # for i in range(len(obstacle_environment)):
        #     self.obstacle_colors.append(np.array(np.random.choice(range(255),size=3))/254)
        self.obstacle_colors = obstacle_colors

        if anim:
            self.fig, self.ax = plt.subplots(figsize=figsize, dpi=120)

        self.converged: bool = False  # IF all the agent has converged

    def update_step(self, ii, anim: bool = True):
        
        for jj in range(self.number_agent):

            if self.agent[jj].object_type == "user":
                continue
            
            if self.agent[jj].static:
                continue

            # if there is no user, do not update the attractor because the last agent in the container is not a user
            if len(self.user_id_list) != 0:
                self.agent[jj].update_attractor_velocity(user_id_list = self.user_id_list)
                self.agent[jj].update_attractor(time_step=self.dt_simulation)
            
        execution_time = 0

        for jj in range(self.number_agent):
            start_time = time.time()
            self.agent[jj].update_velocity(
                time_step=self.dt_simulation,
            )
            end_time = time.time()
            execution_time = execution_time + end_time - start_time
        
        for jj in range(self.number_agent):
            self.agent[jj].compute_metrics(self.dt_simulation)
            self.agent[jj].do_velocity_step(self.dt_simulation)
            self.agent_pos_saver[jj].append(self.agent[jj].position)
        
        self.execution_time_list.append(execution_time)
                       
        if not anim:
            return

        self.plot_animation()

    def has_converged(self, it: int) -> bool:
        if not self.check_convergence:
            return False

        if self.check_collision:
            for jj in range(self.number_agent):
                if self.agent[jj].check_collision():
                    self.collision_stop = True

        if self.collision_stop:
            self.it_final = it + 1 # Because it starts at 0
            print(f"Collision at iteration={self.it_final}.")
            return True

        if self.check_deadlock and it > self.deadlock_count:
            self.deadlock_stop = True
        
        if self.deadlock_stop:
            self.it_final = it + 1 # Because it starts at 0
            print(f"Deadlock at iteration={self.it_final}.")
            return True

        self.converged = True

        for ii in range(self.number_agent):
            if self.agent[ii].converged == False:
                self.converged = False

        if self.converged:
            self.it_final = it + 1  # Because it starts at 0
            print(f"All trajectories converged at iteration={self.it_final}.")
            return True
        else:
            return False

    def logs(self, logs_file_path):
        experiment_data = {}

        # add data to file structure
        experiment_data['it_final'] = self.it_final
        experiment_data['converged'] = self.converged
        experiment_data['collision'] = self.collision_stop
        experiment_data['deadlock'] = self.deadlock_stop
        experiment_data['gamma_critic'] = self.agent[0].gamma_critic
        experiment_data['execution_time'] = np.mean(self.execution_time_list)
        experiment_data['execution_time_std'] = np.std(self.execution_time_list)
        #experiment_data['max_lin_vel'] = self.agent[0].maximum_linear_velocity
        
        time_max = 0
        avg_distance_ratio = 0
        avg_distance = 0
        num_agent = 0
        num_converge = 0
        num_distance_ratio = 0
        for agent in self.agent:
            if not agent.static and agent.object_type != "user":
                num_agent = num_agent + 1
                avg_distance = avg_distance + agent.total_distance
                if agent.converged:
                    num_converge = num_converge + 1
                    if agent.direct_distance > 0:
                        num_distance_ratio = num_distance_ratio + 1
                        distance_ratio = agent.total_distance / agent.direct_distance
                        avg_distance_ratio = avg_distance_ratio + distance_ratio

                if time_max < agent.time_conv:
                    time_max = agent.time_conv
        
        experiment_data['time_total'] = time_max
        experiment_data['num_agent'] = num_agent
        experiment_data['avg_distance'] = avg_distance/num_agent
        if self.converged:
            experiment_data['avg_distance_ratio'] = avg_distance_ratio/num_distance_ratio
        else:
            experiment_data['avg_distance_ratio'] = 0
        experiment_data['converge_rate'] = num_converge/num_agent
        
        with open(logs_file_path, "w") as logs_file:
            json.dump(experiment_data, logs_file, indent=4)

    def run_no_clip(self, save_animation: bool = False) -> None:
        """Runs the without visualization
        --- this function has been recreated what I expected it to be..."""
        self.it_count = 0
        while self.it_max is None or self.it_count < self.it_max:
            self.update_step(self.it_count, anim=False)

            # Check convergence
            if self.has_converged(self.it_count):
                break

            self.it_count += 1

    def plot_animation(self):
        self.ax.clear()

        for jj in range(self.number_agent):
            goal_control_points = self.agent[
                jj
            ].get_goal_control_points()  ##plot agent center position

            if len(self.obstacle_colors) > jj:
                color = self.obstacle_colors[jj]
            else:
                color = "black"

            global_control_points = self.agent[jj].get_global_control_points()
            self.ax.plot(
                global_control_points[0, :],
                global_control_points[1, :],
                color=color,
                marker="o",
            )

            self.ax.plot(
                goal_control_points[0, :],
                goal_control_points[1, :],
                color=color,
                marker="o",
                linestyle="",  ##k=black, o=dot
            )

            self.ax.plot(
                self.agent[jj].position[0],
                self.agent[jj].position[1],
                color=color,
                marker="*",
                linestyle="",  ##k=black, o=dot
            )

            x_values = np.zeros(len(self.agent_pos_saver[jj]))
            y_values = x_values.copy()
            for i in range(len(self.agent_pos_saver[jj])):
                x_values[i] = self.agent_pos_saver[jj][i][0]
                y_values[i] = self.agent_pos_saver[jj][i][1]

            self.ax.plot(
                x_values,
                y_values,
                color=color,
                linestyle="dashed",
            )

        if len(self.obstacle_colors):
            for jj in range(self.number_agent):
                plot_obstacles(
                    ax=self.ax,
                    obstacle_container=[self.obstacle_environment[jj]],
                    x_lim=self.x_lim,
                    y_lim=self.y_lim,
                    showLabel=False,
                    obstacle_color=self.obstacle_colors[jj],
                    draw_reference=False,
                    set_axes=False,
                )
        else:
            plot_obstacles(
                ax=self.ax,
                obstacle_container=self.obstacle_environment,
                x_lim=self.x_lim,
                y_lim=self.y_lim,
                showLabel=False,
                obstacle_color=np.array([176, 124, 124]) / 255.0,
                draw_reference=False,
                set_axes=False,
            )

        self.ax.set_xlabel("x [m]", fontsize=9)
        self.ax.set_ylabel("y [m]", fontsize=9)

        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(self.x_lim)
        self.ax.set_ylim(self.y_lim)
        plt.tight_layout()

