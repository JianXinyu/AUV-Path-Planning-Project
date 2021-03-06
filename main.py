import time
from math import atan2, pi
import numpy as np

from msgdev import PeriodTimer, MsgDevice
from PID import PID, PID_angle
from Simulator import apply_noise, update_obstacle, AUV_model
from utils import plot, POSX, POSY, plot2
from RRTstar import RRTStar
from DWA import DWA



# Visulization
show_animation = True
show_result = True

info_format = \
"Step {i} Info:\n\
SPD:\t real {real_spd:.2f} | target {target_spd:.2f}\n\
YAW:\t real {real_yaw:.2f} | target {target_yaw:.2f}\n\
YAWSPD:\t real {real_yawspd:.2f} | target {target_yawspd:.2f}\n\
Average Output:\t\t{average_output:.2f}\n\
Diff Output:\t{output_diff:.2f}\n\
Calc Time:\t {calc_time:.4f}\n\
==================="


class Interface_rec(object):
    def __init__(self, sub_addr, pose_port):
        self.dev = MsgDevice()
        self.dev.open()
        self.dev.sub_connect(sub_addr + ':' + pose_port)
        self.dev.sub_add_url('posx')
        self.dev.sub_add_url('posy')
        self.dev.sub_add_url('speed')
        self.dev.sub_add_url('speed_tar')
        self.dev.sub_add_url('yaw')
        self.dev.sub_add_url('yaw_tar')
        self.dev.sub_add_url('yawspd')
        self.dev.sub_add_url('yawspd_tar')

    def receive(self, *args):
        data = []
        for i in args:
            data.append(self.dev.sub_get1(i))
        return data

def auv_initialize():
    local_addr = 'tcp://127.0.0.1'
    self_port = '55204'
    auv = Interface_rec(local_addr, self_port)
    return auv


# simulation parameters
class ConfigDWA:
    def __init__(self):
        # robot parameter
        self.max_speed = 1.5  # maximal sailing velocity[m/s]
        self.min_speed = 0  # maximal astern velocity[m/s]
        self.max_yawrate = 0.1  # maximal yaw angle velocity[rad/s]
        self.v_reso = 0.1  # [m/s]
        self.yawrate_reso = pi / 180.0  # [rad/s]

        self.dT = 1.0  # dynamic window time[s]
        self.dt = 0.1  # Trajectory inference time step[s]
        self.predict_time = 10.0  # Trajectory inference total time[s]

        self.to_goal_cost_gain = 0.01  # 0.01
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 1.5
        self.robot_radius = 3  # [m]
        self.robot_radius_square = self.robot_radius ** 2  # [m]

        self.Xprop_max = 500
        self.Xl_max = 1500


def pure_pursuit(self, target):
    dx = target[POSX] - self[POSX]
    dy = target[POSY] - self[POSY]
    target_angle = atan2(dy, dx) % (2 * pi)
    dist = (dx ** 2 + dy ** 2) ** .5
    return target_angle, dist


def get_nearest_path_index(path, state):
    dlist = [(point[0] - state[0]) ** 2 + (point[1] - state[1])
             ** 2 for point in path]
    minind = dlist.index(min(dlist))

    return minind


def main():
    # auv = auv_initialize()
    auv = MsgDevice()
    auv.open()
    auv.pub_bind('tcp://0.0.0.0:55004')

    # Pure Pursuit initialize
    target_angle = 0
    dist = 0
    # DWA initialize
    configDWA = ConfigDWA()
    dynamic_window = DWA(configDWA)
    # initial state [x(m), y(m), yaw(rad), speed(m/s), yaw_speed(rad/s)]
    # init_state = np.array([0, 10, 45 * pi / 180, 0.0, 0.0])
    init_state = np.array([0, 0, 0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    predefined_goal = np.array([90.0, 80.0])
    goal = predefined_goal
    # obstacles [x(m) y(m), spd(m/s), yaw]
    # static_obstacles = np.array([[100.0, 70.0, 0.0, 0.0],
    #                [30.0, 39.0, 0, 0.0],
    #                [30.0, 70.0, 0.0, 0.0],
    #                [50.0, 70.0, 0.0, 0.0],
    #                [80, 70, 0, 0],
    #                [60, 40, 0, 0],
    #                [40, 40, 0, 0],
    #                [70, 80, 0, 0],
    #                [20.0, 20.0, 0.0, 0.0],
    #                ])
    static_obstacles = np.array([[90.0, 60.0, 0.0, 0.0],
                   [60.0, 70.0, 0, 0.0],
                   [80.0, 60.0, 0.0, 0.0],
                   [50.0, 70.0, 0.0, 0.0],
                   [80, 70, 0, 0],
                   [60, 40, 0, 0],
                   [40, 50, 0, 0],
                   [30, 60, 0, 0],
                   [20.0, 30.0, 0.0, 0.0],
                   ])
    moving_obstacles = np.array([[20, 10, 0.7, 2],
                                 [60, 30, 0.2, 2]
                                 ])
    u = np.array([0.0, 0.0])
    # RRT* initialize
    rrt_star = RRTStar(start=[init_state[0], init_state[1]],
                       goal=goal,
                       rand_area=[0, 100],
                       obstacle_list=static_obstacles,
                       expand_dis=5.0,
                       path_resolution=1.0,
                       goal_sample_rate=10,
                       max_iter=100,
                       connect_circle_dist=50.0)
    # path = rrt_star.planning(animation=True, search_until_max_iter=True)
    # path_index = -1
    # localgoal = path[path_index]
    # time.sleep(3)
    traj = [init_state]
    best_traj = None
    state = init_state

    interval = 0.1  # Simulation time step
    pid_spd = PID(kp=2500.0, ki=100.0, kd=0, minout=-1000, maxout=1000, sampleTime=interval)
    pid_yawspd = PID(kp=15000, ki=500.0, kd=10, minout=-1500, maxout=1500, sampleTime=interval)
    pid_yaw = PID_angle(kp=800, ki=3, kd=10, minout=-1500, maxout=1500, sampleTime=interval)
    t = PeriodTimer(interval)
    t.start()
    i = 0

    try:
        while True:
            with t:
                start_time = time.perf_counter()
                if i < 10000:
                    i += 1
                else:
                    i = 0

                goal = predefined_goal
                obstacle = np.vstack((moving_obstacles, static_obstacles))
                min_dist = 100 ** 2
                for ob_single in moving_obstacles:
                    dist = (state[0] - ob_single[0]) ** 2 + (state[1] - ob_single[1]) ** 2
                    if dist < min_dist:
                        min_dist = dist

                # to_localgoal_dist_square = (state[POSX] - localgoal[0]) ** 2 + (state[POSY] - localgoal[1]) ** 2
                # if to_localgoal_dist_square <= configDWA.robot_radius_square:
                #     path_index -= 1
                # if abs(path_index) < len(path):
                #     localgoal = path[path_index]
                # else:
                #     localgoal = goal

                # if min_dist > 8 ** 2:
                if 0:
                    best_traj =None
                    target_angle, dist = pure_pursuit(state, localgoal)
                    if target_angle > pi:
                        target_angle -= 2 * pi
                    elif target_angle < -pi:
                        target_angle += 2 * pi
                    output = pid_yaw.compute(state[2], target_angle)  # yaw, target_angle unit: rad
                    # dead band
                    diff = 0 if abs(output) < 5 else output
                    # use different speed based on distance
                    if dist <= 3:
                        average = 800
                    elif dist <= 10:
                        average = 1000
                    else:
                        average = 1200

                    print('Pure Pursuit, Current path point: ', -path_index, "/", len(path))
                else:
                    u, best_traj = dynamic_window.update(state, u, goal, obstacle)
                    u = [1, 0.1]
                    # Forward velocity
                    average = pid_spd.compute(state[3], u[0])
                    average = 0 if abs(average) < 5 else average
                    # Yaw angle velocity
                    diff = pid_yawspd.compute(state[4], u[1])
                    diff = 0 if abs(diff) < 5 else diff
                    print('Dynamic Window Approach')
                    # which path point to follow when switch to the pure pursuit
                    # path_index = get_nearest_path_index(path, state)-len(path)
                    if min_dist < 3 ** 2:
                        print('Collision!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')


                print(info_format.format(
                    i=i,
                    real_spd=state[3],
                    target_spd=u[0],
                    real_yaw=state[2],
                    target_yaw=target_angle,
                    real_yawspd=state[4],
                    target_yawspd=u[1],
                    average_output=average,
                    output_diff=diff,
                    calc_time=time.perf_counter() - start_time
                ))

                if show_animation:
                    traj.append(state.copy())
                    # plot(best_traj, state, goal, obstacle, path, traj)
                    plot2(best_traj, state, goal, obstacle, traj)

                to_goal_dist_square = (state[POSX] - goal[0]) ** 2 + (state[POSY] - goal[1]) ** 2
                if to_goal_dist_square <= configDWA.robot_radius_square:
                    time.sleep(3)
                    print("Goal!!")
                    break

                # Simulation
                # state = trimaran_model(state, left, right, interval)
                state = AUV_model(state, average, -diff, interval)
                state = apply_noise(state)
                moving_obstacles = update_obstacle(moving_obstacles, interval)

                auv.pub_set1('posx', state[0])
                auv.pub_set1('posy', state[1])
                auv.pub_set1('speed', state[3])
                auv.pub_set1('yaw', state[2])
                auv.pub_set1('yawspd', state[4])
                auv.pub_set1('speed_tar', u[0])
                auv.pub_set1('yaw_tar', target_angle)
                auv.pub_set1('yawspd_tar', u[1])

    finally:
        time.sleep(interval)
        print('everything closed')

from collections import deque

POSX = 0
POSY = 1
YAW = 2
SPD = 3
YAWSPD = 4

if __name__ == "__main__":

    print(__file__ + " start!!")
    main()

    # # Calculate maximum acceleration and velocity based on mathematical model
    # init_state = [0, 0, 0, 0, 0]
    # x = init_state
    # X_prop = 500
    # X_l = 2000
    # cnt = 0
    # flag = 0
    # spd_rec = deque(maxlen=2)
    # yawspd_rec = deque(maxlen=2)
    # spd_rec.append(0)
    # yawspd_rec.append(0)
    # interval = 0.001
    # calc_interval = 0.1  # (s)
    # t = PeriodTimer(interval)
    # t.start()
    # while True:
    #     with t:
    #         # x = trimaran_model(x, left, right, interval)
    #         x = AUV_model(x, X_prop, X_l, interval)
    #         cnt += 1
    #         if cnt == calc_interval / interval:
    #             cnt = 0
    #             speed = x[SPD]
    #             spd_rec.append(speed)
    #             yawspd_rec.append(x[YAWSPD])
    #             acc = (spd_rec[1] - spd_rec[0]) / calc_interval
    #             yaw_acc = (yawspd_rec[1] - yawspd_rec[0]) / calc_interval
    #             print('speed', speed, 'accelerate', acc, 'yaw speed', x[YAWSPD], 'yaw_accelerate', yaw_acc)
