# Dynamic Window Approach Algorithm
import math
from math import pi, cos, sin
import numpy as np
from Simulator import decode_state, encode_state
from utils import SPD, YAWSPD

c_u1p = 7.9283e-04  # [/kg]
c_u1r = 4.5455e-04  # [/kg]
c_u1l = 4.2284e-04  # [/kg]
c_u2 = 6.3397  # n/a
c_u3 = 3.3827  # [m]
c_u4 = -0.1357  # [/m]
c_v1r = -9.7660e-05  # [kg]
c_v1l = -6.5107e-05  # [kg]
c_v2 = 2.4774  # [/m]
c_v3 = 1.5808  # n/a
c_v4 = -1.9095  # [/m]
c_v5 = 0.0618  # [m]
c_r1r = 1.8303e-04  # [/kg]
c_r1l = 1.2202e-04  # [/kg]
c_r2 = -5.1488  # [/m]
c_r3 = -3.2684  # n/a
c_r4 = 2.2485  # [/m]
c_r5 = -0.2510  # [m]


class DWA(object):
    def __init__(self, config):
        self.config = config
        self._ob_trajectory = []

    def update(self, state, acc_list, goal, ob):
        # Calculate the maximum acceleration according to the current velocity u, angular velocity r
        du_max, dr_max = self.update_accel(state[3], state[4])
        dw = self.calc_dynamic_window(state, du_max, dr_max)
        u, traj = self.calc_final_input(state, acc_list, dw, goal, ob)
        return u, traj

    def update_accel(self, accel, r_accel):
        u0 = accel
        v0 = 0  # Assume that the ship's lateral velocity is always 0
        r0 = r_accel
        X_prop = self.config.Xprop_max
        X_l = self.config.Xl_max
        X_r = -X_l
        # It is considered that the acceleration is the highest when the propeller speed is the highest
        du_max = c_u1p * X_prop + c_u1r * X_r + c_u1l * X_l + c_u2 * v0 * r0 + c_u3 * r0 ** 2 + c_u4 * u0 * abs(u0)
        dv = c_v1r * X_r - c_v1l * X_l + c_v2 * abs(u0) * v0 + c_v3 * abs(u0) * r0 + c_v4 * v0 * abs(
            v0) + c_v5 * r0 * abs(r0)

        # It is considered that the angular acceleration is maximum when
        # only one side of the propeller rotates at the highest speed.
        # In order to be close to reality, reduce the maximum speed
        dr_max = c_r1r * X_r + -c_r1l * X_l + c_r2 * u0 * abs(v0) + c_r3 * abs(u0) * r0 + c_r4 * v0 * abs(
            v0) + c_r5 * r0 * abs(r0)
        return du_max, abs(dr_max)

    def calc_final_input(self, x, u, dw, goal, ob):
        xinit = x[:]
        min_cost = 10000
        min_u = u
        min_u[0] = 0.0
        best_traj = [x]
        self._ob_trajectory.clear()
        # evaluate all trajectory with sampled input in dynamic window
        v_step = (dw[1] - dw[0]) / 6
        r_step = (dw[3] - dw[2]) / 12  # 0.1-0.2s
        for v in np.arange(dw[0], dw[1], v_step):
            for y in np.arange(dw[2], dw[3], r_step):
                traj = self.calc_trajectory(xinit, v, y)
                self.update_ob_trajectory(ob, len(traj))
                final_cost = self._get_cost(traj, goal, ob)
                if min_cost >= final_cost:
                    min_cost = final_cost
                    min_u = [v, y]
                    best_traj = traj
        return min_u, best_traj,

    def uniform_accel(self, state, spd_accel, yawspd_accel, dt):
        posx, posy, phi0, u0, r0 = decode_state(state)

        U0 = u0 * cos(phi0)
        V0 = u0 * sin(phi0)

        new_u = u0 + spd_accel * dt
        new_r = r0 + yawspd_accel * dt

        new_phi = phi0 + (new_r + r0) * dt / 2  # Updated heading angle
        new_phi = new_phi % (2 * pi)

        new_U = new_u * cos(new_phi)  # The updated speed, converted to the global coordinate
        new_V = new_u * sin(new_phi)

        new_posx = posx + (U0 + new_U) * dt / 2  # Updated coordinates
        new_posy = posy + (V0 + new_V) * dt / 2

        state = encode_state(new_posx, new_posy, new_phi, new_u, new_r)
        return state

    # motion model
    # x(m), y(m), yaw(rad), v(m/s), yaw spd(rad/s)
    # u[0] v, u[1] yaw spd
    def uniform_spd(self, x, u, dt):
        x[2] += u[1] * dt
        x[0] += u[0] * cos(x[2]) * dt
        x[1] += u[0] * sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x

    def calc_dynamic_window(self, state, du_max, dr_max):
        # Dynamic window from kinematic self.configure
        # admissible velocities

        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yawrate, self.config.max_yawrate]

        # Dynamic window from motion model
        # The dynamic window calculated according to the current speed and acceleration limit
        # in order: minimum speed, maximum speed, minimum angular speed, maximum angular speed
        Vd = [0,
              state[3] + du_max * self.config.dT,
              state[4] - dr_max * self.config.dT,
              state[4] + dr_max * self.config.dT]

        #  [vmin, vmax, yawrate min, yawrate max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    # Trajectory inference function
    # The dynamic window method assumes that the linear and angular velocities
    # of the vehicle remain unchanged in the given planning time domain,
    # Therefore, the local trajectory of the vehicle in the planning time domain can
    # only be a straight line (r = 0) or an arc (r ≠ 0), # Can be determined by (u, r) value
    # But in order to consider maneuverability, a uniform acceleration trajectory to reach the target state is added
    def calc_trajectory(self, state, v, y):
        trajectory = [state.copy()]
        predicted_time = 0
        spd_accel = (v - state[SPD]) / self.config.dT
        yawspd_accel = (y - state[YAWSPD]) / self.config.dT
        while predicted_time <= self.config.predict_time:
            if predicted_time <= self.config.dT:  # Uniform acceleration period
                state = self.uniform_accel(state, spd_accel, yawspd_accel, self.config.dt)
            else:  # uniform velocity period
                state = self.uniform_spd(state, [v, y], self.config.dt)
            trajectory.append(state.copy())  # Record current and all predicted points
            predicted_time += self.config.dt
        return trajectory

    def _get_cost(self, traj, goal, ob):
        to_goal_cost = self.calc_to_goal_cost(traj[-1], goal, self.config)
        speed_cost = self.config.speed_cost_gain * (self.config.max_speed - traj[-1][3])
        ob_cost = self.config.obstacle_cost_gain * self.calc_obstacle_cost(traj, ob)
        final_cost = to_goal_cost + speed_cost + ob_cost
        # print('goal_cost', to_goal_cost, 'speed_cost', speed_cost, 'obstacle_cost', ob_cost)
        return final_cost

    def calc_obstacle_cost(self, traj, ob):

        skip_n = 2  # calculate once every two steps, to accelerate calculation
        minr = float("inf")

        for step in range(0, len(traj), skip_n):
            for ob in self._ob_trajectory[step]:
                r = (traj[step][0] - ob[0]) ** 2 + (traj[step][1] - ob[1]) ** 2
                if r <= self.config.robot_radius_square:
                    return float("Inf")
                minr = min(minr, math.sqrt(r))
        return 1.0 / minr  # OK

    def update_ob_trajectory(self, ob_list, length=0):
        if len(self._ob_trajectory) == 0:
            self._ob_trajectory.append([[ob[0], ob[1]] for ob in ob_list])
        if len(self._ob_trajectory) < length:
            for _ in range(len(self._ob_trajectory), length):
                last_data = self._ob_trajectory[-1].copy()
                new_data = []
                for ob_id, ob in enumerate(ob_list):
                    new_ob = []
                    new_ob.append(last_data[ob_id][0] + self.config.dt * ob[2] * cos(ob[3]))
                    new_ob.append(last_data[ob_id][1] + self.config.dt * ob[2] * sin(ob[3]))
                    new_data.append(new_ob)
                # self._ob_trajectory.append(last_data.copy())
                self._ob_trajectory.append(new_data)

    def calc_to_goal_cost(self, last_point, goal, config):
        # calc to goal cost. It is 2D norm.
        dist = math.sqrt((goal[0] - last_point[0]) ** 2 + (goal[1] - last_point[1]) ** 2)
        cost = config.to_goal_cost_gain * dist
        return cost

