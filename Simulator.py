import random
from math import cos, sin, pi, sqrt

# state [x(m), y(m), yaw(rad), v(m/s), yaw spd(rad/s)]
POSX = 0
POSY = 1
YAW = 2
SPD = 3
YAWSPD = 4


def encode_state(*state):
    assert len(state) == 5
    return list(state)


def decode_state(state):
    return state[POSX], state[POSY], state[YAW], state[SPD], state[YAWSPD]


def trimaran_model(state, left, right, dt):
    # Global frame
    X0 = state[POSX]  # x coordinate[m]
    Y0 = state[POSY]  # y coordinate[m]
    U0 = state[SPD] * cos(state[YAW])  # x-axis velocity [m/s]
    V0 = state[SPD] * sin(state[YAW])  # y-axis velocity [m/s]
    phi0 = state[YAW]  # Yaw angel 0~2PI [rad]
    r0 = state[YAWSPD]  # Yaw angle velocity [rad/s]

    left = left / 60  # left propeller rotation speed  [rps]
    right = right / 60  # right propeller rotation speed [rps]

    # Body-fixed frame
    u0 = V0 * sin(phi0) + U0 * cos(phi0)  # longitudinal velocity
    v0 = V0 * cos(phi0) - U0 * sin(phi0)  # transverse velocity

    du = (-6.7 * u0 ** 2 + 15.9 * r0 ** 2 + 0.01205 * (left ** 2 + right ** 2) - 0.0644 * (
            u0 * (left + right) + 0.45 * r0 * (left - right)) + 58 * r0 * v0) / 33.3
    dv = (-29.5 * v0 + 11.8 * r0 - 33.3 * r0 * u0) / 58
    dr = (-0.17 * v0 - 2.74 * r0 - 4.78 * r0 * abs(r0) + 0.45 * (
            0.01205 * (left ** 2 - right ** 2) - 0.0644 * (
            u0 * (left - right) + 0.45 * r0 * (left + right)))) / 6.1

    u = u0 + du * dt
    v = v0 + dv * dt

    r = r0 + dr * dt  # updated Yaw angle velocity
    phi = phi0 + (r + r0) * dt / 2  # updated Yaw angle
    phi = phi % (2 * pi)
    U = u * cos(phi) - v * sin(phi)  # updated x-axis velocity, Global frame
    V = u * sin(phi) + v * cos(phi)
    state[POSX] = X0 + (U0 + U) * dt / 2  # updated coordinate, Global frame
    state[POSY] = Y0 + (V0 + V) * dt / 2
    state[YAW] = phi
    state[SPD] = sqrt(U ** 2 + V ** 2)
    state[YAWSPD] = r
    return state


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


def AUV_model(state, X_prop, X_l, dt):
    X_r = -X_l
    # Global frame
    X0 = state[POSX]  # x coordinate[m]
    Y0 = state[POSY]  # y coordinate[m]
    U0 = state[SPD] * cos(state[YAW])  # x-axis velocity [m/s]
    V0 = state[SPD] * sin(state[YAW])  # y-axis velocity [m/s]
    phi0 = state[YAW]  # Yaw angel 0~2PI [rad]
    r0 = state[YAWSPD]  # Yaw angle velocity [rad/s]

    # Body-fixed frame
    u0 = V0 * sin(phi0) + U0 * cos(phi0)  # longitudinal velocity
    v0 = V0 * cos(phi0) - U0 * sin(phi0)  # transverse velocity

    du = c_u1p * X_prop + c_u1r * X_r + c_u1l * X_l + c_u2 * v0 * r0 + c_u3 * r0 ** 2 + c_u4 * u0 * abs(u0)
    dv = c_v1r * X_r - c_v1l * X_l + c_v2 * abs(u0) * v0 + c_v3 * abs(u0) * r0 + c_v4 * v0 * abs(v0) + c_v5 * r0 * abs(r0)
    dr = c_r1r * X_r + -c_r1l * X_l + c_r2 * u0 * abs(v0) + c_r3 * abs(u0) * r0 + c_r4 * v0 * abs(v0) + c_r5 * r0 * abs(r0)

    u = u0 + du * dt
    v = v0 + dv * dt

    r = r0 + dr * dt  # updated Yaw angle velocity
    phi = phi0 + (r + r0) * dt / 2  # updated Yaw angle
    phi = phi % (2 * pi)
    U = u * cos(phi) - v * sin(phi)  # updated x-axis velocity, Global frame
    V = u * sin(phi) + v * cos(phi)
    state[POSX] = X0 + (U0 + U) * dt / 2  # updated coordinate, Global frame
    state[POSY] = Y0 + (V0 + V) * dt / 2
    state[YAW] = phi
    state[SPD] = sqrt(U ** 2 + V ** 2)
    state[YAWSPD] = r
    return state


def apply_noise(state):
    noise = [random.gauss(0, 0.01), random.gauss(0, 0.01),  # POSX, POSY
             random.gauss(0, 0.01), random.gauss(0, 0.01),  # Yaw (rad), Speed (m/s)
             random.gauss(0, 0.005)]
    for i in range(5):
        state[i] += noise[i]
    return state


def update_obstacle(obstacles, dt):
    new_obstacles = obstacles.copy()
    for i in range(len(obstacles)):
        new_obstacles[i][0] = obstacles[i][0] + dt * obstacles[i][2] * cos(obstacles[i][3])
        new_obstacles[i][1] = obstacles[i][1] + dt * obstacles[i][2] * sin(obstacles[i][3])
        new_obstacles[i][0] = max(min(new_obstacles[i][0], 100), 0)
        new_obstacles[i][1] = max(min(new_obstacles[i][1], 100), 0)
    return new_obstacles
