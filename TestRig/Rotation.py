import numpy as np
from Configuration import RobotConfig
from Kinematics import inverse_kinematics


def complete_kinematics(r_abductor_foot,yaw,pitch,roll,leg_index,configuration):
    yaw = np.clip(yaw,-configuration.max_yaw_stand,configuration.max_yaw_stand)
    pitch = np.clip(pitch,-configuration.max_pitch,configuration.max_pitch)
    roll = np.clip(roll,-configuration.max_roll,configuration.max_roll)
    (x,y,z) = r_abductor_foot
    o_abd = configuration.abduction_offsets[leg_index]
    w_body = configuration.body_width
    l_body = configuration.body_length

    # ---Calculate yaw---
    # (Convert to radians)
    yaw = np.deg2rad(yaw)

    # Determine global coordinates:
    if leg_index == 0:
        y_global0 = y - 0.5 * w_body - o_abd
        x_global0 = x - 0.5 * l_body

    elif leg_index == 1:
        y_global0 = y + 0.5 * w_body - o_abd
        x_global0 = x - 0.5 * l_body

    elif leg_index == 2:
        y_global0 = y - 0.5 * w_body - o_abd
        x_global0 = x + 0.5 * l_body

    elif leg_index == 3:
        y_global0 = y + 0.5 * w_body - o_abd
        x_global0 = x + 0.5 * l_body

    else:
        raise ValueError(f"Invalid leg_index: {leg_index}")

    # Calculate initial angle:
    theta_initial = np.arctan(y_global0/x_global0)

    # Distance from body center to leg origin
    ############ Maybe change to pythagoras? ##########
    r = y_global0 / np.sin(theta_initial)

    # New angle:
    theta_new = theta_initial + yaw

    # New global coordinates:
    x_global1 = r * np.cos(theta_new)
    y_global1 = r * np.sin(theta_new)

    # New local coordinates:
    if leg_index == 0:
        x1 = x_global1 + 0.5 * l_body
        y1 = y_global1 + 0.5 * w_body + o_abd

    elif leg_index == 1:
        x1 = x_global1 + 0.5 * l_body
        y1 = y_global1 - 0.5 * w_body + o_abd

    elif leg_index == 2:
        x1 = x_global1 - 0.5 * l_body
        y1 = y_global1 + 0.5 * w_body + o_abd

    elif leg_index == 3:
        x1 = x_global1 - 0.5 * l_body
        y1 = y_global1 - 0.5 * w_body + o_abd

    else:
        raise ValueError(f"Invalid leg_index: {leg_index}")

    z1 = z

    # ---Calculate pitch---
    # (Convert to radians)
    pitch = np.deg2rad(pitch)

    if leg_index == 0 or leg_index == 1:
        pitch = - pitch
    else:
        x1 = -x1

    z_global2 = np.sin(pitch)*0.5*l_body
    x_global2 = np.cos(pitch)*0.5*l_body

    z_2ref = z1 - z_global2
    x_2ref = x1 + 0.5*l_body - x_global2

    theta_af_ref = np.arctan(x_2ref/z_2ref)

    d_xz_af = z_2ref/np.cos(theta_af_ref)

    theta_af = theta_af_ref + pitch

    x2 = d_xz_af * np.sin(theta_af)
    z2 = d_xz_af * np.cos(theta_af)
    y2 = y1
    if leg_index == 2 or leg_index == 3:
        x2 = -x2

    # ---Calculate roll---
    # (Convert to radians)
    roll = np.deg2rad(roll)

    if leg_index == 1 or leg_index == 3:
        roll = - roll
        y2 = - y2

    z_global3 = np.sin(roll) * 0.5 * w_body
    y_global3 = np.cos(roll) * 0.5 * w_body

    z_3ref = z2 - z_global3
    y_3ref = y2 + 0.5 * w_body - y_global3

    theta_yz_af_ref = np.arctan(y_3ref / z_3ref)

    d_yz_af = z_3ref / np.cos(theta_yz_af_ref)

    theta_yz_af = theta_yz_af_ref + roll

    y3 = d_yz_af * np.sin(theta_yz_af)
    z3 = d_yz_af * np.cos(theta_yz_af)
    x3 = x2
    if leg_index == 1 or leg_index == 3:
        y3 = -y3

    x_final = x3
    y_final = y3
    z_final = z3
    #thetas = inverse_kinematics([x_final,y_final,z_final], leg_index, configuration)
    return np.array([x_final, y_final, z_final])