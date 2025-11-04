import numpy as np

def inverse_kinematics(r_abductor_foot,leg_index,configuration):
    """
    :param r_abductor_foot: (x,y,z) coordinates of foot relative to leg origin at the abductor
            x: Positive toward front
            y: Positive toward left
            z: Positive vertically upward
    :param leg_index: index of leg
    :param configuration:
    :return:
        numpy array (3)
            Array of joints at abductor(relative to y), hip(relative to -z'), knee (relative to -z')
    """
    (x,y,z) = r_abductor_foot
    o_abd = configuration.abduction_offsets[leg_index]
    l_up = configuration.leg_up
    l_low = configuration.leg_low

    # Determine abductor angle:
    # Angle between direction of d_yz_af from Abductor to Foot in yz-plane and positive y-axis:
    alpha = np.arctan2(z,y)

    # Length of D_yz_AF
    d_yz_af = (y**2+z**2)**0.5

    # Angle between abductor offset o_abd and d_yz_af
    arccos_argument = o_abd/d_yz_af
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    beta = np.arccos(arccos_argument)

    # The abductor angle from y-axis to o_abd is the sum of alpha and beta
    theta_abductor = alpha+beta

    # Determine hip angle:
    # Distance d_yz_hf from Hip to Foot in yz-plane
    d_yz_hf = (d_yz_af**2-o_abd**2)**0.5

    # The rest of the calculations are done in a tilted xz1-plane with normal vector along o_abd
    # The length d_yz_hf in the xz1-plane is the distance along the tilted z1 axis between Hip and Foot.
    # Angle between the tilted negative z1 axis and direction of d_xz1_hf from Hip to Foot in xz1-plane
    gamma = np.arctan2(x,d_yz_hf)

    # Distance d_xz1_hf from Hip to Foot in xz1-plane
    d_xz1_hf = (d_yz_hf**2+x**2)**0.5

    #Angle between upper leg l_up and d_xz1_hf with cosine relation
    arccos_argument = (l_up**2 + d_xz1_hf**2 - l_low**2) / (2*l_up*d_xz1_hf)
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    psi = np.arccos(arccos_argument)

    # The hip angle from negative z1 to upper leg is the difference between psi and gamma
    theta_hip = psi - gamma # Positive angle from -z' toward -x

    # Determine knee angle:
    # Angle from upper leg to lower leg
    arccos_argument = (l_up ** 2 + l_low ** 2 - d_xz1_hf ** 2) / (2 * l_up * l_low)
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    phi = np.arccos(arccos_argument)

    # The knee angle from negative z1 to lower leg is:
    theta_knee = theta_hip + phi - np.pi

    return np.array([theta_abductor, theta_hip, theta_knee])

def four_inverse_kinematics(r_body_foot,configuration):
    alpha = np.zeros(3,4)

    return alpha