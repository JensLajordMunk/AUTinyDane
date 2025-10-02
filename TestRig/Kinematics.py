import numpy as np
from transforms3d.euler import euler2mat

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
    O_abd = configuration.abduction_offsets[leg_index]
    L_up = configuration.leg_up
    L_low = configuration.leg_low

    # Determine abductor angle:
    # Angle between direction of D_yz_AF from Abductor to Foot in yz-plane and positive y-axis:
    alpha = np.arctan2(z,y)

    # Length of D_yz_AF
    D_yz_AF = (y**2+z**2)**0.5 # TODO: Change variable name to respect lowercase variable convention if possible

    # Angle between abductor offset O_abd and D_yz_AF
    arccos_argument = O_abd/D_yz_AF
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99) #TODO: Determine clip values
    beta = np.arccos(arccos_argument)

    # The abductor angle from y-axis to O_abd is the sum of alpha and beta
    theta_abductor = alpha+beta

    # Determine hip angle:
    # Distance D_yz_HF from Hip to Foot in yz-plane
    D_yz_HF = (D_yz_AF**2-O_abd**2)**0.5 # TODO: Change variable name to respect lowercase variable convention if possible

    # The rest of the calculations are done in a tilted xz1-plane with normal vector along O_abd
    # The length D_yz_HF in the xz1-plane is the distance along the tilted z1 axis between Hip and Foot.
    # Angle between the tilted negative z1 axis and direction of D_xz1_HF from Hip to Foot in xz1-plane
    gamma = np.arctan2(x/D_yz_HF)

    # Distance D_xz1_HF from Hip to Foot in xz1-plane
    D_xz1_HF = (D_yz_HF**2+x**2)**0.5 # TODO: Change variable name to respect lowercase variable convention if possible

    #Angle between upper leg L_up and D_xz1_HF with cosine relation
    arccos_argument = (L_up**2 + D_xz1_HF**2 - L_low**2) / (2*L_up*D_xz1_HF)
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99) #TODO: Determine clip values
    psi = np.arccos(arccos_argument)

    # The hip angle from negative z1 to upper leg is the difference between phi and gamma
    theta_hip = psi - gamma

    # Determine knee angle:
    # Angle from upper leg to lower leg
    arccos_argument = (L_up ** 2 + L_low ** 2 - D_xz1_HF ** 2) / (2 * L_up * L_low)
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)  #TODO: Determine clip values
    phi = np.arccos(arccos_argument)

    # The knee angle from negative z1 to lower leg is:
    theta_knee = theta_hip + phi - np.pi

    return np.array([theta_abductor, theta_hip, theta_knee])

def four_inverse_kinematics(r_body_foot,configuration):
    alpha = np.zeros(3,4)

    return alpha