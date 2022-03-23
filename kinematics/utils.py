def drive_arc_convert(velocity, radius, time):
    """
    Converts a drive arc message into body velocities to send to kinematics
    """
    psi_dot = velocity / (radius * time)
    x_dot = velocity / time
    return psi_dot, x_dot
