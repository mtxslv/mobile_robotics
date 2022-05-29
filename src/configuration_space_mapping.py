def get_normals_orientation(euler_z_angle, polygon_type = 'cubic'):
    """This method returns the normals on a polygon's horizontal faces, considering one of the faces is aligned with the X axis.

    Args:
        euler_z_angle (float): the angle between the polygon's X axis and the global X axis.
        polygon_type (str, optional): the type of polygon to be considered. Defaults to 'cubic'.

    Raises:
        ValueError: raised when an unsupported polygon is requested
        ValueError: raised when the Z euler angle is not within [-2pi, 2pi]

    Returns:
        tuple: x_angle, y_angle, negative_x_angle, and negative_y_angle. 
    """
    if polygon_type != 'cubic':
        raise ValueError("only cubic polygons supported so far")
    
    pi = 3.14159265359
    if euler_z_angle > 2*pi or euler_z_angle < -2*pi:
        raise ValueError("Euler angle is out of limits")
    else:
        x_angle = euler_z_angle
        y_angle = euler_z_angle + pi/2
        negative_x_angle = euler_z_angle + pi
        negative_y_angle = euler_z_angle + pi/2 + pi

    return x_angle, y_angle, negative_x_angle, negative_y_angle

