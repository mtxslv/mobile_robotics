import numpy as np

def round_nearest(x, a):
    # https://stackoverflow.com/questions/28425705/python-round-a-float-to-nearest-0-05-or-to-multiple-of-another-float
    """Round to nearest multiple of a.

    Args:
        x (float): number to be rounded
        a (float): base number used for rounding

    Returns:
        float: rounded number
    """
    return round(round(x / a) * a, 2)

def get_cell(position, centersX, centersY):
    """Get cell from position.

    Args:
        position (numpy ndarray): robot's position in simulated cell
        centersX (numpy ndarray): centers of cells in X axis
        centersY (numpy ndarray): centers of cells in Y axis

    Returns:
        cell_x (int):the number of centersX's cell
        cell_y (int):the number of centersY's cell
    """
    # Tamanho da célula
    cell_size = 0.05
    # é necessário arredondar para 2 casas decimais
    centers_x = np.round(centersX,2)
    centers_y = np.round(centersY,2)

    position_x = round_nearest(position[0],cell_size)
    position_y = round_nearest(position[1],cell_size)
    cell_x = np.where(centers_x == position_x)[0][0]
    cell_y = np.where(centers_y == position_y)[0][0]

    return cell_x, cell_y

def cell_to_coord(cell):
    """Get an spatial coordinate (between -2.6 and 2.6) from a cell.

    Args:
        cell (tuple or np.array): the cell's x and y numbers.

    Returns:
        coord_x (int): the x coordinate of the cell
        coord_y (int): the y coordinate of the cell
    """
    coord_x = cell[0]/105*2*2.6-2.6
    coord_y = cell[1]/105*2*2.6-2.6
    return coord_x, coord_y

