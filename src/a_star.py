from sympy import Polygon, Line, Point
import numpy as np

def a_star(robo_cell, goal_cell, graph_connections, g_N, h_N, cells_poly):
    """A* algorithm for minimum cost path retrieval.

    Args:
        robo_cell (int): the number of the cell where the robot is
        goal_cell (int): the number of the cell where the goal is
        graph_connections (dict): the connectivity graph, as a dict of lists
        g_N (dict): arcs' weights values, as a dict of lists (following the same structure as graph_connections)
        h_N (dict): heuristics' weights values, as a dict of values (following the same structure as graph_connections)
        cells_poly (list): the list of polygons, as a list of Polygon objects
    Returns:
        path (list): the path's cells sequence
    """

    path = [robo_cell]
    # we define that the current_node is the cell where the robot is
    current_node = robo_cell
    following_node = current_node
    iterations = 0

    while(following_node!=goal_cell and iterations < len(cells_poly)):
        # we now compute the cost function on the current node
        g_N_current = g_N[current_node]
        h_N_current = []
        for next_node in graph_connections[current_node]:
            h_N_current.append(h_N[next_node])
        f_N_current = np.array(g_N_current) + np.array(h_N_current)

        # we now find out where f[N] is minimal
        where_f_is_minimal = f_N_current.argmin()

        # maybe the next node is already in path. Thus it is necessary to find the real next one
        following_node = graph_connections[current_node][where_f_is_minimal]
        while following_node in path:
            f_N_current_copy = f_N_current.copy()
            next_conections = graph_connections[current_node].copy()
            f_N_current_copy = np.delete(f_N_current_copy, where_f_is_minimal)
            del next_conections[where_f_is_minimal]
            where_f_is_minimal = f_N_current_copy.argmin()
            following_node = next_conections[where_f_is_minimal]
        
        path.append(following_node)
        current_node = following_node
        iterations += 1
    if path[-1] != goal_cell:
        print('no path found')
        return None
    else:
        print('path found')
        return path