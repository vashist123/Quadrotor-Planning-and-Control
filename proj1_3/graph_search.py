from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from .occupancy_map import OccupancyMap  # Recommended.
from .Node import Node


def pop_from_heap(prior_q):
    node = heappop(prior_q)
    flag = True
    while(prior_q or flag is True):
        if node.is_closed is True:
            node = heappop(prior_q)
        elif node.is_closed is False:
            return node
        flag = False

def get_path(node,goal,start,start_node,occ_map):
    path = []
    path.append(np.asarray(goal))
    while node.index_coord != start_node.index_coord:
        path.append(np.asarray(occ_map.index_to_metric_center(node.index_coord)))
        node = node.parent
    path.append(np.asarray(start))
    path.reverse()
    path = np.array(path).reshape(-1,3)

    return path

def get_norm(start,goal,resolution):

    norm = np.sqrt(((goal[0]-start[0])/resolution[0])**2 + ((goal[1]-start[1])/resolution[1])**2 + ((goal[2]-start[2])/resolution[2])**2)
    return norm


def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    nodes_expanded = 0

    start_node = Node(start_index)
    goal_node = Node(goal_index)
    start_node.g = 0
    start_node.h = get_norm(np.asarray(start_node.index_coord),np.asarray(goal_node.index_coord),resolution)

    open = []
    closed = []
    closed_dict = {}
    heappush(open, start_node)
    open_dict = {start_node.index_coord: start_node}

    while closed_dict.get(goal_node.index_coord) is None and len(open) != 0:

        curr_node = pop_from_heap(open)
        closed_dict[curr_node.index_coord] = curr_node
        heappush(closed, curr_node)

        actions = [-1, 0, 1]
        curr_x, curr_y, curr_z = curr_node.index_coord[0], curr_node.index_coord[1], curr_node.index_coord[2]

        for i in actions:
            for j in actions:
                for k in actions:
                    if i == j == k == 0:
                        continue
                    else:
                        new_node_coord = (curr_x + i, curr_y + j, curr_z + k)
                        if occ_map.is_valid_index(new_node_coord) and not occ_map.is_occupied_index(new_node_coord):
                            new_node = Node(new_node_coord)
                            if new_node.index_coord == goal_node.index_coord:
                                goal_node.parent = curr_node
                                closed_dict[goal_node.index_coord] = goal_node
                            if open_dict.get(new_node.index_coord) is None and closed_dict.get(new_node.index_coord) is None:
                                nodes_expanded += 1
                                new_node.g = curr_node.g + get_norm(np.asarray(curr_node.index_coord),np.asarray(new_node.index_coord),resolution)
                                if astar:
                                    new_node.h = get_norm(np.asarray(new_node.index_coord),np.asarray(goal_node.index_coord),resolution)
                                else:
                                    new_node.h = 0
                                new_node.f = new_node.g + new_node.h
                                new_node.parent = curr_node
                                heappush(open, new_node)
                                open_dict[new_node.index_coord] = new_node
                            elif open_dict.get(new_node.index_coord) is not None:
                                existing_node = open_dict.get(new_node.index_coord)
                                new_g = curr_node.g + get_norm(np.asarray(curr_node.index_coord),np.asarray(existing_node.index_coord),resolution)
                                new_f = new_g + existing_node.h
                                if new_f < existing_node.f:
                                    existing_node.is_closed = True
                                    new_node_dc = Node(new_node.index_coord)
                                    new_node_dc.g = new_g
                                    new_node_dc.f = new_f
                                    new_node_dc.h = existing_node.h
                                    new_node_dc.parent = curr_node
                                    open_dict[new_node.index_coord] = new_node_dc
                                    heappush(open, new_node_dc)

    if closed_dict.get(goal_node.index_coord) is not None:
        path = get_path(goal_node,goal,start,start_node,occ_map)
        return (path,nodes_expanded)

    # Return a tuple (path, nodes_expanded)
    return None, 0
