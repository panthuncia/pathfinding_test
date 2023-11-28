

import numpy as np
import matplotlib.pyplot as plt
import random
import math
import scipy.ndimage as nd 
from collections import defaultdict 
from PriorityQueue import PriorityQueue
from typing import List

TURN_WEIGHT = 100

# class Node:
#     x: int
#     y: int
#     neighbors: List[__class__]
#     def __init__(self, x: int, y: int):
#         self.x = x
#         self.y = y

def create_blob(grid, start_x, start_y, blob_size):
    directions = [(1,0), (0,1), (-1,0), (0,-1)]
    for _ in range(blob_size):
        grid[start_y][start_x] = 1
        dx, dy = random.choice(directions)
        start_x = max(0, min(grid.shape[1] - 1, start_x + dx))
        start_y = max(0, min(grid.shape[0] - 1, start_y + dy))

def generate_grid(width, height, num_obstacles, max_blob_size):
    grid = np.zeros((height, width))

    for _ in range(num_obstacles):
        blob_start_x = random.randint(0, width-1)
        blob_start_y = random.randint(0, height-1)
        blob_size = random.randint(1, max_blob_size)
        
        create_blob(grid, blob_start_x, blob_start_y, blob_size)

    return grid

def raycast(x1, y1, x2, y2):
    #supercover line algorithm
    #adapted from http://eugen.dedu.free.fr/projects/bresenham/
    x_list = []
    y_list = []
    i =0           # loop counter
    ystep=0
    xstep=0    # the step on y and x axis
    error=0           # the error accumulated during the increment
    errorprev=0       # *vision the previous value of the error variable
    y = y1 
    x = x1  # the line points
    ddy=0 
    ddx=0        # compulsory variables: the double values of dy and dx
    dx = x2 - x1
    dy = y2 - y1
    #POINT (y1, x1);  # first point
    x_list.append(x1)
    y_list.append(y1)
    # NB the last point can't be here, because of its previous point (which has to be verified)
    if (dy < 0):
        ystep = -1
        dy = -dy
    else:
        ystep = 1
    if (dx < 0):
        xstep = -1
        dx = -dx
    else:
        xstep = 1
    ddy = 2 * dy
    ddx = 2 * dx
    if (ddx >= ddy):  # first octant (0 <= slope <= 1)
        # compulsory initialization (even for errorprev, needed when dx==dy)
        errorprev = error = dx  # start in the middle of the square
        for i in range(dx):  # do not use the first point (already done)
            x += xstep
            error += ddy
            if (error > ddx):  # increment y if AFTER the middle ( > )
                y += ystep
                error -= ddx
                # three cases (octant == right->right-top for directions below):
                if (error + errorprev < ddx):  # bottom square also
                    #POINT (y-ystep, x)
                    x_list.append(x)
                    y_list.append(y-ystep)
                elif (error + errorprev > ddx):  # left square also
                    #POINT (y, x-xstep)
                    x_list.append(x-xstep)
                    y_list.append(y)
                else:  # corner: bottom and left squares also
                    #POINT (y-ystep, x)
                    x_list.append(x)
                    y_list.append(y-ystep)
                    # #POINT (y, x-xstep)
                    x_list.append(x-xstep)
                    y_list.append(y)
                    pass
            
            #POINT (y, x)
            x_list.append(x)
            y_list.append(y)
            errorprev = error
        
    else:  # the same as above
        errorprev = error = dy
        for i in range(dy):
            y += ystep
            error += ddx
            if (error > ddy):
                x += xstep
                error -= ddy
                if (error + errorprev < ddy):
                    #POINT (y, x-xstep)
                    x_list.append(x-xstep)
                    y_list.append(y)
                elif (error + errorprev > ddy):
                    #POINT (y-ystep, x)
                    x_list.append(x)
                    y_list.append(y-ystep)
                else:
                    #POINT (y, x-xstep)
                    x_list.append(x-xstep)
                    y_list.append(y)
                    # #POINT (y-ystep, x)
                    x_list.append(x)
                    y_list.append(y-ystep)
                    pass
            
            #POINT (y, x)
            x_list.append(x)
            y_list.append(y)
            errorprev = error
    # assert ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the same with the last point of the algorithm
    return x_list, y_list

def hasLOS(grid, a, b):
    """
    uses raycasting to calculate LOS between nodes on an OccupancyGrid
    :param grid The occupancy grid
    :param a [(x,y)] the first point
    :param b [(x, y)] the second point
    :return     [bool] has LOS
    """
    #+0.5 on each direction to account for center-cell offset
    # x, y = PathPlanner.dda(a, b)
    x, y = raycast(a[0], a[1], b[0], b[1])
    for i in range(len(x)):
        if(grid[y[i]][x[i]]>0.5):
            print("conflict on "+str((x[i], y[i])))
            return (x[i], y[i])
    return None

def distance(x1, y1, x2, y2):
    x2x1 = x2-x1
    y2y1 = y2-y1
    return math.sqrt(x2x1*x2x1 + y2y1*y2y1)

def is_cell_walkable(grid, h, w, x, y):
    if(x>=0 and y>=0 and x<w and y<h and grid[y][x]<=0.5):
        return True
    return False

def neighbors_of_7(mapdata, h, w, x, y):
    neighbors = []
    if(is_cell_walkable(mapdata, h, w, x, y-1)):
        neighbors.append((x, y-1))
    if(is_cell_walkable(mapdata, h, w, x-1, y)):
        neighbors.append((x-1, y))
    if(is_cell_walkable(mapdata, h, w, x+1, y)):
        neighbors.append((x+1, y))
    # if(is_cell_walkable(mapdata, h, w, x, y+1)):
    #     neighbors.append((x, y+1))
    if(is_cell_walkable(mapdata, h, w, x-1, y-1)):
        neighbors.append((x-1, y-1))
    if(is_cell_walkable(mapdata, h, w, x+1, y-1)):
        neighbors.append((x+1, y-1))
    if(is_cell_walkable(mapdata, h, w, x-1, y+1)):
        neighbors.append((x-1, y+1))
    if(is_cell_walkable(mapdata, h, w, x+1, y+1)):
        neighbors.append((x+1, y+1))
    return neighbors

# def neighbors_of_3(mapdata, h, w, x, y):
#     neighbors = []
#     if(is_cell_walkable(mapdata, h, w, x, y-1)):
#         neighbors.append((x, y-1))
#     if(is_cell_walkable(mapdata, h, w, x-1, y)):
#         neighbors.append((x-1, y))
#     if(is_cell_walkable(mapdata, h, w, x+1, y)):
#         neighbors.append((x+1, y))
#     if(is_cell_walkable(mapdata, h, w, x, y+1)):
#         neighbors.append((x, y+1))
#     return neighbors

def reconstruct_path(end, cameFrom: dict):
    path = []
    path.append(end)
    while end in cameFrom:
        end = cameFrom.get(end)
        path.insert(0, end)
    return path

def turn_penalty(previous, current, next):
    if current[0] - previous[0] != 0 and next[0] - current[0] != 0:
        slope1 = (current[1] - previous[1]) / (current[0] - previous[0])
        slope2 = (next[1] - current[1]) / (next[0] - current[0])
        if math.isclose(slope1, slope2):
            return 0
        else:
            return TURN_WEIGHT
    else:
        # If the x-coordinates are the same, the points are collinear if
        # they all have the same x-coordinate
        if previous[0] == current[0] == next[0]:
            return 0
        elif previous[1] == current[1] == next[1]:
            return 0
        else:
            return TURN_WEIGHT

def a_star(mapdata, height, width, start, goal, visualize=False):
    ### REQUIRED CREDIT
    print("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from the start to n currently known.
    cameFrom = {}
    #map with default value of Infinity
    gScore = defaultdict(lambda: float('inf'))
    #map with default value of Infinity
    fScore = defaultdict(lambda: float('inf'))
    openSet = PriorityQueue()
    open = set()
    closed = set()
    closed_list = []
    current = start
    gScore[start]=0
    start_f = distance(current[0], current[1], goal[0], goal[1])
    fScore[start]= start_f
    #cursed
    if(current == goal):
        return reconstruct_path(goal, cameFrom)
    else:
        openSet.put(current, start_f)
        open.add(current)
    while current != goal and not openSet.empty():
        # if visualize:
        #     self.showAstarPoints(mapdata, closed_list, openSet.elements)
        current = openSet.get()
        open.remove(current)
        neighbors = neighbors_of_7(mapdata, height, width, current[0], current[1])
        for node in neighbors:
            if node in closed:
                continue
            current_turn_penalty = 0
            if current in cameFrom:
                current_turn_penalty = turn_penalty(cameFrom[current], current, node)
            tentative_gscore = gScore[current]+distance(node[0], node[1], current[0], current[1])+current_turn_penalty
            if(tentative_gscore<gScore[node]):
                cameFrom[node] = current
                gScore[node] = tentative_gscore
                #currently just uses distance as a heuristic, which is redundant. Code exists anyway, in case we want a more complex cost-avoidance system
                f = tentative_gscore+distance(node[0], node[1], goal[0], goal[1])
                fScore[node] = f
                if node in open:
                    openSet.remove(node)
                open.add(node)
                openSet.put(node, f)
                closed.add(node)
    if(openSet.empty()):
        print("No path found")
        return []
    return reconstruct_path(goal, cameFrom)

def display_grids(original, rotated, wind_angle_degrees, p1, p1_trans, p2, p2_trans, path_points, path_points_trans, conflict: bool, conflict_pos):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    first_color='green'
    second_color = 'green'
    if(conflict):
        first_color = 'red'

    path_points.append(p1)
    path_points.append(p2)

    if not conflict:
        path_points_trans.append(p1_trans)
        path_points_trans.append(p2_trans)

    axes[0].imshow(original, cmap='Greys', interpolation='none')
    axes[0].invert_yaxis()
    center_x, center_y = np.array(original.shape) / 2

    # Length of the arrow
    arrow_length = min(original.shape) / 3

    # Calculate arrow direction
    angle_radians = np.radians(wind_angle_degrees)
    dx = arrow_length * np.cos(angle_radians)
    dy = arrow_length * np.sin(angle_radians)

    # Draw the arrow
    axes[0].arrow(center_x, center_y, dx, -dy, head_width=0.5, head_length=0.5, fc='blue', ec='blue')

    axes[0].plot(p1[0], p1[1], 'bo')
    axes[0].plot(p2[0], p2[1], 'ro')
    axes[0].plot([p[0] for p in path_points], [p[1] for p in path_points], color=first_color)
    if conflict_pos is not None:
        axes[0].plot(conflict_pos[0], conflict_pos[1], 'ro')
    axes[0].set_title('Original Grid')
    #axes[0].axis('off')

    axes[1].imshow(rotated, cmap='Greys', interpolation='none')
    axes[1].invert_yaxis()
    axes[1].plot(p1_trans[0], p1_trans[1], 'bo')
    axes[1].plot(p2_trans[0], p2_trans[1], 'ro')
    axes[1].plot([p[0] for p in path_points_trans], [p[1] for p in path_points_trans], color=second_color)
    axes[1].set_title('Rotated Grid')
    #axes[1].axis('off')

    plt.show()

def rotate_and_scale(pt, radians, origin, h, w, h_new, w_new):
    x, y = pt
    offset_x, offset_y = origin
    adjusted_x = (x - offset_x)
    adjusted_y = (y - offset_y)
    cos_rad = math.cos(radians)
    sin_rad = math.sin(radians)
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
    xoffset, yoffset = (w_new - w)/2, (h_new - h)/2
    x1_new, y1_new = qx+xoffset, qy+yoffset
    return int(x1_new), int(y1_new)

def random_unblocked_point(grid):
    # Find the indices of all unblocked cells
    unblocked_indices = np.argwhere(grid == 0)
    print(unblocked_indices)

    if unblocked_indices.size == 0:
        raise ValueError("No unblocked cells found in the grid.")

    unblocked_tuples = [tuple(index) for index in unblocked_indices]
    # Choose a random index
    random_index = random.choice(unblocked_tuples)

    return tuple(random_index)

def main():
    x=100
    y=100
    grid = generate_grid(x, y, 30, 100)
    wind_angle_deg = 10

    #top-down, and Scipy's rotate goes clockwise
    map_angle_deg = 90+wind_angle_deg
    wind_display_angle_deg = -wind_angle_deg
    map_angle_rad = math.radians(map_angle_deg)
    rotated_grid = nd.rotate(grid, map_angle_deg, reshape=True, order=0, mode='constant', cval=1.0)
    rotated_grid = (rotated_grid > 0.5).astype(int)
    
    h, w = grid.shape[:2]
    h_new, w_new = rotated_grid.shape[:2]
    origin = (w/2, h/2)

    #create points
    p1 = (95, 55)#random_unblocked_point(grid)
    p2 = (5,45)#random_unblocked_point(grid)

    #rotate
    p1_new = rotate_and_scale(p1, map_angle_rad, origin, h, w, h_new, w_new)
    p2_new = rotate_and_scale(p2, map_angle_rad, origin, h, w, h_new, w_new)

    a=p2[0]-p1[0]
    b=p2[1]-p1[1]
    angle = (math.degrees(math.atan2(b, a))+360)%360

    # a = math.atan2(p1[1], p1[0])
    # b = math.atan2(p2[1], p2[0])
    # angle = math.degrees(a-b)%360
    print("Angle: "+str(angle))

    print("Dist: "+str(distance(p1[0], p1[1], p2[0], p2[1])))
    print("X diff: "+str(p2[0]-p1[0]))

    conflict_pos = hasLOS(grid, p1, p2)
    collision = False
    if(conflict_pos is not None):
        collision = True

    path = []
    if(collision):
        path = a_star(rotated_grid, h_new, w_new, p1_new, p2_new, False)

    #display
    display_grids(grid, rotated_grid, wind_display_angle_deg, p1, p1_new, p2, p2_new, path_points=[], path_points_trans=path, conflict=collision, conflict_pos=conflict_pos)

if __name__ == "__main__":
    main()