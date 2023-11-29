

import numpy as np
import matplotlib.pyplot as plt
import random
import math
import scipy.ndimage as nd 
from typing import List

from pathfinding_strategies.linear_raycast import LinearRaycastPathfindingStrategy
from pathfinding_strategies.a_star import AStarPathfindingStrategy

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

def display_grids(original, rotated, wind_angle_degrees, path_points, path_points_trans):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    axes[0].imshow(original, cmap='Greys', interpolation='none')
    axes[0].invert_yaxis()
    axes[0].set_title('Original Grid')
    
    axes[1].imshow(rotated, cmap='Greys', interpolation='none')
    axes[1].invert_yaxis()
    axes[1].set_title('Rotated Grid')
    center_x, center_y = np.array(original.shape) / 2

    # Length of the arrow
    arrow_length = min(original.shape) / 3

    # Calculate arrow direction
    angle_radians = np.radians(wind_angle_degrees)
    dx = arrow_length * np.cos(angle_radians)
    dy = arrow_length * np.sin(angle_radians)

    # Draw the arrow
    axes[0].arrow(center_x, center_y, dx, -dy, head_width=3, head_length=3, fc='blue', ec='blue')

    if len(path_points)>0:
        axes[0].plot(path_points[0][0], path_points[0][1], 'bo')
        axes[0].plot(path_points[-1][0], path_points[-1][1], 'ro')
        axes[0].plot([p[0] for p in path_points], [p[1] for p in path_points], color='green')
        #axes[0].axis('off')

    if len(path_points_trans)>0:
        axes[1].plot(path_points_trans[0][0], path_points_trans[0][1], 'bo')
        axes[1].plot(path_points_trans[-1][0], path_points_trans[-1][1], 'ro')
        axes[1].plot([p[0] for p in path_points_trans], [p[1] for p in path_points_trans], color='green')
        #axes[1].axis('off')
    
    fig.canvas.mpl_connect('key_press_event', lambda event: on_key_press(event, plt))

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
    return x1_new, y1_new

def random_unblocked_point(grid):
    # Find the indices of all unblocked cells
    unblocked_indices = np.argwhere(grid == 0)

    if unblocked_indices.size == 0:
        raise ValueError("No unblocked cells found in the grid.")

    unblocked_tuples = [tuple(index) for index in unblocked_indices]
    # Choose a random index
    random_index = random.choice(unblocked_tuples)

    return tuple(random_index)

def is_within_30_degrees(A, B):
    # Normalize angles to 0 to 360 degrees
    A = A % 360
    B = B % 360

    # Compute the absolute difference
    difference = abs(A - B)

    # Adjust for angles crossing 0 degrees
    if difference > 180:
        difference = 360 - difference

    # Check if the difference is within 30 degrees
    return difference <= 30

def opposite_angle(angle):
    return (angle + 180) % 360

def on_key_press(event, plt):
    if event.key == ' ':
        plt.close(event.canvas.figure)
        main()

def main():
    x=100
    y=100
    grid = generate_grid(x, y, 30, 100)
    wind_angle_deg = random.randrange(360)# 10

    #top-down, and Scipy's rotate goes clockwise
    map_angle_deg = 90+wind_angle_deg
    wind_display_angle_deg = -wind_angle_deg
    map_angle_rad = math.radians(map_angle_deg)
    rotated_grid = nd.rotate(grid, map_angle_deg, reshape=True, order=0, mode='constant', cval=1.0)
    rotated_grid = (rotated_grid > 0.5).astype(int)
    
    h, w = grid.shape[:2]
    h_new, w_new = rotated_grid.shape[:2]
    origin = (w/2, h/2)
    new_origin = (w_new/2, h_new/2)

    #create points
    p1 = random_unblocked_point(grid)#(95, 55)
    p2 = random_unblocked_point(grid)#(5,45)

    #rotate
    p1_proj = rotate_and_scale(p1, map_angle_rad, origin, h, w, h_new, w_new)
    p2_proj = rotate_and_scale(p2, map_angle_rad, origin, h, w, h_new, w_new)

    a=p2[0]-p1[0]
    b=p2[1]-p1[1]
    angle = (math.degrees(math.atan2(b, a))+360)%360

    # a = math.atan2(p1[1], p1[0])
    # b = math.atan2(p2[1], p2[0])
    # angle = math.degrees(a-b)%360
    # print("Angle: "+str(angle))

    # print("Dist: "+str(distance(p1[0], p1[1], p2[0], p2[1])))
    # print("X diff: "+str(p2[0]-p1[0]))

    wind_blocked = False
    if(is_within_30_degrees(wind_angle_deg, opposite_angle(angle))):
        print("Wind blocks direct path")
        wind_blocked = True

    path = LinearRaycastPathfindingStrategy.solve(grid, p1, p2)
    path_trans = [rotate_and_scale(p, map_angle_rad, origin, h, w, h_new, w_new) for p in path]
    collision = False
    if len(path)==0:
        print("raycast failed")
        collision = True

    if wind_blocked or collision:
        print("Running A*")
        path_trans = AStarPathfindingStrategy.solve(rotated_grid, p1_proj, p2_proj)
        #un-rotate
        path = [rotate_and_scale(p, -map_angle_rad, new_origin, h_new, w_new, h, w) for p in path_trans]
    #display
    display_grids(original=grid, rotated=rotated_grid, wind_angle_degrees=wind_display_angle_deg, path_points=path, path_points_trans=path_trans)

if __name__ == "__main__":
    main()