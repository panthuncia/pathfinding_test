from pathfinding_strategy import PathfindingStrategyInterface
from collections import defaultdict 
import math
import time

from priority_queue import PriorityQueue

TURN_WEIGHT = 5

def distance(x1, y1, x2, y2):
    x2x1 = x2-x1
    y2y1 = y2-y1
    return math.sqrt(x2x1*x2x1 + y2y1*y2y1)

def is_cell_walkable(grid, h, w, x, y):
    if(x>=0 and y>=0 and x<w and y<h and grid[y][x]<=0.5):
        return True
    return False

def jumpDL(x, y, end, grid, h, w):
    print("JumpDL")
    while(True):
        x -= 1
        y -= 1
        if (not is_cell_walkable(grid, h, w, x, y)): 
            return None
        if x == end[0] and y == end[1]: 
            return x, y
        # diagonal cannot be forced on vertices.
        if jumpL(x,y, end, grid, h, w) is not None:
            return x,y
        if jumpD(x,y, end, grid, h, w) is not None:
            return x,y

def jumpDR(x, y, end, grid, h, w):
    print("JumpDR")
    i=0
    while(True):
        i+=1
        x += 1
        y -= 1
        if (not is_cell_walkable(grid, h, w, x-1, y)):
            print("Executions: "+str(i))
            return None
        if (x == end[0] and y == end[1]):
            print("Executions: "+str(i))
            return x,y
        # diagonal cannot be forced on vertices.
        if (jumpD(x,y, end, grid, h, w) is not None):
            print("Executions: "+str(i))
            return x,y
        if (jumpR(x,y, end, grid, h, w) is not None):
            print("Executions: "+str(i))
            return x,y

def jumpUL(x, y, end, grid, h, w):
    print("JumpUL")
    i=0
    while(True):
        i+=1
        x -= 1
        y += 1
        if (not is_cell_walkable(grid, h, w, x, y-1)):
            print("Executions: "+str(i))
            return None
        if (x == end[0] and y == end[1]):
            print("Executions: "+str(i))
            return x,y
        # diagonal cannot be forced on vertices.
        if (jumpL(x,y, end, grid, h, w) is not None):
            print("Executions: "+str(i))
            return x,y
        if (jumpU(x,y, end, grid, h, w) is not None):
            print("Executions: "+str(i))
            return x,y

def jumpUR(x, y, end, grid, h, w):
    print("JumpUR")
    while(True):
        x += 1
        y += 1
        if (not is_cell_walkable(grid, h, w, x-1, y-1)):
            return None
        if (x == end[0] and y == end[1]):
            return x,y
        # diagonal cannot be forced on vertices.
        if (jumpU(x,y, end, grid, h, w) is not None):
            return x,y
        if (jumpR(x,y, end, grid, h, w) is not None):
            return x,y

def jumpL(x, y, end, grid, h, w):
    #print("JumpL")
    while(True):
        x -= 1
        if (not is_cell_walkable(grid, h, w, x, y)):
            if (not is_cell_walkable(grid, h, w, x, y-1)):
                return None
            else:
                if (is_cell_walkable(grid, h, w, x-1, y)):
                    return x,y
        if (not is_cell_walkable(grid, h, w, x, y-1)):
            if (is_cell_walkable(grid, h, w, x-1, y-1)):
                return x,y
        if (x == end[0] and y == end[1]):
            return x,y

def jumpR(x, y, end, grid, h, w):
    #print("JumpR")
    while(True):
        x += 1
        if (not is_cell_walkable(grid, h, w, x-1, y)):
            if (not is_cell_walkable(grid, h, w, x-1, y-1)):
                return None
            else:
                if (is_cell_walkable(grid, h, w, x, y)):
                    return x,y
        if (not is_cell_walkable(grid, h, w, x-1, y-1)):
            if (is_cell_walkable(grid, h, w, x, y-1)):
                return x,y
        if (x == end[0] and y == end[1]):
            return x,y

def jumpD(x, y, end, grid, h, w):
    #print("JumpD")
    i=0
    while(True):
        i+=1
        y -= 1
        if (not is_cell_walkable(grid, h, w, x, y)):
            if (not is_cell_walkable(grid, h, w, x-1, y)):
                #print("D Executions: "+str(i))
                return None
            else:
                if (is_cell_walkable(grid, h, w, x, y-1)):
                    #print("D Executions: "+str(i)) 
                    return (x,y)
        if (not is_cell_walkable(grid, h, w, x-1, y)):
            if (is_cell_walkable(grid, h, w, x-1, y-1)):
                #print("D Executions: "+str(i))
                return x,y
        if (x == end[0] and y == end[1]):
            #print("D Executions: "+str(i))
            return x,y

def jumpU(x, y, end, grid, h, w):
    #print("JumpU")
    while(True):
        y += 1
        if (not is_cell_walkable(grid, h, w, x, y-1)):
            if (not is_cell_walkable(grid, h, w, x-1, y-1)):
                return None
            else:
                if (is_cell_walkable(grid, h, w, x, y)):
                    return x,y
        if (not is_cell_walkable(grid, h, w, x-1, y-1)):
            if (is_cell_walkable(grid, h, w, x-1, y)):
                return x,y
        if (x == end[0] and y == end[1]):
            return x,y

def jump(grid, h, w, x, y, dx, dy, start, goal):
    #print("jumping")
    #print("dx: "+str(dx)+" dy: "+str(dy))
    if (dx < 0):
        if (dy < 0):
            return jumpDL(x, y, goal, grid, h, w)
        elif (dy > 0):
            return jumpUL(x, y, goal, grid, h, w)
        else:
            return jumpL(x, y, goal, grid, h, w)
    elif (dx > 0):
        if (dy < 0):
            return jumpDR(x, y, goal, grid, h, w)
        elif (dy > 0):
            return jumpUR(x, y, goal, grid, h, w)
        else:
            return jumpR(x, y, goal, grid, h, w)
    else:
        if (dy < 0):
            return jumpD(x, y, goal, grid, h, w)
        else:
            return jumpU(x, y, goal, grid, h, w)



def neighbors_of_7(mapdata, start, goal, h, w, x, y):
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
    
    jump_points = []
    for neighbor in neighbors:
        dx = neighbor[0]-x
        dy = neighbor[1]-y
        before = time.process_time()
        jump_point = jump(mapdata, h, w, x, y, dx, dy, start, goal)
        print("jump time: "+str(time.process_time()-before))
        if jump_point is not None:
            #print("None...")
            jump_points.append(jump_point)
            #print(jump_point)

    print("Jumps from "+str(x)+" "+str(y)+": "+str(jump_point))
    return jump_points


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
        
class JPSPathfindingStrategy(PathfindingStrategyInterface):

    @staticmethod
    def solve(map, start, goal, wind_angle_rad=0, no_go_angle_rad=0) -> list:
        start = int(start[0]), int(start[1])
        goal = int(goal[0]), int(goal[1])

        height, width = map.shape[:2]
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
        num_expanded = 0
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
            neighbors = neighbors_of_7(map, start, goal, height, width, current[0], current[1])
            #print("Received "+str(len(neighbors))+" neighbors")
            for node in neighbors:
                if node in closed:
                    continue
                current_turn_penalty = 0
                if current in cameFrom:
                    current_turn_penalty = turn_penalty(cameFrom[current], current, node)
                tentative_gscore = gScore[current]+distance(node[0], node[1], current[0], current[1])+current_turn_penalty
                if(tentative_gscore<gScore[node]):
                    num_expanded+=1
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
        print("Num expanded: "+str(num_expanded))
        return reconstruct_path(goal, cameFrom)