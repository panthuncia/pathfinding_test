from pathfinding_strategy import PathfindingStrategyInterface
from collections import defaultdict 
import math

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
        
class AStarPathfindingStrategy(PathfindingStrategyInterface):

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
            neighbors = neighbors_of_7(map, height, width, current[0], current[1])
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