from pathfinding_strategy import PathfindingStrategyInterface
import raycast
import math

class TackRaycastPathfindingStrategy(PathfindingStrategyInterface):

    @staticmethod
    def solve(map, start, goal, wind_angle_rad=0, no_go_angle_rad=math.radians(30)) -> list:
        h, w = map.shape[:2]
        distAtoB = math.sqrt(pow(goal[0]-start[0], 2)+pow(goal[1]-start[1], 2))
        print("dist A to B: "+str(distAtoB))

        angleAtoB = math.atan2(goal[1]-start[1], goal[0]-start[0])
        angle_nogo_1 = wind_angle_rad-math.pi-no_go_angle_rad
        angle_nogo_2 = wind_angle_rad-math.pi+no_go_angle_rad
        a=angleAtoB-(wind_angle_rad-math.pi-no_go_angle_rad)

        
        print("a: "+str(math.degrees(a)))

        b=math.pi-2*no_go_angle_rad
        print("b: "+str(math.degrees(b)))

        c = math.pi-a-b
        print("c: "+str(math.degrees(c)))

        length=(distAtoB*math.sin(c))/math.sin(b)
        # Calculate the intermediary points C1 and C2
        C1 = (int(start[0] + length * math.cos(angle_nogo_1)), int(start[1] + length * math.sin(angle_nogo_1)))
        length2 = math.sqrt(pow(goal[0]-C1[0], 2)+pow(goal[1]-C1[1], 2))
        C2 = (int(start[0] + length2 * math.cos(angle_nogo_2)), int(start[1] + length2 * math.sin(angle_nogo_2)))
        print("C1: "+str(C1))
        print("C2: "+str(C2))
        
        #check both tack directions, return the first that work
        if raycast.hasLOS(map, start, C1) is None:
            if raycast.hasLOS(map, C1, goal) is None:
                return [start, C1, goal]
        if raycast.hasLOS(map, start, C2) is None:
            if raycast.hasLOS(map, C2, goal) is None:
                return [start, C2, goal]
            
        return []
        
            
