from pathfinding_strategy import PathfindingStrategyInterface
import raycast
class LinearRaycastPathfindingStrategy(PathfindingStrategyInterface):

    @staticmethod
    def solve(map, start, goal, wind_angle_rad=0, no_go_angle_rad=0) -> list:
        conflict_pos = raycast.hasLOS(map, start, goal)
        if(conflict_pos is not None):
            return []
        else:
            return [start, goal]