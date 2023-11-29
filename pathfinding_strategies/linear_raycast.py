from pathfinding_strategy import PathfindingStrategyInterface
import raycast
class LinearRaycastPathfindingStrategy(PathfindingStrategyInterface):

    @staticmethod
    def solve(map, start, goal) -> list:
        conflict_pos = raycast.hasLOS(map, start, goal)
        if(conflict_pos is not None):
            return []
        else:
            return [start, goal]