
class PathfindingStrategyInterface:
    def solve(map, start, goal, wind_angle_rad=0, no_go_angle=0) -> list:
        """Solve a pathfinding problem. Returns list of points, empty if pathfinding failed"""
        pass