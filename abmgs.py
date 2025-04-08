import math
from search_algorithm import SearchAlgorithm

class AStar_Based_Multi_Goal_Search(SearchAlgorithm):
    def __init__(self, graph):
        """Initialize with a graph"""
        super().__init__(graph)
        self.node_coordinates = graph.node_coordinates

    def find_best_goal(self, current, remaining_goals):
        """Find the best goal using A* evaluation"""
        from astar import AStar
        search_algo = AStar(self.graph)
        
        best_goal = None
        best_path = None
        min_cost = float('inf')
        min_nodes_expanded = 0
        
        # Try A* to each remaining goal
        for goal in remaining_goals:
            goal_reached, nodes_expanded, path = search_algo.search(current, [goal])
            if goal_reached is not None:
                # Calculate actual path cost
                path_cost = 0
                for i in range(len(path) - 1):
                    from_node = path[i]
                    to_node = path[i + 1]
                    for neighbor, cost in self.graph.adjacency_list.get(from_node, []):
                        if neighbor == to_node:
                            path_cost += cost
                            break
                
                if path_cost < min_cost:
                    min_cost = path_cost
                    best_goal = goal
                    best_path = path
                    min_nodes_expanded = nodes_expanded
        
        return best_goal, min_nodes_expanded, best_path

    def search(self, start, goals):
        remaining_goals = set(goals)
        current_position = start
        total_path = [start]
        total_nodes_expanded = 0
        goals_visited_order = []
        segment_info = []

        while remaining_goals:
            # Use A* to find the best next goal
            next_goal, nodes_expanded, path = self.find_best_goal(current_position, remaining_goals)
            
            if next_goal is None:
                print(f"Warning: No reachable goals from {current_position}")
                break
            
            # Update tracking variables
            total_nodes_expanded += nodes_expanded

            # Calculate segment cost
            segment_cost = 0
            if len(path) > 1:
                for i in range(1, len(path)):
                    from_node = path[i-1]
                    to_node = path[i]
                    for neighbor, cost in self.graph.adjacency_list.get(from_node, []):
                        if neighbor == to_node:
                            segment_cost += cost
                            break

            # Update path
            if len(total_path) > 0:
                path = path[1:]
            total_path.extend(path)
            
            # Update current position and tracking info
            current_position = next_goal
            goals_visited_order.append(next_goal)
            segment_info.append({
                'goal': next_goal,
                'nodes_expanded': nodes_expanded,
                'path_length': len(path),
                'cost': segment_cost
            })
            
            remaining_goals.remove(next_goal)

        if not goals_visited_order:
            return None, total_nodes_expanded, []

        self.goals_visited_order = goals_visited_order
        self.segment_info = segment_info
        self.total_cost = sum(segment['cost'] for segment in segment_info)

        return goals_visited_order[-1], total_nodes_expanded, total_path