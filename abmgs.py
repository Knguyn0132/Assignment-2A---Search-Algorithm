import math
from search_algorithm import SearchAlgorithm

class AStar_Based_Multi_Goal_Search(SearchAlgorithm):
    def __init__(self, graph):
        # Initialize with a graph
        super().__init__(graph)
        self.node_coordinates = graph.node_coordinates

    def find_best_goal(self, current, remaining_goals):
        # Find the best goal using A* evaluation
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
                # Calculate the actual path cost
                path_cost = 0
                for i in range(len(path) - 1):
                    from_node = path[i]
                    to_node = path[i + 1]
                    for neighbor, cost in self.graph.adjacency_list.get(from_node, []):
                        if neighbor == to_node:
                            path_cost += cost
                            break
                
                # Update the best goal if this path is cheaper
                if path_cost < min_cost:
                    min_cost = path_cost
                    best_goal = goal
                    best_path = path
                    min_nodes_expanded = nodes_expanded
        
        return best_goal, min_nodes_expanded, best_path

    def search(self, start, goals):
        # Execute multi-goal search using A* for each segment

        remaining_goals = set(goals)  # Track goals that haven't been visited
        current_position = start  # Start from the initial node
        total_path = [start]  # Track the full path
        total_nodes_expanded = 0  # Count total nodes expanded
        goals_visited_order = []  # Track the order in which goals are visited
        segment_info = []  # Store information about each segment

        while remaining_goals:
            # Use A* to find the best next goal
            next_goal, nodes_expanded, path = self.find_best_goal(current_position, remaining_goals)
            
            if next_goal is None:  # If no reachable goals remain
                print(f"Warning: No reachable goals from {current_position}")
                break
            
            # Update total nodes expanded
            total_nodes_expanded += nodes_expanded

            # Calculate the cost of the current segment
            segment_cost = 0
            if len(path) > 1:
                for i in range(1, len(path)):
                    from_node = path[i-1]
                    to_node = path[i]
                    for neighbor, cost in self.graph.adjacency_list.get(from_node, []):
                        if neighbor == to_node:
                            segment_cost += cost
                            break

            # Update the total path, skipping the first node if it's already included
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
            
            # Remove the reached goal from the remaining goals
            remaining_goals.remove(next_goal)

        if not goals_visited_order:  # If no goals were visited
            return None, total_nodes_expanded, []

        # Store additional information about the search
        self.goals_visited_order = goals_visited_order
        self.segment_info = segment_info
        self.total_cost = sum(segment['cost'] for segment in segment_info)

        # Return the last goal reached, total nodes expanded, and the full path
        return goals_visited_order[-1], total_nodes_expanded, total_path