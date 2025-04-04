# gmgs.py
import math
from search_algorithm import SearchAlgorithm

class GMGS(SearchAlgorithm):
    """
    Greedy Multi-Goal Search implementation.
    
    This algorithm finds paths to all destination nodes by greedily selecting
    the next closest goal and incrementally building a complete tour.
    """
    
    def __init__(self, graph):
        """Initialize with a graph"""
        super().__init__(graph)
        self.node_coordinates = graph.node_coordinates
    
    def euclidean_distance(self, node1, node2):
        """Calculate Euclidean distance between two nodes using their coordinates."""
        if node1 not in self.node_coordinates or node2 not in self.node_coordinates:
            return float('inf')
            
        x1, y1 = self.node_coordinates[node1]
        x2, y2 = self.node_coordinates[node2]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def find_closest_goal(self, current, remaining_goals):
        """Find the closest goal to the current node based on Euclidean distance."""
        closest_goal = None
        min_distance = float('inf')
        
        for goal in remaining_goals:
            distance = self.euclidean_distance(current, goal)
            if distance < min_distance:
                min_distance = distance
                closest_goal = goal
                
        return closest_goal
    
    def search(self, start, goals):
        """
        Execute Greedy Multi-Goal Search from start node to all goal nodes.
        
        :param start: Starting node ID
        :param goals: Set or list of goal node IDs
        :return: (last_goal_reached, total_nodes_expanded, complete_path)
        """
        # Create a copy of goals to modify
        remaining_goals = set(goals)
        
        # Initialize tracking variables
        current_position = start
        total_path = [start]
        total_nodes_expanded = 0
        goals_visited_order = []
        segment_info = []
        
        # Continue until all goals are visited
        while remaining_goals:
            # Find the closest unvisited goal
            next_goal = self.find_closest_goal(current_position, remaining_goals)
            
            # Use an existing algorithm to find the path to this goal
            # We'll use A* as it's generally efficient for single-goal pathfinding
            from astar import AStar
            search_algo = AStar(self.graph)
            goal_reached, nodes_expanded, path = search_algo.search(current_position, [next_goal])
            
            # If no path found to this goal, skip it
            if goal_reached is None:
                print(f"Warning: No path found to goal {next_goal}. Skipping.")
                remaining_goals.remove(next_goal)
                continue
            
            ## After finding a path to next_goal

            # Update tracking variables
            total_nodes_expanded += nodes_expanded

            # Calculate segment cost correctly
            segment_cost = 0
            from_node = current_position  # Starting point for this segment

            # Iterate through the path pairs (excluding the starting point which is already in our path)
            if len(path) > 1:  # Make sure path has at least 2 nodes
                for i in range(1, len(path)):
                    to_node = path[i]
                    from_node = path[i-1]
                    
                    # Find the cost between from_node and to_node
                    edge_cost = None
                    for neighbor, cost in self.graph.adjacency_list.get(from_node, []):
                        if neighbor == to_node:
                            edge_cost = cost
                            break
                    
                    # Add the cost if found
                    if edge_cost is not None:
                        segment_cost += edge_cost
                    else:
                        print(f"Warning: Edge cost not found for {from_node}->{to_node}")

            # Skip the first node in the path as it's already in our total path
            if len(total_path) > 0:
                path = path[1:]
                
            total_path.extend(path)
            current_position = next_goal  # Update current position to the goal we just reached
            goals_visited_order.append(next_goal)
            
            segment_info.append({
                'goal': next_goal,
                'nodes_expanded': nodes_expanded,
                'path_length': len(path),
                'cost': segment_cost
            })
            
            # Remove the visited goal
            remaining_goals.remove(next_goal)
        
        # Return the last goal reached, total nodes expanded, and complete path
        if not goals_visited_order:
            return None, total_nodes_expanded, []
            
        # Store the detailed information as an attribute for later access
        self.goals_visited_order = goals_visited_order
        self.segment_info = segment_info
        self.total_cost = sum(segment['cost'] for segment in segment_info)
        
        return goals_visited_order[-1], total_nodes_expanded, total_path