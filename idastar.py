<<<<<<< HEAD
from search_algorithm import SearchAlgorithm
import math

class IDA(SearchAlgorithm):
    """
    Iterative Deepening A* (IDA*) search implementation.
    
    IDA* combines the memory efficiency of iterative deepening with the
    optimality and heuristic-informed nature of A*. It performs a series
    of depth-first searches with increasing f-value bounds.
    """
    
=======
import math
from search_algorithm import SearchAlgorithm

class IDAStar(SearchAlgorithm):
>>>>>>> 7d73caab1d08e70d7ba42f068d1e7453e21a5098
    def __init__(self, graph):
        """
        Initialize the IDA* search with a graph.
        
        :param graph: The SearchGraph object containing nodes and edges
        """
        super().__init__(graph)
        self.nodes = graph.node_coordinates
<<<<<<< HEAD
=======
        
        # Build adjacency list
        self.adjacency_list = {}
        for from_node, neighbors in graph.adjacency_list.items():
            if from_node not in self.adjacency_list:
                self.adjacency_list[from_node] = []
            
            for to_node, cost in neighbors:
                self.adjacency_list[from_node].append((to_node, cost))
>>>>>>> 7d73caab1d08e70d7ba42f068d1e7453e21a5098
    
    def euclidean_distance(self, node1, node2):
        """
        Calculate Euclidean distance between two nodes using their coordinates.
<<<<<<< HEAD
        
        :param node1: First node ID
        :param node2: Second node ID
        :return: Euclidean distance between the nodes
=======
>>>>>>> 7d73caab1d08e70d7ba42f068d1e7453e21a5098
        """
        x1, y1 = self.nodes[node1]
        x2, y2 = self.nodes[node2]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
<<<<<<< HEAD
    def get_min_heuristic(self, node, goals):
        """
        Calculate minimum heuristic distance from node to any goal.
        
        :param node: Current node ID
        :param goals: Set of goal node IDs
        :return: Minimum heuristic value to any target
        """
        return min([self.euclidean_distance(node, goal) for goal in goals])
    
    def search(self, start, goals):
        """
        Execute IDA* search from start node to any goal node.
=======
    def search(self, start, goals):
        """
        Execute IDA* Search from start node to any goal node.
>>>>>>> 7d73caab1d08e70d7ba42f068d1e7453e21a5098
        
        :param start: Starting node ID
        :param goals: Set or list of goal node IDs
        :return: (goal_reached, nodes_expanded, path)
        """
        # Convert goals to set for faster lookup
        goals = set(goals)
<<<<<<< HEAD
        
        # Check if start is already a goal
        if start in goals:
            return start, 1, [start]
        
        # Initial f-bound is the heuristic value of the start node
        f_bound = self.get_min_heuristic(start, goals)
        
        # Track path and nodes expanded across iterations
        path = [start]
        nodes_expanded = 0
        
        while True:
            # Get minimum f-value exceeding current bound, and nodes expanded in this iteration
            result, f_min, new_nodes_expanded = self._search_with_bound(
                start, 0, f_bound, path, goals, set())
            
            # Update total nodes expanded
            nodes_expanded += new_nodes_expanded
            
            # If a goal was found, return the result
            if result[0]:
                return result[1], nodes_expanded, result[2]
            
            # If no solution exists (f_min is infinity), return failure
            if f_min == float('inf'):
                return None, nodes_expanded, []
            
            # Increase f-bound for the next iteration
            f_bound = f_min
    
    def _search_with_bound(self, node, g, f_bound, path, goals, visited):
        """
        Recursive depth-first search with an f-bound cutoff.
        
        :param node: Current node being explored
        :param g: Cost from start to current node
        :param f_bound: Current f-bound limit
        :param path: Current path from start to node
        :param goals: Set of goal nodes
        :param visited: Set of visited nodes in current path (to avoid cycles)
        :return: ((found_goal, goal_node, path), min_f_exceeding_bound, nodes_expanded)
        """
        # Calculate f-value (g + h) for current node
        h = self.get_min_heuristic(node, goals)
        f = g + h
        
        # If f exceeds the bound, return the f-value as the new minimum
        if f > f_bound:
            return (False, None, []), f, 0
        
        # If current node is a goal, return success
        if node in goals:
            return (True, node, path), f, 1
        
        # Track minimum f-value of children for the next iteration
        f_min = float('inf')
        
        # Add current node to visited set
        visited.add(node)
        
        # Count this node as expanded
        nodes_expanded = 1
        
        # Get neighbors sorted by ascending node ID (as per assignment requirement)
        neighbors = sorted(self.graph.adjacency_list.get(node, []))
        
        for neighbor, cost in neighbors:
            # Skip if this neighbor is already in the current path (avoid cycles)
            if neighbor in visited:
                continue
            
            # Calculate g-value for the neighbor
            new_g = g + cost
            
            # Add neighbor to path
            new_path = path + [neighbor]
            
            # Recursively search with updated path and g-value
            result, new_f, new_nodes = self._search_with_bound(
                neighbor, new_g, f_bound, new_path, goals, visited.copy())
            
            # Update nodes expanded
            nodes_expanded += new_nodes
            
            # If goal found, return success
            if result[0]:
                return result, new_f, nodes_expanded
            
            # Update minimum f-value for next iteration
            f_min = min(f_min, new_f)
        
        # Remove current node from visited after exploring all its neighbors
        visited.remove(node)
        
        # Return failure, minimum f-value, and nodes expanded
        return (False, None, []), f_min, nodes_expanded
=======
        nodes_expanded = 0
        
        def min_heuristic(node):
            """Calculate minimum heuristic to any goal"""
            return min([self.euclidean_distance(node, goal) for goal in goals])
        
        def dfs(node, g_score, threshold, path, visited_on_path):
            nonlocal nodes_expanded
            
            # Calculate f_score = g_score + heuristic
            f_score = g_score + min_heuristic(node)
            
            # Debug: Print current node exploration
            # print(f"Exploring node {node}, g={g_score}, h={min_heuristic(node)}, f={f_score}, threshold={threshold}")
            
            # If f_score exceeds threshold, return f_score as new minimum
            if f_score > threshold:
                return f_score
            
            # If goal is reached, return "FOUND" signal
            if node in goals:
                return "FOUND"
            
            nodes_expanded += 1
            
            min_threshold = float('inf')
            
            # Explore neighbors
            for neighbor, cost in self.adjacency_list.get(node, []):
                # Skip if already in current path (avoid cycles)
                if neighbor in visited_on_path:
                    continue
                
                # Add neighbor to visited nodes on current path
                visited_on_path.add(neighbor)
                
                # Add neighbor to path
                path.append(neighbor)
                
                # Recursive DFS call
                result = dfs(neighbor, g_score + cost, threshold, path, visited_on_path)
                
                # If goal found, propagate the "FOUND" signal
                if result == "FOUND":
                    return "FOUND"
                
                # Update min_threshold with the minimum f_score that exceeded threshold
                if result < min_threshold:
                    min_threshold = result
                
                # Remove neighbor from path and visited set (backtrack)
                path.pop()
                visited_on_path.remove(neighbor)
            
            return min_threshold
        
        # Initial threshold is the heuristic from start to closest goal
        threshold = min_heuristic(start)
        
        # Initial path contains only the start node
        path = [start]
        
        while True:
            # Debug: Print iteration information
            # print(f"\nStarting iteration with threshold {threshold}")
            
            # Track visited nodes on the current path to avoid cycles
            visited_on_path = {start}
            
            result = dfs(start, 0, threshold, path, visited_on_path)
            
            # If goal found, return path and expanded nodes
            if result == "FOUND":
                return path[0], nodes_expanded, path
            
            # If no solution exists
            if result == float('inf'):
                return None, nodes_expanded, []
            
            # Update threshold for next iteration
            threshold = result
>>>>>>> 7d73caab1d08e70d7ba42f068d1e7453e21a5098
