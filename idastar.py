import math
from search_algorithm import SearchAlgorithm

class IDAStar(SearchAlgorithm):
    def __init__(self, graph):
        """
        Initialize the IDA* search with a graph.
        
        :param graph: The SearchGraph object containing nodes and edges
        """
        super().__init__(graph)
        self.nodes = graph.node_coordinates
        
        # Build adjacency list
        self.adjacency_list = {}
        for from_node, neighbors in graph.adjacency_list.items():
            if from_node not in self.adjacency_list:
                self.adjacency_list[from_node] = []
            
            for to_node, cost in neighbors:
                self.adjacency_list[from_node].append((to_node, cost))
    
    def euclidean_distance(self, node1, node2):
        """
        Calculate Euclidean distance between two nodes using their coordinates.
        """
        x1, y1 = self.nodes[node1]
        x2, y2 = self.nodes[node2]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def search(self, start, goals):
        """
        Execute IDA* Search from start node to any goal node.
        
        :param start: Starting node ID
        :param goals: Set or list of goal node IDs
        :return: (goal_reached, nodes_expanded, path)
        """
        # Convert goals to set for faster lookup
        goals = set(goals)
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