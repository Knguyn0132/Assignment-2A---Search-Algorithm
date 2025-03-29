import heapq
import math
from search_algorithm import SearchAlgorithm

class AStar(SearchAlgorithm):
    def __init__(self, graph):
        """
        Initialize the A* search with a graph.
        
        :param graph: The SearchGraph object containing nodes and edges
        """
        super().__init__(graph)
        self.nodes = graph.node_coordinates
        
        # Build adjacency list and edges dictionary
        self.edges = {}
        self.adjacency_list = {}
        
        for from_node, neighbors in graph.adjacency_list.items():
            if from_node not in self.adjacency_list:
                self.adjacency_list[from_node] = []
            
            for to_node, cost in neighbors:
                self.adjacency_list[from_node].append((to_node, cost))
                self.edges[(from_node, to_node)] = cost
    
    def euclidean_distance(self, node1, node2):
        """
        Calculate Euclidean distance between two nodes using their coordinates.
        """
        # This function is used to calculate the heuristic value
        x1, y1 = self.nodes[node1]
        x2, y2 = self.nodes[node2]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def search(self, start, goals):
        """
        Execute A* Search from start node to any goal node.
        
        :param start: Starting node ID
        :param goals: Set or list of goal node IDs
        :return: (goal_reached, nodes_expanded, path)
        """
        # Convert goals to set for faster lookup
        goals = set(goals)
        
        # Select closest goal for heuristic calculation
        goal = min(goals, key=lambda g: self.euclidean_distance(start, g))
        
        # Track costs from start to each node
        g_scores = {start: 0}
        
        # Priority queue with (f_score, node_id, path)
        open_list = [(self.euclidean_distance(start, goal), start, [start])]
        heapq.heapify(open_list)
        
        # Track visited nodes
        visited = set()
        nodes_expanded = 0
        
        while open_list:
            # Pop the node with lowest f_score
            f_score, current, path = heapq.heappop(open_list)
            
            # If already visited, skip
            if current in visited:
                continue
            
            # Mark as visited and count as expanded
            visited.add(current)
            nodes_expanded += 1
            
            # Check if we've reached a goal
            if current in goals:
                return current, nodes_expanded, path
            
            # Process all neighbors
            neighbors = self.adjacency_list.get(current, [])
            for neighbor, cost in neighbors:
                # Calculate new g_score
                new_g_score = g_scores[current] + cost
                
                # Only consider this path if it's better than any previous one
                if neighbor not in g_scores or new_g_score < g_scores[neighbor]:
                    # Update g_score
                    g_scores[neighbor] = new_g_score
                    
                    # Calculate f_score
                    f_score = new_g_score + self.euclidean_distance(neighbor, goal)
                    
                    # Add to open list
                    new_path = path + [neighbor]
                    heapq.heappush(open_list, (f_score, neighbor, new_path))
        
        # No path found
        return None, nodes_expanded, []