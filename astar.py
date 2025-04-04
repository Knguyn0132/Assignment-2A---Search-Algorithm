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
        Execute A* Search from start node to any goal node.
        
        :param start: Starting node ID
        :param goals: Set or list of goal node IDs
        :return: (goal_reached, nodes_expanded, path)
        """
        # Convert goals to set for faster lookup
        goals = set(goals)
        
        # Track costs from start to each node
        g_scores = {start: 0}
        
        # Create a unique ID for each queue entry to avoid comparing paths
        entry_id = 0
        
        # Priority queue with (f_score, entry_id, node_id, path)
        open_list = []
        
        # Calculate initial heuristic - use min distance to any goal
        initial_h = min([self.euclidean_distance(start, goal) for goal in goals])
        heapq.heappush(open_list, (initial_h, entry_id, start, [start]))
        entry_id += 1
        
        # Track visited nodes
        visited = set()
        nodes_expanded = 0
        
        while open_list:
            # Pop the node with lowest f_score
            _, _, current, path = heapq.heappop(open_list)
            
            # If already visited, skip
            if current in visited:
                continue
            
            # Mark as visited and count as expanded
            visited.add(current)
            nodes_expanded += 1
            
            # Debug: print current node exploration
            # print(f"Exploring node {current} with path {path}")
            
            # Check if we've reached a goal
            if current in goals:
                return current, nodes_expanded, path
            
            # Process all neighbors
            neighbors = sorted(self.adjacency_list.get(current, []))
            for neighbor, cost in neighbors:
                # Calculate new g_score
                new_g_score = g_scores[current] + cost
                
                # Only consider this path if it's better than any previous one
                if neighbor not in g_scores or new_g_score < g_scores[neighbor]:
                    # Update g_score
                    g_scores[neighbor] = new_g_score
                    
                    # Calculate h_score as minimum distance to any goal
                    h_score = min([self.euclidean_distance(neighbor, goal) for goal in goals])
                    
                    # Calculate f_score
                    f_score = new_g_score + h_score
                    
                    # Debug: print neighbor evaluation
                    # print(f"  Neighbor {neighbor}: g={new_g_score}, h={h_score}, f={f_score}")
                    
                    # Add to open list with unique entry ID
                    new_path = path + [neighbor]
                    heapq.heappush(open_list, (f_score, entry_id, neighbor, new_path))
                    entry_id += 1
        
        # No path found
        return None, nodes_expanded, []