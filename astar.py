import heapq
import math
from search_algorithm import SearchAlgorithm

class AStar(SearchAlgorithm):
    def __init__(self, graph):
        """
        Initialize the A* search with a graph.

        """
        super().__init__(graph)
        self.nodes = graph.node_coordinates
        
        # Create an adjacency list 
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

        """
        # Convert goals to set for O(1) lookup
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
            
            # if the node is visited, skip 
            if current in visited:
                continue
            
            # add current node to visited set and increment nodes expanded
            visited.add(current)
            nodes_expanded += 1
            
            # check for special condition: if current is a goal
            if current in goals:
                return current, nodes_expanded, path
            
            # get all the neighbors of the current node in the adjacency list
            neighbors = sorted(self.adjacency_list.get(current, []))
            for neighbor, cost in neighbors:
                # calculate new g_score
                new_g_score = g_scores[current] + cost
                
                # only consider this path if it's better than any previous one
                if neighbor not in g_scores or new_g_score < g_scores[neighbor]:
                    # Update g_score
                    g_scores[neighbor] = new_g_score
                    
                    # Calculate h_score as minimum distance to any goal
                    h_score = min([self.euclidean_distance(neighbor, goal) for goal in goals])
                    
                    # Calculate f_score
                    f_score = new_g_score + h_score
                  
                    # Add to open list with unique entry ID
                    new_path = path + [neighbor]
                    heapq.heappush(open_list, (f_score, entry_id, neighbor, new_path))
                    entry_id += 1
        
        # No path found
        return None, nodes_expanded, []