import heapq

class AStar:
    def __init__(self, nodes, edges):
        """
        Initialize the A* search with nodes and edges.
        
        :param nodes: Dictionary mapping node ID to (x,y) coordinates
        :param edges: Dictionary mapping (from_node, to_node) to edge cost
        """
        self.nodes = nodes
        self.edges = edges
        
        # Build adjacency list for easier neighbor access
        self.adjacency_list = {}
        for (from_node, to_node), cost in edges.items():
            if from_node not in self.adjacency_list:
                self.adjacency_list[from_node] = []
            self.adjacency_list[from_node].append((to_node, cost))
            
            # Note: We don't add the reverse direction since this is a directed graph
    
    def manhattan_distance(self, node1, node2):
        """
        Calculate Manhattan distance between two nodes using their coordinates.
        """
        x1, y1 = self.nodes[node1]
        x2, y2 = self.nodes[node2]
        return abs(x1 - x2) + abs(y1 - y2)
    
    def search(self, start, goals):
        """
        Execute A* Search from start node to any goal node.
        
        :param start: Starting node ID
        :param goals: Set or list of goal node IDs
        :return: (goal_reached, nodes_expanded, path, cost)
        """
        # Convert goals to set for faster lookup
        goals = set(goals)
        
        # Select closest goal for heuristic calculation
        goal = min(goals, key=lambda g: self.manhattan_distance(start, g))
        
        # Track costs from start to each node
        g_scores = {start: 0}
        
        # Priority queue with (f_score, node_id, path)
        open_list = [(self.manhattan_distance(start, goal), start, [start])]
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
                return current, nodes_expanded, path, g_scores[current]
            
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
                    f_score = new_g_score + self.manhattan_distance(neighbor, goal)
                    
                    # Add to open list
                    new_path = path + [neighbor]
                    heapq.heappush(open_list, (f_score, neighbor, new_path))
        
        # No path found
        return None, nodes_expanded, [], float('inf')


