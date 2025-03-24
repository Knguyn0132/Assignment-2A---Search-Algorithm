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
    
    def search(self, start, destination):
    """
    Execute A* Search from start node to a specific destination node.
    
    :param start: Starting node ID
    :param destination: Destination node ID (single target)
    :return: (nodes_expanded, path, cost)
    """
    # Use the destination directly for heuristic calculations
    goal = destination  # Define goal explicitly
    
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
        
        # Check if we've reached the goal
        if current == goal:
            return nodes_expanded, path, g_scores[current]
        
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
    return nodes_expanded, [], float('inf')


# Set up the graph from the problem
nodes = {
    1: (4, 1),
    2: (2, 2),
    3: (4, 4),
    4: (6, 3),
    5: (5, 6),
    6: (7, 5)
}

# These edges are directed - can only travel in the specified direction
edges = {
    (2, 1): 4,  # Can go from 2 to 1, but not from 1 to 2
    (3, 1): 5,
    (1, 3): 5,
    (2, 3): 4,
    (3, 2): 5,
    (4, 1): 6,
    (1, 4): 6,
    (4, 3): 5,
    (3, 5): 6,
    (5, 3): 6,
    (4, 5): 7,
    (5, 4): 8,
    (6, 3): 7,
    (3, 6): 7
}

# Create A* search instance
astar = AStar(nodes, edges)

# Execute search from node 2 to destinations 5 and 4
start_node = 2
goal_nodes = [5, 4]

goal_reached, nodes_expanded, path, cost = astar.search(start_node, goal_nodes)

# Print results
print(f"Goal reached: {goal_reached}")
print(f"Nodes expanded: {nodes_expanded}")
print(f"Path found: {path}")
print(f"Total cost: {cost}")

# Find path to other destination as well
if goal_reached in goal_nodes:
    other_goal = [g for g in goal_nodes if g != goal_reached][0]
    print(f"\nFinding path to other destination {other_goal}:")
    other_goal_reached, other_nodes_expanded, other_path, other_cost = astar.search(start_node, [other_goal])
    print(f"Goal reached: {other_goal_reached}")
    print(f"Nodes expanded: {other_nodes_expanded}")
    print(f"Path found: {other_path}")
    print(f"Total cost: {other_cost}")