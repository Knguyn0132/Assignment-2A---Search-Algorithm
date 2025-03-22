from search_algorithm import SearchAlgorithm

class DFS(SearchAlgorithm):
    """
    Implements Depth-First Search (DFS) in a more readable way.
    """

    def __init__(self, graph):
        super().__init__(graph)  # Call the parent class constructor

    def search(self, start, goals):
        """
        Perform Depth-First Search (DFS) to find a path from start to one of the goal nodes.
        
        :param start: The starting node.
        :param goals: A set of goal nodes.
        :return: (goal_node, nodes_expanded, path_to_goal) or (None, nodes_expanded, []) if no path is found.
        """
        
        # Initialize the stack with the starting node and its path
        stack = [(start, [start])]
        
        # Keep track of visited nodes
        visited = set()
        
        # Counter for nodes expanded
        nodes_expanded = 0

        # Loop while there are still nodes in the stack
        while stack:
            # Pop the last-added node from the stack (DFS follows LIFO order)
            node, path = stack.pop()

            # Skip this node if it has already been visited
            if node in visited:
                continue

            # Mark the node as visited and increase the expanded nodes counter
            visited.add(node)
            nodes_expanded += 1

            # Check if we have reached a goal node
            if node in goals:
                return node, nodes_expanded, path  # Return the goal node, count, and path

            # Get all neighboring nodes of the current node
            neighbors = self.graph.adjacency_list.get(node, [])

            # Sort neighbors in reverse order (ensures correct DFS order)
            sorted_neighbors = sorted(neighbors, reverse=True)

            # Add neighbors to the stack (DFS will explore the last-added first)
            for neighbor, _ in sorted_neighbors: #extract the value ignore the cost
                new_path = path + [neighbor]  # Create a new path that includes the neighbor
                stack.append((neighbor, new_path))

        # If no solution is found, return None with the number of nodes expanded
        return None, nodes_expanded, []
