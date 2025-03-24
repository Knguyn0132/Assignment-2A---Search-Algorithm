from search_algorithm import SearchAlgorithm
import heapq


class AStar(SearchAlgorithm):
    """
    Implements A* Search algorithm using both path cost and heuristic.
    """

    def __init__(self, graph, heuristic_fn):
        """
        Initialize with graph and heuristic function.
        
        :param graph: Graph with adjacency list
        :param heuristic_fn: Function that takes (node, goal) and returns estimated cost
        """
        super().__init__(graph)
        self.heuristic = heuristic_fn

    def search(self, start, goals):
        """
        Execute A* Search from start node to any goal node,
        using f(n) = g(n) + h(n) as the evaluation metric.

        :param start: Starting node ID
        :param goals: Set of goal node IDs
        :return: (goal_reached, nodes_expanded, path)
        """
        # For A*, we need to track the cost to reach each node (g score)
        g_scores = {start: 0}  # Initialize g_scores with the start node having a cost of 0.
        
        # Select the closest goal for heuristic calculation if multiple goals
        goal = min(goals, key=lambda g: self.heuristic(start, g)) if goals else None
        # If there are multiple goals, choose the one with the smallest heuristic value from the start node.

        # Priority queue with (f_score, g_score, node, path)
        # f_score = g_score + heuristic
        open_list = [(self.heuristic(start, goal), 0, start, [start])]
        # Initialize the priority queue with the start node. The f_score is calculated as g_score (0) + heuristic.

        visited = set()  # Set of visited nodes
        nodes_expanded = 0  # Counter to track the number of nodes expanded during the search.

        while open_list:  # Continue processing while there are nodes in the priority queue.
            # Pop the node with the lowest f_score
            _, g_score, current, path = heapq.heappop(open_list)
            # Extract the node with the smallest f_score from the priority queue.

            # If already visited, skip
            if current in visited:
                continue
            # Skip processing if the current node has already been visited.

            # Mark as visited and count as expanded
            visited.add(current)  # Add the current node to the visited set.
            nodes_expanded += 1  # Increment the counter for expanded nodes.

            # Check if we've reached a goal
            if current in goals:
                return current, nodes_expanded, path

            # Process all neighbors
            for neighbor, edge_cost in self.graph.adjacency_list.get(current, []):
                if neighbor not in visited:
                    # Calculate new g_score for this neighbor
                    new_g_score = g_score + edge_cost
                    
                    # Only consider this path if it's better than any previous one
                    if neighbor not in g_scores or new_g_score < g_scores[neighbor]:
                        # Update the g_score
                        g_scores[neighbor] = new_g_score
                        
                        # Calculate f_score = g_score + heuristic
                        f_score = new_g_score + self.heuristic(neighbor, goal)
                        
                        # Add to open list
                        heapq.heappush(open_list, (f_score, new_g_score, neighbor, path + [neighbor]))

        # No path found
        return None, nodes_expanded, []