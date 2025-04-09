from search_algorithm import SearchAlgorithm
import heapq


class GBFS(SearchAlgorithm):
    """
    Implements Greedy Best-First Search.
    """

    def __init__(self, graph):
        super().__init__(graph)

    def heuristic(self, node, goal):
        """
        Calculate the heuristic distance between node and goal using Euclidean distance.

        """
        if node not in self.graph.node_coordinates or goal not in self.graph.node_coordinates:
            return float('inf')  # Return infinity if coordinates are not available

        # Get coordinates
        x1, y1 = self.graph.node_coordinates[node]
        x2, y2 = self.graph.node_coordinates[goal]

        # Return the actual Euclidean distance
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def get_best_goal_heuristic(self, node, goals):
        """
        Find the minimum heuristic distance to any goal.
        """
        return min(self.heuristic(node, goal) for goal in goals)

    def search(self, start, goals):
        """
        Execute Greedy Best-First Search from start node to any goal node.

        """
        # Counter to track the order in which nodes are added to the priority queue (used for tie-breaking)
        insertion_counter = 0

        # Priority queue: (heuristic_value, insertion_order, node, path)
        open_list = [(self.get_best_goal_heuristic(start, goals), insertion_counter, start, [start])]
        heapq.heapify(open_list)
        closed_set = set()  # Set of visited nodes
        nodes_expanded = 0

        while open_list:
            # Get the node with the smallest heuristic value from the priority queue
            h, insertion_order, current_node, path = heapq.heappop(open_list)

            # Continue if the node has already been visited
            if current_node in closed_set:
                continue

            # Mark the current node as visited and increment the expanded node counter
            closed_set.add(current_node)
            nodes_expanded += 1

            # Return when reach a goal node
            if current_node in goals:
                return current_node, nodes_expanded, path

            # Explore neighbors
            for neighbor, _ in sorted(self.graph.adjacency_list.get(current_node, [])):
                if neighbor not in closed_set:
                    # Increment insertion counter for stable tie-breaking
                    insertion_counter += 1
                    # Calculate heuristic for this neighbor
                    h = self.get_best_goal_heuristic(neighbor, goals)
                    # Add to open list with path
                    heapq.heappush(open_list, (
                        h,  # Primary sorting by heuristic value
                        insertion_counter,  # Secondary sorting by insertion order (ensures first-added expands first)
                        neighbor,  # Tertiary sorting by node value (ensures ascending order when all else is equal)
                        path + [neighbor]  # Path to the neighbor
                    ))

        # No path found
        return None, nodes_expanded, []
