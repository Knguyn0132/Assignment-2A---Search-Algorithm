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

        :param node: Current node ID
        :param goal: Goal node ID
        :return: Euclidean distance between the nodes
        """
        if node not in self.graph.node_coordinates or goal not in self.graph.node_coordinates:
            return float('inf')  # Return infinity if coordinates are not available

        # Get coordinates
        x1, y1 = self.graph.node_coordinates[node]
        x2, y2 = self.graph.node_coordinates[goal]

        # Return the actual Euclidean distance (not squared)
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def get_best_goal_heuristic(self, node, goals):
        """
        Find the minimum heuristic distance to any goal.

        :param node: Current node ID
        :param goals: Set of goal node IDs
        :return: Minimum heuristic distance to any goal
        """
        return min(self.heuristic(node, goal) for goal in goals)

    def search(self, start, goals):
        """
        Execute Greedy Best-First Search from start node to any goal node.

        :param start: Starting node ID
        :param goals: Set of goal node IDs
        :return: (goal_reached, nodes_expanded, path)
        """
        # Track insertion order to resolve tie-breaks
        insertion_counter = 0
        # Initialize the priority queue with (heuristic_value, node, path)
        # Priority queue with (Using heuristic_value as priority (lower values first), node_value, insertion_order, node, path)
        open_list = [(self.get_best_goal_heuristic(start, goals), start, insertion_counter, [start])]
        heapq.heapify(open_list)
        closed_set = set()  # Set of visited nodes
        nodes_expanded = 0

        while open_list:
            # Pop the node with the smallest heuristic value, smallest node value, and earliest insertion
            h, current_node, insertion_order, path = heapq.heappop(open_list)

            # If we've already visited this node, skip it
            if current_node in closed_set:
                continue

            # Mark as visited and count as expanded
            closed_set.add(current_node)
            nodes_expanded += 1

            # Check if we've reached a goal
            if current_node in goals:
                return current_node, nodes_expanded, path

            # Explore neighbors
            for neighbor, _ in self.graph.adjacency_list.get(current_node, []):
                if neighbor not in closed_set:
                    # Increment insertion counter for stable tie-breaking
                    insertion_counter += 1
                    # Calculate heuristic for this neighbor
                    h = self.get_best_goal_heuristic(neighbor, goals)
                    # Add to open list with path
                    heapq.heappush(open_list, (
                        h,  # Primary sorting by heuristic value
                        neighbor,  # Secondary sorting by node value
                        insertion_counter,  # Tertiary sorting by insertion order
                        path + [neighbor]  # Path to the neighbor
                    ))

        # No path found
        return None, nodes_expanded, []