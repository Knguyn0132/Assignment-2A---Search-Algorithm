from search_algorithm import SearchAlgorithm
import heapq


class GBFS(SearchAlgorithm):
    """
    Implements Greedy Best-First Search using edge costs.
    """

    def __init__(self, graph):
        super().__init__(graph)

    def search(self, start, goals):
        """
        Execute Greedy Best-First Search from start node to any goal node,
        using edge costs as the evaluation metric.

        :param start: Starting node ID
        :param goals: Set of goal node IDs
        :return: (goal_reached, nodes_expanded, path)
        """
        # Priority queue with (cost, node, path)
        open_list = [(0, start, [start])]  # Start with cost 0
        visited = set()  # Set of visited nodes
        nodes_expanded = 0

        while open_list:
            # Pop the node with the lowest cost edge
            _, current, path = heapq.heappop(open_list)

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
            for neighbor, cost in self.graph.adjacency_list.get(current, []):
                if neighbor not in visited:
                    # Use cost as the evaluation metric (greedy approach)
                    heapq.heappush(open_list, (cost, neighbor, path + [neighbor]))

        # No path found
        return None, nodes_expanded, []
