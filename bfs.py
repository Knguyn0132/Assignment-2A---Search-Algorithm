from search_algorithm import SearchAlgorithm
from collections import deque

class BFS(SearchAlgorithm):
    """
    Implements Breadth-First Search.
    """

    def __init__(self, graph):
        super().__init__(graph)

    def search(self, start, goals):
        """
        Perform Breadth-First Search.
        :param start: The starting node.
        :param goals: A set of goal nodes.
        :return: (goal_node, num_nodes_expanded, path_to_goal)
        """
        queue = deque([(start, [start])])  # Queue stores (current node, path taken)
        visited = set()
        nodes_expanded = 0  # Track expanded nodes

        while queue:
            node, path = queue.popleft()

            if node in visited:
                continue
            visited.add(node)
            nodes_expanded += 1

            if node in goals:  # Goal reached
                return node, nodes_expanded, path

            for neighbor, _ in self.graph.graph.get(node, []):
                queue.append((neighbor, path + [neighbor]))

        return None, nodes_expanded, []  # No solution found
