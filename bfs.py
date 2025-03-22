from search_algorithm import SearchAlgorithm
from collections import deque

class BFS(SearchAlgorithm):
    """
    Implements Breadth-First Search.
    """

    def __init__(self, graph):
        super().__init__(graph)

    def search(self, start, goals):
        queue = deque([(start, [start])])
        visited = set()
        nodes_expanded = 0

        while queue:
            node, path = queue.popleft()

            if node in visited:
                continue
            visited.add(node)
            nodes_expanded += 1

            if node in goals:
                return node, nodes_expanded, path

            for neighbor, _ in self.graph.adjacency_list.get(node, []):
                queue.append((neighbor, path + [neighbor]))

        return None, nodes_expanded, []  # No solution found
