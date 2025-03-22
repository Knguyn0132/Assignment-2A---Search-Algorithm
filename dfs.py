from search_algorithm import SearchAlgorithm

class DFS(SearchAlgorithm):
    """
    Implements Depth-First Search.
    """

    def __init__(self, graph):
        super().__init__(graph)

    def search(self, start, goals):
        stack = [(start, [start])]
        visited = set()
        nodes_expanded = 0

        while stack:
            node, path = stack.pop()

            if node in visited:
                continue
            visited.add(node)
            nodes_expanded += 1

            if node in goals:
                return node, nodes_expanded, path

            for neighbor, _ in sorted(self.graph.adjacency_list.get(node, []), reverse=True):
                stack.append((neighbor, path + [neighbor]))

        return None, nodes_expanded, []  # No solution found
