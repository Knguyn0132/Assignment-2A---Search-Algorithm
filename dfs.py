from search_algorithm import SearchAlgorithm

class DFS(SearchAlgorithm):
    """
    Implements Depth-First Search.
    """

    def __init__(self, graph):
        super().__init__(graph)  # Call the parent class constructor

    def search(self, start, goals):
        """
        Perform Depth-First Search.
        :param start: The starting node.
        :param goals: A set of goal nodes.
        :return: (goal_node, num_nodes_expanded, path_to_goal)
        """
        stack = [(start, [start])]  # Stack stores (current node, path taken)
        visited = set()
        nodes_expanded = 0  # Keep track of expanded nodes

        while stack:
            node, path = stack.pop()

            if node in visited:
                continue
            visited.add(node)
            nodes_expanded += 1

            if node in goals:  # Goal reached
                return node, nodes_expanded, path

            for neighbor, _ in sorted(self.graph.graph.get(node, []), reverse=True):
                stack.append((neighbor, path + [neighbor]))

        return None, nodes_expanded, []  # No solution found
