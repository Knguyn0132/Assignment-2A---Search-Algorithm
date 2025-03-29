from search_algorithm import SearchAlgorithm
from collections import deque

class BFS(SearchAlgorithm):

    def __init__(self, graph):
        super().__init__(graph)

    def search(self, start, goals):
        queue = deque([(start, [start])]) #use queue instead of stack, deque is better than list for popping in front
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

            for neighbor, _ in sorted(self.graph.adjacency_list.get(node, [])):
                queue.append((neighbor, path + [neighbor])) #doesn't matter the order of what node is being added to the queue first it will discover all anyway 
                                                            #E.g: D and E or E and D

        return None, nodes_expanded, []  # No solution found
