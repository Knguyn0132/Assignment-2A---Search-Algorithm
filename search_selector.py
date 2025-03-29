from dfs import DFS
from bfs import BFS
from gbfs import GBFS
from astar import AStar
from ucs import UCS
from weighted_astar import WeightedAStar

class SearchSelector:
    """
    Handles search algorithm selection.
    """

    @staticmethod #Call the method directly without creating an instance
    def get_search_algorithm(method, graph):
        """
        Returns an instance of the requested search algorithm.
        :param method: The search algorithm name (DFS, BFS, etc.).
        :param graph: The graph to be searched.
        :return: An instance of the selected search algorithm.
        """
        if method == "DFS":
            return DFS(graph)
        elif method == "BFS":
            return BFS(graph)
        elif method == "GBFS":
            return GBFS(graph)
        elif method == "A*":
            return AStar(graph)
        elif method == "UCS":
            return UCS(graph)
        elif method == "WEIGHTED_A*":  # Add Weighted A*
            return WeightedAStar(graph, weight=1.5)  # Adjust weight as needed
        else:
            raise ValueError(f"Error: Unknown search method '{method}'")
