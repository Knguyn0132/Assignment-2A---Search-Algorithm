from dfs import DFS
from bfs import BFS
from gbfs import GBFS
from cus1 import UCS
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
        elif method == "UCS":
            return UCS(graph)
        else:
            raise ValueError(f"Error: Unknown search method '{method}'")
