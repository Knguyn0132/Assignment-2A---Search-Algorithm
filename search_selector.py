# Modification to search_selector.py
from dfs import DFS
from bfs import BFS
from gbfs import GBFS
from astar import AStar
from ucs import UCS
from fringe import Fringe
from gmgs import GMGS  # Import the new GMGS class

class SearchSelector:
    """
    Handles search algorithm selection.
    """

    @staticmethod
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
        elif method == "AS":
            return AStar(graph)
        elif method == "CUS1":
            return UCS(graph)
        elif method == "CUS2":
            return Fringe(graph)
        elif method == "GMGS":
            return GMGS(graph)
        else:
            raise ValueError(f"Error: Unknown search method '{method}'")