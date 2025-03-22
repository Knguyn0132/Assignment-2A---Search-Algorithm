from dfs import DFS

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
        else:
            raise ValueError(f"Error: Unknown search method '{method}'")
