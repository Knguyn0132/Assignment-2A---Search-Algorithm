import pytest
from search_graph import SearchGraph
from search_selector import SearchSelector

def run_search(filename, method):
    """
    Helper function to run a search algorithm on a given graph file.
    """
    graph = SearchGraph()
    graph.load_from_file(filename)

    search_algorithm = SearchSelector.get_search_algorithm(method, graph)
    return search_algorithm.search(graph.origin, graph.destinations)

def test_basic_pathfinding():
    """
    Test 1: Basic Pathfinding Test
    Ensures that DFS, BFS, and GBFS return a valid path using test_graph.txt.
    """

    filename = "test_graph.txt"  # Use the default provided graph
    expected_goals = {5, 4}  # The expected possible goals
    valid_paths = {
        5: [[2, 3, 5], [2, 1, 3, 5]],  # Both shortest (BFS) and DFS paths
        4: [[2, 1, 4]]  # Only one expected path to 4
    }
    expected_nodes_expanded = 3  # Minimum nodes that should be visited

    for method in ["DFS", "BFS", "GBFS"]:  # Only testing these 3
        goal, nodes_expanded, path = run_search(filename, method)

        print(f"{method} -> Goal: {goal}, Path: {path}, Nodes Expanded: {nodes_expanded}")

        assert goal in expected_goals, f"{method} failed: Expected goal in {expected_goals} but got {goal}"
        assert path in valid_paths[goal], f"{method} failed: Expected one of {valid_paths[goal]} but got {path}"
        assert nodes_expanded >= expected_nodes_expanded, f"{method} failed: Expected at least {expected_nodes_expanded} nodes expanded but got {nodes_expanded}"
