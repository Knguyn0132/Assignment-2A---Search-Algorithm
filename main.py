import sys
from search_graph import SearchGraph
from search_selector import SearchSelector

def main():
    if len(sys.argv) != 3: # always 3 input: name of main, filename and search method
        print("Usage: python search.py <filename> <method>")
        exit(1)

    filename = sys.argv[1]
    method = sys.argv[2].upper()

    # Load the graph
    graph = SearchGraph()
    graph.load_from_file(filename)

    # Get the correct search algorithm instance
    try:
        search_algorithm = SearchSelector.get_search_algorithm(method, graph)
    except ValueError as e:
        print(e)
        exit(1)

    # Run the search
    goal, nodes_expanded, path = search_algorithm.search(graph.origin, graph.destinations)

    # Print result
    if goal is not None:
        print(f"{filename} {method}")
        print(f"{goal} {nodes_expanded}")
        print(" -> ".join(map(str, path)))
    else:
        print(f"{filename} {method}")
        print("No solution found.")

if __name__ == "__main__":
    main()

