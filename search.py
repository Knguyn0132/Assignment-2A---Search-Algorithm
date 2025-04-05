# Modification to search.py
import sys
from search_graph import SearchGraph
from search_selector import SearchSelector

def main():
    if len(sys.argv) != 3:
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
        
        # Special handling for GMGS
        if method == "ABMGS" and hasattr(search_algorithm, 'goals_visited_order'):
            print(f"Total nodes expanded: {nodes_expanded}")
            print(f"Goals visited in order: {' -> '.join(map(str, search_algorithm.goals_visited_order))}")
            print(f"Complete path: {' -> '.join(map(str, path))}")
            print(f"Total cost: {search_algorithm.total_cost}")
            
            # Display detailed segment information
            print("\nSegment details:")
            for i, segment in enumerate(search_algorithm.segment_info):
                print(f"Segment {i+1} (to goal {segment['goal']}):")
                print(f"  Nodes expanded: {segment['nodes_expanded']}")
                print(f"  Path length: {segment['path_length']}")
                print(f"  Cost: {segment['cost']}")
        else:
            # Standard output for regular algorithms
            print(f"{goal} {nodes_expanded}")
            print(" -> ".join(map(str, path)))
    else:
        print(f"{filename} {method}")
        print("No solution found.")

if __name__ == "__main__":
    main()