# Modification to search.py
import sys
import time
import os
import psutil  # You'll need to install this with: pip install psutil
from search_graph import SearchGraph
from search_selector import SearchSelector

def main():
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print("Usage: python search.py <filename> <method> [metrics]")
        exit(1)

    filename = sys.argv[1]
    method = sys.argv[2].upper()
    metrics = False
    
    if len(sys.argv) == 4 and sys.argv[3].lower() == "metrics":
        metrics = True

    # Load the graph
    graph = SearchGraph()
    graph.load_from_file(filename)

    # Get the correct search algorithm instance
    try:
        search_algorithm = SearchSelector.get_search_algorithm(method, graph)
    except ValueError as e:
        print(e)
        exit(1)

    # Start timing and get initial memory usage if metrics requested
    start_time = time.time() if metrics else None
    process = psutil.Process(os.getpid())
    start_memory = process.memory_info().rss / (1024 * 1024) if metrics else None  # Initial memory in MB
    
    # Run the search
    goal, nodes_expanded, path = search_algorithm.search(graph.origin, graph.destinations)

    # Calculate metrics if requested
    runtime = (time.time() - start_time) if metrics else None
    end_memory = process.memory_info().rss / (1024 * 1024) if metrics else None  # Final memory in MB
    memory_used = end_memory - start_memory if metrics else None  # Memory difference in MB

    # Print result
    if goal is not None:
        print(f"{filename} {method}")
        
        # Special handling for ABMGS
        if method == "ABMGS" and hasattr(search_algorithm, 'goals_visited_order'):
            print(f"Total nodes expanded: {nodes_expanded}")
            print(f"Goals visited in order: {str(search_algorithm.goals_visited_order)}")
            print(f"Complete path: {str(path)}")
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
            print(str(path))
        
        # Print metrics if requested
        if metrics:
            print(f"\nPerformance Metrics:")
            print(f"Runtime: {runtime:.6f} seconds")
            print(f"Memory usage: {memory_used:.2f} MB")
    else:
        print(f"{filename} {method}")
        print("No solution found.")

if __name__ == "__main__":
    main()