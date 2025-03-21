from graph import Graph

def main():
    # Load the graph from a file
    graph = Graph()
    graph.load_from_file("test_graph.txt")

    # Display graph details (for debugging)
    graph.display()

if __name__ == "__main__":
    main()
