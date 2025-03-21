class Graph:
    """
    A class to represent a graph for the Route Finding Problem.
    It loads nodes, edges, origin, and destinations from a structured text file.
    """

    def __init__(self):
        """Initializes an empty graph structure."""
        self.graph = {}  # Adjacency list: {node: [(neighbor, cost), ...]}
        self.node_coordinates = {}  # Stores node positions: {node: (x, y)}
        self.origin = None  # The start node
        self.destinations = set()  # Set of goal nodes

    def load_from_file(self, filename):
        """Reads a structured graph file and extracts nodes, edges, origin, and destinations."""
        try:
            with open(filename, 'r') as file:
                section = None  # Tracks which section we are reading

                for line in file:
                    line = line.strip()  # Remove whitespace

                    if not line or line.startswith("#"):  # Ignore empty lines and comments
                        continue

                    # Detect section headers and continue processing the current line
                    new_section = self.read_section(line)
                    if new_section:
                        section = new_section
                        continue  # Move to the next line

                    # Process valid lines within each section
                    if section == "nodes":
                        self.parse_nodes(line)
                    elif section == "edges":
                        self.parse_edges(line)
                    elif section == "origin":
                        self.parse_origin(line)
                    elif section == "destinations":
                        self.parse_destinations(line)

        except FileNotFoundError:
            print(f"Error: File '{filename}' not found.")
            exit(1)

    def read_section(self, line):
        """Detects and updates the section being read in the file."""
        sections = ["Nodes:", "Edges:", "Origin:", "Destinations:"]
        if line in sections:
            return line[:-1].lower()  # Convert "Nodes:" → "nodes"
        return None  # Return None if the line is not a section header

    def parse_nodes(self, line):
        """Parses node coordinates and stores them in a dictionary."""
        try:
            # Ensure the line contains a node ID and coordinates
            if ":" not in line:
                return  # Skip invalid lines

            node_id, coords = line.split(":")
            node_id = int(node_id.strip())  # Convert node ID to integer

            # Ensure coordinates are correctly formatted
            coords = coords.strip().replace("(", "").replace(")", "")
            x, y = map(int, coords.split(","))  # Extract (x, y) as integers

            self.node_coordinates[node_id] = (x, y)  # Store node coordinates
        except ValueError:
            print(f"Warning: Skipping invalid node line: {line}")

    def parse_edges(self, line):
        """Parses edges and stores them in an adjacency list."""
        try:
            if ":" not in line or "(" not in line or ")" not in line:
                return  # Skip invalid lines

            edge_data, cost = line.split(":")
            cost = int(cost.strip())  # Convert cost to integer
            node_a, node_b = map(int, edge_data.strip("()").split(","))  # Extract edge nodes

            # Ensure both nodes exist in the graph
            if node_a not in self.graph:
                self.graph[node_a] = []
            if node_b not in self.graph:
                self.graph[node_b] = []

            self.graph[node_a].append((node_b, cost))
        except ValueError:
            print(f"Warning: Skipping invalid edge line: {line}")

    def parse_origin(self, line):
        """Parses and stores the origin (start node)."""
        try:
            self.origin = int(line.strip())
        except ValueError:
            print(f"Warning: Invalid origin value: {line}")

    def parse_destinations(self, line):
        """Parses and stores the destination nodes."""
        try:
            self.destinations = set(map(int, line.split(";")))  # Convert to a set
        except ValueError:
            print(f"Warning: Invalid destinations format: {line}")

    def display(self):
        """Prints the graph details for debugging."""
        print("Graph (Adjacency List):", self.graph)
        print("Node Coordinates:", self.node_coordinates)
        print("Origin:", self.origin)
        print("Destinations:", self.destinations)


