from search_algorithm import SearchAlgorithm
import math
import heapq


class Fringe(SearchAlgorithm):
    """
    Fringe Search implementation.

    Fringe Search is an informed search algorithm that combines attributes of A* and IDA*
    to minimize node expansions while still finding optimal paths.
    """

    def __init__(self, graph):
        """Initialize with a graph"""
        super().__init__(graph)
        # Debug the graph structure to verify edges
        # print("Graph structure:", self.graph.adjacency_list)

    def heuristic(self, node, goal):
        """Calculate Euclidean distance heuristic between two nodes"""
        if node not in self.graph.node_coordinates or goal not in self.graph.node_coordinates:
            return float('inf')

        x1, y1 = self.graph.node_coordinates[node]
        x2, y2 = self.graph.node_coordinates[goal]

        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_min_heuristic(self, node, goals):
        """Get minimum heuristic to any goal"""
        return min(self.heuristic(node, goal) for goal in goals)

    def search(self, start, goals):
        """Execute Fringe Search from start to any goal."""
        # Convert goals to set for O(1) lookup
        goals = set(goals)

        # If start is already a goal, return immediately
        if start in goals:
            return start, 1, [start]

        nodes_expanded = 0

        # Create a priority queue-based implementation for better sorting
        counter = 0
        open_set = []  # Using a list with heapq operations

        # Initial f-limit based on start node heuristic
        f_limit = self.get_min_heuristic(start, goals)

        # Entry format: (f, g, counter, node, path)
        heapq.heappush(open_set, (f_limit, 0, counter, start, [start]))
        counter += 1

        # Nodes that exceed current f_limit go here
        later = []

        # Track best g-value for each node
        best_g = {start: 0}

        while open_set or later:
            # If open_set is empty, update f_limit and restore nodes from later
            if not open_set:
                if not later:
                    # No path found
                    return None, nodes_expanded, []

                # Find minimum f in later list to set as new f_limit
                min_f = float('inf')
                for f, _, _, _, _ in later:
                    min_f = min(min_f, f)

                f_limit = min_f

                # Move all nodes from later to open_set
                open_set = later
                heapq.heapify(open_set)
                later = []
                continue

            # Get node with lowest f-value
            f, g, _, node, path = heapq.heappop(open_set)
            nodes_expanded += 1

            # Check if goal reached
            if node in goals:
                return node, nodes_expanded, path

            # If f exceeds f_limit, add to later list
            if f > f_limit:
                later.append((f, g, counter, node, path))
                counter += 1
                continue

            # Expand neighbors
            for neighbor, cost in sorted(self.graph.adjacency_list.get(node, [])):
                # Calculate new path cost
                new_g = g + cost

                # Skip if we already have a better path to this neighbor
                if neighbor in best_g and best_g[neighbor] <= new_g:
                    continue

                # Update best path to this neighbor
                best_g[neighbor] = new_g

                # Calculate f-value (g + h)
                h = self.get_min_heuristic(neighbor, goals)
                new_f = new_g + h

                # Create new path
                new_path = path + [neighbor]

                # Insert into appropriate list
                if new_f > f_limit:
                    later.append((new_f, new_g, counter, neighbor, new_path))
                else:
                    heapq.heappush(open_set, (new_f, new_g, counter, neighbor, new_path))

                counter += 1

        # No solution found
        return None, nodes_expanded, []