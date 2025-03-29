from search_algorithm import SearchAlgorithm
import heapq


class UCS(SearchAlgorithm):
    """
    Uniform Cost Search (UCS) algorithm implementation.

    UCS is an uninformed search method that always expands the node with the lowest
    cumulative cost from the start node. It's essentially Dijkstra's algorithm
    when searching for any goal in a graph.
    """

    def search(self, start, goals):
        """
        Perform Uniform Cost Search to find the lowest-cost path to any goal.

        :param start: The starting node
        :param goals: A set of goal nodes
        :return: (goal_node, num_nodes_expanded, path_to_goal)
        """
        # Priority queue to store (cost, node, path)
        frontier = [(0, start, [start])]

        # Set to keep track of explored nodes to avoid revisiting
        explored = set()

        # Track number of nodes expanded
        nodes_expanded = 0

        while frontier:
            # Get the node with the lowest cumulative cost
            current_cost, current_node, current_path = heapq.heappop(frontier)

            # Skip if we've already explored this node
            if current_node in explored:
                continue

            # Mark node as explored
            explored.add(current_node)
            nodes_expanded += 1

            # Check if current node is a goal
            if current_node in goals:
                return current_node, nodes_expanded, current_path

            # Explore neighbors
            if current_node in self.graph.adjacency_list:
                for neighbor, edge_cost in self.graph.adjacency_list[current_node]:
                    # Skip already explored nodes
                    if neighbor not in explored:
                        # Create new path and calculate new cost
                        new_path = current_path + [neighbor]
                        new_cost = current_cost + edge_cost

                        # Add to frontier
                        heapq.heappush(frontier, (new_cost, neighbor, new_path))

        # No path found
        return None, nodes_expanded, []