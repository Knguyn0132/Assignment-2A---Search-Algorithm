from search_algorithm import SearchAlgorithm
import heapq


class UCS(SearchAlgorithm):
    """
    Uniform Cost Search (UCS) algorithm implementation.

    UCS is an uninformed search method that always expands the node with the lowest
    cumulative cost from the start node.
    """

    def search(self, start, goals):
        """
        Perform Uniform Cost Search to find the lowest-cost path to any goal.

        :param start: The starting node
        :param goals: A set of goal nodes
        :return: (goal_node, num_nodes_expanded, path_to_goal)
        """
        # Use a counter to break ties in chronological order when costs are equal
        counter = 0

        # Priority queue to store (cost, node_id, insertion_order, path)
        # - cost: primary sort criterion
        # - node_id: secondary sort criterion (for tie-breaking by node ID)
        # - insertion_order: tertiary criterion to maintain chronological order when both costs and IDs are equal
        frontier = [(0, start, counter, [start])]
        counter += 1

        # Set to keep track of explored nodes to avoid revisiting
        explored = set()

        # Track number of nodes expanded
        nodes_expanded = 0

        while frontier:
            # Get the node with the lowest cost (primary criterion)
            # If costs are equal, the tie is broken by node ID (ascending)
            # If node IDs are equal, the tie is broken by insertion order (chronological)
            current_cost, current_node, _, current_path = heapq.heappop(frontier)

            # Skip if we've already explored this node
            if current_node in explored:
                continue

            # Mark node as explored
            explored.add(current_node)
            nodes_expanded += 1

            # Check if current node is a goal (NOTE 1: reach one of the destination nodes)
            if current_node in goals:
                return current_node, nodes_expanded, current_path

            # Get all neighbors and their costs, then sort by node ID to ensure
            # ascending order processing (NOTE 2)
            neighbors = []
            if current_node in self.graph.adjacency_list:
                neighbors = self.graph.adjacency_list[current_node]

            # Process neighbors in ascending node ID order when adding to frontier
            # This ensures that when costs are equal, lower IDs are expanded first
            for neighbor, edge_cost in sorted(neighbors, key=lambda x: x[0]):
                # Skip already explored nodes
                if neighbor not in explored:
                    # Check if this node is already in the frontier with a higher cost
                    in_frontier_with_higher_cost = False
                    new_cost = current_cost + edge_cost
                    new_path = current_path + [neighbor]

                    # Add to frontier with node ID as secondary criterion for tie-breaking
                    # This ensures that when costs are equal, nodes with smaller IDs are expanded first
                    # Counter is used as the tertiary criterion for chronological ordering
                    heapq.heappush(frontier, (new_cost, neighbor, counter, new_path))
                    counter += 1

        # No path found
        return None, nodes_expanded, []