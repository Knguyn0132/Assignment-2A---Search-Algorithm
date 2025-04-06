from search_algorithm import SearchAlgorithm
import heapq


class UCS(SearchAlgorithm):
    """
    Uniform Cost Search (UCS) algorithm implementation.

    UCS is a search method that always explores the node with the lowest total cost
    from the start node. It guarantees finding the shortest path in terms of cost.
    """

    def search(self, start, goals):
        """
        Perform Uniform Cost Search to find the lowest-cost path to any goal node.

        :param start: The starting node
        :param goals: A set of goal nodes
        :return: A tuple (goal_node, num_nodes_expanded, path_to_goal)
        """
        # Counter to ensure nodes are processed in the order they are added to the queue
        # when costs are the same (used for tie-breaking)
        insertion_counter = 0

        # Priority queue (frontier) to store nodes to explore.
        # Each entry is a tuple: (total_cost, insertion_order, node_id, path_to_node)
        # - total_cost: The cumulative cost to reach this node (used for sorting).
        # - insertion_order: Ensures nodes with the same cost are processed in the order they were added.
        # - node_id: The current node being explored.
        # - path_to_node: The path taken to reach this node.
        frontier = [(0, insertion_counter, start, [start])]
        heapq.heapify(frontier)  # Ensure the list is a valid priority queue
        insertion_counter += 1

        # Set to keep track of nodes that have already been explored
        explored = set()

        # Counter to track how many nodes have been expanded during the search
        nodes_expanded = 0

        while frontier:
            # Get the node with the lowest total cost from the priority queue
            # If multiple nodes have the same cost, the one added first is processed first
            current_cost, _, current_node, current_path = heapq.heappop(frontier)

            # Skip this node if it has already been explored
            if current_node in explored:
                continue

            # Mark the current node as explored and increment the counter for expanded nodes
            explored.add(current_node)
            nodes_expanded += 1

            # Check if the current node is one of the goal nodes
            if current_node in goals:
                # If a goal is reached, return the goal node, the number of nodes expanded, and the path
                return current_node, nodes_expanded, current_path

            # Get all neighbors of the current node from the graph's adjacency list
            neighbors = self.graph.adjacency_list.get(current_node, [])

            # Process each neighbor
            for neighbor, edge_cost in sorted(neighbors, key=lambda x: x[0]):  # Sort neighbors by node ID
                if neighbor not in explored:
                    # Calculate the total cost to reach this neighbor
                    new_cost = current_cost + edge_cost
                    # Create the updated path to this neighbor
                    new_path = current_path + [neighbor]

                    # Add the neighbor to the priority queue with its cost and path
                    heapq.heappush(frontier, (new_cost, insertion_counter, neighbor, new_path))
                    # Increment the insertion counter for tie-breaking
                    insertion_counter += 1

        # If no path to any goal is found, return None
        return None, nodes_expanded, []