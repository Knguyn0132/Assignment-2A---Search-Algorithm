import heapq
import math

class BidirectionalSearch:
    def __init__(self, nodes, edges):
        """
        Initialize the bidirectional search with nodes and edges.
        
        :param nodes: Dictionary mapping node ID to (x,y) coordinates
        :param edges: Dictionary mapping (from_node, to_node) to edge cost
        """
        self.nodes = nodes
        self.edges = edges
        
        # Build forward adjacency list
        self.forward_adj = {}
        for (from_node, to_node), cost in edges.items():
            if from_node not in self.forward_adj:
                self.forward_adj[from_node] = []
            self.forward_adj[from_node].append((to_node, cost))
        
        # Build backward adjacency list
        self.backward_adj = {}
        for (from_node, to_node), cost in edges.items():
            if to_node not in self.backward_adj:
                self.backward_adj[to_node] = []
            self.backward_adj[to_node].append((from_node, cost))
    
    def euclidean_distance(self, node1, node2):
        """
        Calculate Euclidean distance between two nodes using their coordinates.
        """
        x1, y1 = self.nodes[node1]
        x2, y2 = self.nodes[node2]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def search(self, start, goal):
        """
        Execute bidirectional search from start node to goal node.
        
        :param start: Starting node ID
        :param goal: Goal node ID
        :return: (nodes_expanded, path, cost)
        """
        # Forward search data structures
        forward_g = {start: 0}
        forward_open = [(self.euclidean_distance(start, goal), start, [start])]
        forward_closed = set()
        
        # Backward search data structures
        backward_g = {goal: 0}
        backward_open = [(self.euclidean_distance(goal, start), goal, [goal])]
        backward_closed = set()
        
        # Counters and best path tracking
        nodes_expanded = 0
        best_path = None
        best_cost = float('inf')
        
        while forward_open and backward_open:
            # FORWARD SEARCH PHASE
            if forward_open:
                # Pop node with lowest f-score from forward open list
                _, current_forward, path_forward = heapq.heappop(forward_open)
                
                # Skip if already visited
                if current_forward in forward_closed:
                    continue
                
                # Mark as visited and count as expanded
                forward_closed.add(current_forward)
                nodes_expanded += 1
                
                # Check if we've reached a node in the backward closed set
                # This means we have a potential path
                if current_forward in backward_closed:
                    # Find matching backward path
                    for _, node, path in backward_open:
                        if node == current_forward:
                            backward_path = path
                            break
                    else:
                        # Search in closed nodes if not found in open
                        for f_score, node, path in backward_open:
                            if node == current_forward:
                                backward_path = path
                                break
                    
                    # Calculate total path cost
                    total_cost = forward_g[current_forward] + backward_g[current_forward]
                    
                    # Update if better than current best
                    if total_cost < best_cost:
                        best_cost = total_cost
                        # Combine forward path with reversed backward path (excluding duplicate node)
                        best_path = path_forward + backward_path[1:][::-1]
                
                # Expand forward neighbors
                for neighbor, cost in self.forward_adj.get(current_forward, []):
                    new_g_score = forward_g[current_forward] + cost
                    
                    if neighbor not in forward_g or new_g_score < forward_g[neighbor]:
                        forward_g[neighbor] = new_g_score
                        f_score = new_g_score + self.euclidean_distance(neighbor, goal)
                        new_path = path_forward + [neighbor]
                        heapq.heappush(forward_open, (f_score, neighbor, new_path))
            
            # BACKWARD SEARCH PHASE
            if backward_open:
                # Pop node with lowest f-score from backward open list
                _, current_backward, path_backward = heapq.heappop(backward_open)
                
                # Skip if already visited
                if current_backward in backward_closed:
                    continue
                
                # Mark as visited and count as expanded
                backward_closed.add(current_backward)
                nodes_expanded += 1
                
                # Check if we've reached a node in the forward closed set
                if current_backward in forward_closed:
                    # Find matching forward path
                    for _, node, path in forward_open:
                        if node == current_backward:
                            forward_path = path
                            break
                    else:
                        # Search in closed nodes if not found in open
                        for f_score, node, path in forward_open:
                            if node == current_backward:
                                forward_path = path
                                break
                    
                    # Calculate total path cost
                    total_cost = forward_g[current_backward] + backward_g[current_backward]
                    
                    # Update if better than current best
                    if total_cost < best_cost:
                        best_cost = total_cost
                        # Combine forward path with reversed backward path (excluding duplicate node)
                        best_path = forward_path + path_backward[1:][::-1]
                
                # Expand backward neighbors
                for neighbor, cost in self.backward_adj.get(current_backward, []):
                    new_g_score = backward_g[current_backward] + cost
                    
                    if neighbor not in backward_g or new_g_score < backward_g[neighbor]:
                        backward_g[neighbor] = new_g_score
                        f_score = new_g_score + self.euclidean_distance(neighbor, start)
                        new_path = path_backward + [neighbor]
                        heapq.heappush(backward_open, (f_score, neighbor, new_path))
            
            # Termination check - if the best path cost is better than any possible path via expanding
            min_forward = min([f for f, _, _ in forward_open]) if forward_open else float('inf')
            min_backward = min([f for f, _, _ in backward_open]) if backward_open else float('inf')
            
            if best_path and best_cost <= min_forward + min_backward:
                return nodes_expanded, best_path, best_cost
        
        # Return best path found (or empty if none)
        return nodes_expanded, best_path or [], best_cost
    #end


