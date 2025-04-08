def generate_massive_testcase(filename, cols=316, rows=316):
    """
    Generate an enormous test case with ~100,000 nodes.
    This creates a grid-like graph with cols*rows + 2 nodes (including origin and destination).
    """
    with open(filename, 'w') as f:
        # Write test case header
        f.write("# Test Case: Enormous-Scale Graph for Performance Metrics\n")
        f.write("# A grid with ~100,000 nodes to comprehensively stress-test search algorithms\n")
        f.write("# Designed to show dramatic differences in runtime and memory usage\n\n")
        
        f.write("Nodes:\n")
        
        # Origin node at bottom
        f.write(f"1: ({cols//2},0)\n\n")
        
        # Generate rows of nodes
        node_id = 2
        for y in range(1, rows+1):
            f.write(f"# Row y={y}\n")
            for x in range(cols):
                f.write(f"{node_id}: ({x},{y})\n")
                node_id += 1
            f.write("\n")
        
        # Destination node at top
        f.write(f"# Destination node\n")
        f.write(f"{node_id}: ({cols//2},{rows+1})\n\n")
        
        f.write("Edges:\n")
        
        # Connect origin to first row with increasing costs from center
        center = cols // 2
        f.write("# Origin to first row\n")
        for x in range(cols):
            cost = abs(x - center) + 1
            f.write(f"(1,{x+2}): {cost}\n")
        f.write("\n")
        
        # Connect each row to the row above it
        node_id = 2
        for y in range(1, rows):
            f.write(f"# Row {y} to row {y+1}\n")
            for x in range(cols):
                from_node = node_id + x
                to_node = from_node + cols
                f.write(f"({from_node},{to_node}): 1\n")
            node_id += cols
            f.write("\n")
        
        # Connect top row to destination with increasing costs from center
        f.write("# Top row to destination\n")
        dest_node = node_id + cols
        for x in range(cols):
            node = node_id + x
            cost = abs(x - center) + 1
            f.write(f"({node},{dest_node}): {cost}\n")
        f.write("\n")
        
        # Origin and destination
        f.write("Origin:\n1\n\n")
        f.write("Destinations:\n")
        f.write(f"{dest_node}")

# Generate a test case with 316 columns and 316 rows (~100,000 nodes)
generate_massive_testcase("enormous_testcase.txt", 316, 316)