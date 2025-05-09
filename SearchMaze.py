import heapq
from collections import deque
import os

# The maze is a grid where:
# '%' = walls 
# 'P' = player starting point 
# '.' = goal/target destination
class Maze:
    def __init__(self, filepath):
        self.grid = []
        self.start = None
        self.goal = None
        self.load_maze(filepath)
        
        # Sanity check - make sure we found both start and end points
        if not self.start or not self.goal:
            raise ValueError("Couldn't find start 'P' or goal '.' in the maze file!")

    def load_maze(self, filepath):
        # Read in the maze from file 
        try:
            with open(filepath, 'r') as f:
                lines = f.readlines()
            
            # Converting each line to a list of characters
            self.grid = [list(line.strip('\n')) for line in lines]
            
            # Find the special cells 
            for y, row in enumerate(self.grid):
                for x, cell in enumerate(row):
                    if cell == 'P':
                        self.start = (x, y) 
                    elif cell == '.':
                        self.goal = (x, y)
                        
        except FileNotFoundError:
            print(f"Darn it, can't find the maze file: {filepath}")
            raise

    def is_wall(self, pos):
        # Checking if the position is a wall
        x, y = pos
        return self.grid[y][x] == '%'

    def in_bounds(self, pos):
        # Making sure we don't go out of bounds 
        x, y = pos
        return 0 <= y < len(self.grid) and 0 <= x < len(self.grid[0])

    def get_neighbors(self, pos):
        # Getting all valid adjacent cells
        x, y = pos
        # Right, left, down, up - the classic 4 directions
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        # Filtering out walls and out-of-bounds positions
        return [n for n in neighbors if self.in_bounds(n) and not self.is_wall(n)]

    def display_path(self, path):
        # Creating a copy so we don't mess up the original grid
        display_grid = [row[:] for row in self.grid]
        
        # Marking the path with 'o' to make it more visible
        # Skipping the start and end points to keep them as is)
        for x, y in path:
            if display_grid[y][x] not in ('P', '.'):
                display_grid[y][x] = 'o'
                
        # Print out the solved maze
        for row in display_grid:
            print(''.join(row))


# breadth-first search
# Tried and true for finding shortest paths when all steps cost the same
def bfs(maze):
    start, goal = maze.start, maze.goal
    frontier = deque([start])  # using deque for efficient popping from front
    came_from = {start: None}  # keep track of where we came from
    nodes_expanded = 0 
    max_depth = 0
    max_fringe = 1  # tracks how wide our search gets
    
    # Start of loop
    while frontier:
        current = frontier.popleft()
        nodes_expanded += 1
        
        # the goal
        if current == goal:
            break
            
        # Checking all neighboring cells
        for neighbor in maze.get_neighbors(current):
            if neighbor not in came_from:  
                frontier.append(neighbor)
                came_from[neighbor] = current
                
        # Keeping track of our search stats
        max_depth = max(max_depth, nodes_expanded)
        max_fringe = max(max_fringe, len(frontier))

    # Couldn't find a path - this shouldn't happen with well-formed mazes
    if goal not in came_from:
        return [], 0, nodes_expanded, max_depth, max_fringe

    # Reconstruct the path from goal to start, then reverse it
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    
    return path, len(path) - 1, nodes_expanded, max_depth, max_fringe


# Manhattan distance
# Just count how many horizontal and vertical steps it would take to get from a to b
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# A* search - BFS's smarter cousin that uses a heuristic to guide the search
# Should be more efficient for most mazes (fingers crossed!)
def astar(maze):
    start, goal = maze.start, maze.goal
    
    # Priority queue based on f(n) = g(n) + h(n)
    # where g(n) is cost so far and h(n) is heuristic estimate
    frontier = [(manhattan(start, goal), 0, start)]  # (f, g, position)
    came_from = {start: None}
    cost_so_far = {start: 0}
    nodes_expanded = 0
    max_depth = 0
    max_fringe = 1

    while frontier:
        # Get the node with lowest f-value
        _, cost, current = heapq.heappop(frontier)
        nodes_expanded += 1
        
     
        if current == goal:
            break
            
        # Check all neighbors
        for neighbor in maze.get_neighbors(current):
            new_cost = cost + 1  # Each step costs 1 in our simple maze
            
            # If we've found a better path to this neighbor
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                # Priority based on f(n) = g(n) + h(n)
                priority = new_cost + manhattan(neighbor, goal)
                heapq.heappush(frontier, (priority, new_cost, neighbor))
                came_from[neighbor] = current
                
        # Track stats for comparison
        max_depth = max(max_depth, nodes_expanded)
        max_fringe = max(max_fringe, len(frontier))

    # No path found
    if goal not in came_from:
        return [], 0, nodes_expanded, max_depth, max_fringe

    # Reconstruct path (same as BFS)
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    
    return path, len(path) - 1, nodes_expanded, max_depth, max_fringe



if __name__ == "__main__":
    # test mazes 
    maze_files = [
        "smallMaze.lay",
        "mediumMaze.lay",
        "bigMaze.lay",
        "openMaze.lay"
    ]

    # Run both algorithms on each maze and compare
    for filename in maze_files:
        print(f"\n{'=' * 10} Running on {filename} {'=' * 10}")
        
        # Make sure the path works regardless of how we run the script
        base_dir = os.path.dirname(os.path.abspath(__file__))
        path_to_file = os.path.join(base_dir, "Maze", filename)
        
        try:
            # First, try BFS
            print("\n→ BFS (always optimal but might be inefficient):")
            maze = Maze(path_to_file)
            path, cost, nodes_exp, max_depth, max_fringe = bfs(maze)
            if path:
                maze.display_path(path)
                print(f"Path cost: {cost} steps")
                print(f"Nodes expanded: {nodes_exp}")
                print(f"Max depth reached: {max_depth}")
                print(f"Max fringe size: {max_fringe}")
            else:
                print("No path found! Is this maze solvable?")

            # Now, try A*
            print("\n→ A* Search (should be more efficient):")
            maze = Maze(path_to_file)  # Fresh maze for A*
            path, cost, nodes_exp, max_depth, max_fringe = astar(maze)
            if path:
                maze.display_path(path)
                print(f"Path cost: {cost} steps")
                print(f"Nodes expanded: {nodes_exp} (hopefully less than BFS!)")
                print(f"Max depth reached: {max_depth}")
                print(f"Max fringe size: {max_fringe}")
            else:
                print("No path found! Weird, BFS should have found one...")
                
        except Exception as e:
            print(f"Oops! Something went wrong with {filename}: {e}")
            # Keep going with other mazes instead of crashing completely
            continue