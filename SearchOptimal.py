import heapq
import math
import os
from collections import deque
import time  # For tracking execution time

def read_maze(filename):
    
    """ Reads a maze from a file and locates the start and goal positions.
    
    I went with this simple format because it's easy to visualize in text files.
    The maze uses these characters:
    - 'P': Player starting position (there should be exactly one)
    - '.': Goal positions (there can be multiple)
    - '%': Walls that can't be traversed
    - ' ': Empty spaces that can be walked through"""
    
    try:
        with open(filename, 'r') as file:
            maze = [list(line.strip()) for line in file]
        
        # Find the start and all goal positions
        start, goals = None, set()
        for i, row in enumerate(maze):
            for j, cell in enumerate(row):
                if cell == 'P':  # Player start position
                    if start is not None:
                        print("Warning: Multiple start positions found! Using the last one.")
                    start = (i, j)
                elif cell == '.':  # Goal position
                    goals.add((i, j))
        
        # Make sure we found what we need
        if start is None:
            raise ValueError("No start position 'P' found in the maze!")
        if not goals:
            raise ValueError("No goal positions '.' found in the maze!")
            
        print(f"Loaded maze with dimensions {len(maze)}x{len(maze[0])}")
        print(f"Start position: {start}")
        print(f"Found {len(goals)} goal positions to visit")
        
        return maze, start, goals
        
    except FileNotFoundError:
        print(f"Couldn't find maze file: {filename}")
        raise
    except Exception as e:
        print(f"Error reading maze: {e}")
        raise


# Distance calculation functions - tried both to see which works better
def manhattan_distance(p1, p2):
       
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def euclidean_distance(p1, p2):
    
    """Calculate the Euclidean (straight-line) distance between two points."""
  
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def bfs_multi_goal(maze, start, goals):
    """
    Breadth-First Search to find the shortest path that visits all goals."""
    # Initialize the BFS queue with starting state
    queue = deque([(start, frozenset(goals), 0)])
    visited = set()  # Track visited states to avoid cycles
    
    # Expanded nodes counter - curious how this compares to A*
    expanded = 0
    max_queue_size = 1
    
    print("Starting BFS search for multi-goal path...")
    start_time = time.time()
    
    # BFS main loop
    while queue:
        max_queue_size = max(max_queue_size, len(queue))
        pos, remaining_goals, cost = queue.popleft()
        expanded += 1
        
        # Skip if we've already seen this state
        if (pos, remaining_goals) in visited:
            continue
            
        # Mark this state as visited
        visited.add((pos, remaining_goals))
        
        # Check if we're done (all goals visited)
        if not remaining_goals:
            elapsed = time.time() - start_time
            print(f"BFS completed! Expanded {expanded} nodes. Max queue size: {max_queue_size}")
            print(f"Took {elapsed:.4f} seconds")
            return cost
            
        # Try all four directions (up, down, left, right)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_pos = (pos[0] + dx, pos[1] + dy)
            
            # Only move to valid positions (inside maze and not a wall)
            # This could crash with IndexError if new_pos is outside the maze - should add bounds check
            if 0 <= new_pos[0] < len(maze) and 0 <= new_pos[1] < len(maze[0]) and maze[new_pos[0]][new_pos[1]] != '%':
                # If this new position is a goal, remove it from the remaining goals
                new_goals = remaining_goals - {new_pos} if new_pos in remaining_goals else remaining_goals
                queue.append((new_pos, new_goals, cost + 1))
    
    # If we get here, there's no solution
    print("BFS couldn't find a path visiting all goals!")
    return float('inf')


def astar_multi_goal(maze, start, goals, heuristic='manhattan'):
    
    # A* search to find the shortest path that visits all goals.
    
    # Similar to BFS but uses a priority queue based on f = g + h where:
    # - g = cost so far
    # - h = heuristic estimate of remaining cost
    
    # Heuristic options:
    # - 'manhattan': Manhattan distance to the nearest unvisited goal
    # - 'euclidean': Euclidean distance to the nearest unvisited goal
    
    # Create inner function for heuristic calculation
    def heuristic_func(pos, remaining_goals):
        """Calculates the heuristic value based on the specified method"""
        if not remaining_goals:
            return 0
            
        if heuristic == 'manhattan':
            # Distance to the closest remaining goal
            return min(manhattan_distance(pos, g) for g in remaining_goals)
        elif heuristic == 'euclidean':
            return min(euclidean_distance(pos, g) for g in remaining_goals)
        else:
            # Fall back to Manhattan if an invalid heuristic is specified
            print(f"Warning: Unknown heuristic '{heuristic}', using manhattan instead")
            return min(manhattan_distance(pos, g) for g in remaining_goals)
    
    # Initialize the priority queue with the starting state
    # Format: (f_value, position, remaining_goals, cost_so_far)
    priority_queue = []
    heapq.heappush(priority_queue, (0, start, frozenset(goals), 0))
    
    # Track visited states and their costs
    visited = {}  # Using a dict to store cost for each state
    
    # Performance tracking
    expanded = 0
    max_queue_size = 1
    
    print(f"Starting A* search with {heuristic} heuristic...")
    start_time = time.time()
    
    # A* main loop
    while priority_queue:
        max_queue_size = max(max_queue_size, len(priority_queue))
        f, pos, remaining_goals, cost = heapq.heappop(priority_queue)
        expanded += 1
        
        # Skip if we've already found a better path to this state
        if (pos, remaining_goals) in visited and visited[(pos, remaining_goals)] <= cost:
            continue
            
        # Record the best cost to reach this state
        visited[(pos, remaining_goals)] = cost
        
        # Check if we're done (all goals visited)
        if not remaining_goals:
            elapsed = time.time() - start_time
            print(f"A* ({heuristic}) completed! Expanded {expanded} nodes. Max queue size: {max_queue_size}")
            print(f"Took {elapsed:.4f} seconds")
            return cost
            
        # Try all four directions (up, down, left, right)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_pos = (pos[0] + dx, pos[1] + dy)
            
            # Check bounds first to avoid index errors
            if 0 <= new_pos[0] < len(maze) and 0 <= new_pos[1] < len(maze[0]) and maze[new_pos[0]][new_pos[1]] != '%':
                # If this new position is a goal, remove it from remaining goals
                new_goals = remaining_goals - {new_pos} if new_pos in remaining_goals else remaining_goals
                
                # Calculate new g and h values
                g = cost + 1  # Each step costs 1
                h = heuristic_func(new_pos, new_goals)
                
                # Only add to the queue if it's a promising path
                # This could be optimized further with additional checks
                heapq.heappush(priority_queue, (g + h, new_pos, new_goals, g))
    
    # If we get here, there's no solution
    print(f"A* with {heuristic} heuristic couldn't find a path visiting all goals!")
    return float('inf')


def compare_methods(filename):
    """
    Runs and compares different pathfinding methods on the same maze.
    
    This is helpful for analyzing which algorithm performs better in different scenarios.
    My hypothesis is that A* should typically be more efficient than BFS for most mazes,
    especially larger ones with multiple goals.
    """
    print(f"\n{'='*50}")
    print(f"Testing maze: {filename}")
    print(f"{'='*50}")
    
    # Load the maze
    maze, start, goals = read_maze(filename)
    
    # Run all three methods and time them
    print("\nRunning BFS...")
    bfs_cost = bfs_multi_goal(maze, start, goals)
    
    print("\nRunning A* with Manhattan distance...")
    astar_manhattan = astar_multi_goal(maze, start, goals, 'manhattan')
    
    print("\nRunning A* with Euclidean distance...")
    astar_euclidean = astar_multi_goal(maze, start, goals, 'euclidean')
    
    # Print results
    print("\n"+"-"*50)
    print("Results Summary:")
    print("-"*50)
    print(f"BFS Cost: {bfs_cost}")
    print(f"A* Manhattan Cost: {astar_manhattan}")
    print(f"A* Euclidean Cost: {astar_euclidean}")
    
    # Compare results
    if bfs_cost == astar_manhattan == astar_euclidean:
        print("\nAll methods found paths of the same length. This is expected for optimal algorithms.")
    else:
        print("\nWARNING: Different path costs found! This suggests a bug in one of the implementations.")
        print("BFS should always find the optimal path, so any A* result that differs is suspect.")
    
    min_cost = min(bfs_cost, astar_manhattan, astar_euclidean)
    winners = []
    if bfs_cost == min_cost:
        winners.append("BFS")
    if astar_manhattan == min_cost:
        winners.append("A* Manhattan")
    if astar_euclidean == min_cost:
        winners.append("A* Euclidean")
    
    print(f"\nOptimal solution(s): {', '.join(winners)} with cost {min_cost}")


if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(base_dir, "Maze", "smallSearch.lay")

    
    try:
        compare_methods(filename)
    except Exception as e:
        print(f"Error running comparison: {e}")
        
    print("\nDone! Check the results above to see which method performed best.")
    