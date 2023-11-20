import pygame
import heapq
import timeit
from sys import getsizeof
import matplotlib.pyplot as plt
from collections import deque

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0
        self.h = 0
        self.parent = None

    def __lt__(self, other):
        return (self.g + self.h) < (other.g + other.h)

def find_path_dijkstra(maze, start, goal):
    open_set = []
    closed_set = set()

    start_node = Node(*start)
    goal_node = Node(*goal)

    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add((current_node.x, current_node.y))

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dx, dy in neighbors:
            neighbor = Node(current_node.x + dx, current_node.y + dy)
            if (
                0 <= neighbor.x < len(maze)
                and 0 <= neighbor.y < len(maze[0])
                and maze[neighbor.x][neighbor.y] == 0
                and (neighbor.x, neighbor.y) not in closed_set
            ):
                neighbor.g = current_node.g + 1
                neighbor.parent = current_node
                heapq.heappush(open_set, neighbor)

    return None

def find_path_astar(maze, start, goal):
    open_set = []
    closed_set = set()

    start_node = Node(*start)
    goal_node = Node(*goal)

    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add((current_node.x, current_node.y))

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dx, dy in neighbors:
            neighbor = Node(current_node.x + dx, current_node.y + dy)
            if (
                0 <= neighbor.x < len(maze)
                and 0 <= neighbor.y < len(maze[0])
                and maze[neighbor.x][neighbor.y] == 0
                and (neighbor.x, neighbor.y) not in closed_set
            ):
                neighbor.g = current_node.g + 1
                neighbor.h = heuristic(neighbor, goal)
                neighbor.parent = current_node
                heapq.heappush(open_set, neighbor)

    return None

def find_path_greedy_best_first(maze, start, goal):
    open_set = []
    closed_set = set()

    start_node = Node(*start)
    goal_node = Node(*goal)

    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add((current_node.x, current_node.y))

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dx, dy in neighbors:
            neighbor = Node(current_node.x + dx, current_node.y + dy)
            if (
                0 <= neighbor.x < len(maze)
                and 0 <= neighbor.y < len(maze[0])
                and maze[neighbor.x][neighbor.y] == 0
                and (neighbor.x, neighbor.y) not in closed_set
            ):
                neighbor.h = heuristic(neighbor, goal)
                neighbor.parent = current_node
                heapq.heappush(open_set, neighbor)

    return None

def find_path_bfs(maze, start, goal):
    queue = deque()
    visited = set()

    start_node = Node(*start)
    goal_node = Node(*goal)

    queue.append(start_node)

    while queue:
        current_node = queue.popleft()

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        visited.add((current_node.x, current_node.y))

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dx, dy in neighbors:
            neighbor = Node(current_node.x + dx, current_node.y + dy)
            if (
                0 <= neighbor.x < len(maze)
                and 0 <= neighbor.y < len(maze[0])
                and maze[neighbor.x][neighbor.y] == 0
                and (neighbor.x, neighbor.y) not in visited
            ):
                neighbor.parent = current_node
                queue.append(neighbor)

    return None

def find_path_dfs(maze, start, goal):
    stack = []
    visited = set()

    start_node = Node(*start)
    goal_node = Node(*goal)

    stack.append(start_node)

    while stack:
        current_node = stack.pop()

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        visited.add((current_node.x, current_node.y))

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dx, dy in neighbors:
            neighbor = Node(current_node.x + dx, current_node.y + dy)
            if (
                0 <= neighbor.x < len(maze)
                and 0 <= neighbor.y < len(maze[0])
                and maze[neighbor.x][neighbor.y] == 0
                and (neighbor.x, neighbor.y) not in visited
            ):
                neighbor.parent = current_node
                stack.append(neighbor)

    return None

def heuristic(node, goal):
    return abs(node.x - goal[0]) + abs(node.y - goal[1])

def draw_maze_with_path(screen, maze, path=None):
    screen.fill((255, 255, 255))

    cell_size = 80
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            rect = pygame.Rect(j * cell_size, i * cell_size, cell_size, cell_size)
            if path and (i, j) in path:
                pygame.draw.rect(screen, (255, 255, 0), rect)
            elif maze[i][j] == 1:
                pygame.draw.rect(screen, (0, 0, 0), rect)
            else:
                pygame.draw.rect(screen, (255, 255, 255), rect)

    pygame.display.flip()

def solve_maze():
    maze = [
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
    ]

    start = (0, 0)
    goal = (9, 9)

    pygame.init()
    screen = pygame.display.set_mode((len(maze[0]) * 80, len(maze) * 80))
    pygame.display.set_caption("Maze Solver")

    # Time Complexity and Space Complexity Evaluation for Dijkstra's Algorithm
    dijkstra_start_time = timeit.default_timer()
    dijkstra_path = find_path_dijkstra(maze, start, goal)
    dijkstra_time = timeit.default_timer() - dijkstra_start_time
    dijkstra_space = getsizeof(dijkstra_path)
    print("Dijkstra's algorithm time:", dijkstra_time, "seconds")
    print("Dijkstra's algorithm space:", dijkstra_space, "bytes")

    # Time Complexity and Space Complexity Evaluation for A* Algorithm
    astar_start_time = timeit.default_timer()
    astar_path = find_path_astar(maze, start, goal)
    astar_time = timeit.default_timer() - astar_start_time
    astar_space = getsizeof(astar_path)
    print("A* algorithm time:", astar_time, "seconds")
    print("A* algorithm space:", astar_space, "bytes")

    # Time Complexity and Space Complexity Evaluation for Greedy Best-First Search Algorithm
    greedy_best_first_start_time = timeit.default_timer()
    greedy_best_first_path = find_path_greedy_best_first(maze, start, goal)
    greedy_best_first_time = timeit.default_timer() - greedy_best_first_start_time
    greedy_best_first_space = getsizeof(greedy_best_first_path)
    print("Greedy Best-First algorithm time:", greedy_best_first_time, "seconds")
    print("Greedy Best-First algorithm space:", greedy_best_first_space, "bytes")

    # Time Complexity and Space Complexity Evaluation for BFS Algorithm
    bfs_start_time = timeit.default_timer()
    bfs_path = find_path_bfs(maze, start, goal)
    bfs_time = timeit.default_timer() - bfs_start_time
    bfs_space = getsizeof(bfs_path)
    print("BFS algorithm time:", bfs_time, "seconds")
    print("BFS algorithm space:", bfs_space, "bytes")

    # Time Complexity and Space Complexity Evaluation for DFS Algorithm
    dfs_start_time = timeit.default_timer()
    dfs_path = find_path_dfs(maze, start, goal)
    dfs_time = timeit.default_timer() - dfs_start_time
    dfs_space = getsizeof(dfs_path)
    print("DFS algorithm time:", dfs_time, "seconds")
    print("DFS algorithm space:", dfs_space, "bytes")

    # Bar Chart for Time Complexity
    labels = ['Dijkstra', 'A*', 'Greedy Best-First', 'BFS', 'DFS']
    times = [dijkstra_time, astar_time, greedy_best_first_time, bfs_time, dfs_time]

    plt.bar(labels, times)
    plt.ylabel('Time (seconds)')
    plt.title('Time Complexity Comparison')
    plt.show()

    # Bar Chart for
    space_complexities = [dijkstra_space, astar_space, greedy_best_first_space, bfs_space, dfs_space]

    plt.bar(labels, space_complexities)
    plt.ylabel('Space (bytes)')
    plt.title('Space Complexity Comparison')
    plt.show()

    current_path = dijkstra_path


    running = True
    algorithm_index = 0

    try:
        while running:
            # Draw the maze and path
            if algorithm_index == 0:
                current_path = dijkstra_path
            elif algorithm_index == 1:
                current_path = astar_path
            elif algorithm_index == 2:
                current_path = greedy_best_first_path
            elif algorithm_index == 3:
                current_path = bfs_path
            elif algorithm_index == 4:
                current_path = dfs_path

            draw_maze_with_path(screen, maze, current_path)

            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        algorithm_index = (algorithm_index + 1) % 5

    except pygame.error:
        pygame.quit()


# Run the maze-solving application
solve_maze()
