import networkx as nx
import random
import itertools
import pygame
import numpy as np


def generate_hypercube_graph(n):
    """
    Generates an n-dimensional hypercube graph with nodes labeled as n-bit tuples.
    """
    G = nx.Graph()
    nodes = list(itertools.product([0, 1], repeat=n))
    G.add_nodes_from(nodes)
    for node in nodes:
        for i in range(n):
            neighbor = list(node)
            neighbor[i] ^= 1  # Flip bit at position i
            neighbor = tuple(neighbor)
            G.add_edge(node, neighbor)
    return G


def generate_maze(G):
    """
    Generates a maze from the given graph using Prim's algorithm for minimum spanning tree.
    """
    # Assign random weights to each edge
    for u, v in G.edges():
        G.edges[u, v]["weight"] = random.random()
    # Compute the minimum spanning tree using Prim's algorithm
    T = nx.minimum_spanning_tree(G, algorithm="prim")
    return T


def precompute_node_positions(maze, screen_size):
    """
    Computes positions for all nodes using a suitable layout algorithm.
    """
    # Use spring layout for a better spread of nodes
    pos = nx.spring_layout(
        maze,
        scale=min(screen_size) - 50,
        center=(0, 0),  # Center positions around (0, 0) for easier camera calculations
        iterations=50,
    )
    return pos


def play_maze_pygame(maze, start, goal, screen_size):
    """
    Runs the maze game using Pygame, allowing the user to navigate from the start to the goal node.
    """
    pygame.init()

    # Set up the display
    screen = pygame.display.set_mode(screen_size)
    pygame.display.set_caption("Hypercube Maze Game")

    font = pygame.font.SysFont(None, 24)

    clock = pygame.time.Clock()

    current_node = start

    node_radius = 15  # Increase radius for easier clicking
    center_x = screen_size[0] // 2
    center_y = screen_size[1] // 2

    visited_nodes = set()

    # Precompute positions
    all_pos = precompute_node_positions(maze, screen_size)

    # Initialize camera position centered on the current node
    camera_target = all_pos[current_node]
    camera_position = np.array(camera_target)

    # Animation settings
    animation_in_progress = False
    animation_duration = 0.5  # Duration of the camera movement animation in seconds
    animation_progress = 0.0
    camera_start_position = np.array(camera_target)
    camera_end_position = np.array(camera_target)

    # Game loop
    running = True
    true_maze= maze
    while running:
        maze = nx.ego_graph(true_maze, current_node, radius=1)
        curr_nodes = maze.nodes()

        pos = {node: all_pos[node] for node in curr_nodes}
        delta_time = clock.get_time() / 1000.0  # Convert milliseconds to seconds
        screen.fill((255, 255, 255))  # Clear the screen with white background

        visited_nodes.add(current_node)

        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN and not animation_in_progress:
                mouse_pos = event.pos
                # Adjust mouse position relative to camera
                adjusted_mouse_pos = (
                    mouse_pos[0] - center_x + camera_position[0],
                    mouse_pos[1] - center_y + camera_position[1],
                )
                # Check if user clicked on neighbor
                neighbors = list(maze.neighbors(current_node))
                moved = False
                for neighbor in neighbors:
                    x, y = pos[neighbor]
                    distance = (
                        (adjusted_mouse_pos[0] - x) ** 2
                        + (adjusted_mouse_pos[1] - y) ** 2
                    ) ** 0.5
                    if distance <= node_radius:
                        # Start camera movement animation
                        animation_in_progress = True
                        animation_progress = 0.0
                        camera_start_position = camera_position.copy()
                        camera_end_position = np.array(pos[neighbor])
                        current_node = neighbor
                        print(f"Moved to {current_node}")
                        moved = True
                        break
                if not moved:
                    print("Clicked outside neighbor nodes.")

        # Update camera animation
        if animation_in_progress:
            animation_progress += delta_time
            t = min(animation_progress / animation_duration, 1.0)
            # Smoothstep interpolation for smoother animation
            t_smooth = t * t * (3 - 2 * t)
            camera_position = (
                1 - t_smooth
            ) * camera_start_position + t_smooth * camera_end_position
            # End of animation
            if t >= 1.0:
                animation_in_progress = False
        def should_highlight(current_node, u, v):
            """
            Returns True if the edge (u, v) should be highlighted.
            """
            if u == current_node or v == current_node:
                return True
            return False

        # Draw edges (lines)
        for u, v in maze.edges():

            x1, y1 = pos[u]
            x2, y2 = pos[v]
            # Adjust positions relative to camera
            screen_x1 = center_x + x1 - camera_position[0]
            screen_y1 = center_y + y1 - camera_position[1]
            screen_x2 = center_x + x2 - camera_position[0]
            screen_y2 = center_y + y2 - camera_position[1]
            pygame.draw.line(
                screen,
                (200, 200, 200),
                # (0, 0, 0),
                (screen_x1, screen_y1),
                (screen_x2, screen_y2),
                2 if should_highlight(current_node, u,v) else 1,
            )

        # Draw nodes
        for node in pos:
            x, y = pos[node]
            # Adjust positions relative to camera
            screen_x = center_x + x - camera_position[0]
            screen_y = center_y + y - camera_position[1]
            x_int, y_int = int(screen_x), int(screen_y)

            # Draw only if within the screen
            if 0 <= x_int <= screen_size[0] and 0 <= y_int <= screen_size[1]:
                if node == current_node:
                    color = (255, 0, 0)  # Red for current node
                elif node == goal:
                    color = (255, 165, 0)  # Orange for goal node
                else:
                    if node in visited_nodes:
                        color = (0, 128, 0)  # Green for visited nodes
                    else:
                        color = (0, 0, 255)  # Blue for unvisited nodes
                falpha=0.9
                alpha = int(255 * falpha)
                pygame.draw.circle(
                    screen, color + (alpha,), (x_int, y_int), node_radius
                )
                # Display node labels
                node_label = "".join(map(str, node))
                label_text = font.render(node_label, True, (0, 0, 0))
                screen.blit(
                    label_text, (x_int - node_radius - 5, y_int - node_radius - 5)
                )

        # Draw texts
        goal_label = "".join(map(str, goal))
        goal_text = font.render(f"Goal: {goal_label}", True, (0, 0, 0))
        screen.blit(goal_text, (10, 10))

        current_label = "".join(map(str, current_node))
        current_text = font.render(f"Current: {current_label}", True, (0, 0, 0))
        screen.blit(current_text, (10, 30))

        # Instructions
        instr_text = font.render(
            "Click on a neighboring node to move.", True, (0, 0, 0)
        )
        screen.blit(instr_text, (10, 50))

        if current_node == goal:
            # Display winning message
            win_text = font.render(
                "Congratulations! You have reached the goal!", True, (0, 255, 0)
            )
            screen.blit(
                win_text,
                (screen_size[0] // 2 - win_text.get_width() // 2, screen_size[1] // 2),
            )
            pygame.display.flip()
            pygame.time.delay(3000)
            running = False

        pygame.display.flip()
        clock.tick(60)  # Set frame rate

    pygame.quit()


if __name__ == "__main__":
    # Dimension of the hypercube (set to a reasonable value for visualization)
    n = 5  # This creates a graph with 32 nodes

    G = generate_hypercube_graph(n)
    maze = generate_maze(G)

    # Start at the node with all zeros
    start = tuple(0 for _ in range(n))
    # Choose a goal node (all ones)
    goal = tuple(1 for _ in range(n))

    print("Welcome to the Hypercube Maze Game!")

    # Screen size for Pygame window
    screen_size = (800, 600)

    # Start the game
    play_maze_pygame(maze, start, goal, screen_size)
