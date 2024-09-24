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


def layout_nodes(maze, current_node, center_x, center_y):
    """
    Computes positions for nodes for visualization, using the spring layout algorithm.
    """
    # Set the position of the current node

    # Get the subgraph of nodes within 1 step of the current node
    subgraph = nx.ego_graph(maze, current_node, radius=1)

    # Use spring layout with the current node fixed
    pos = nx.planar_layout(
        subgraph,

        center=(center_x, center_y),
        scale=250,
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

    # Game loop
    running = True
    while running:
        screen.fill((255, 255, 255))  # Clear the screen with white background

        visited_nodes.add(current_node)

        # Get neighbors
        neighbors = list(maze.neighbors(current_node))

        # Compute positions for nodes
        pos = layout_nodes(
            maze, current_node, center_x, center_y
        )

        # Draw edges (lines)
        for neighbor in neighbors:
            if neighbor in pos:
                pygame.draw.line(screen, (0, 0, 0), pos[current_node], pos[neighbor], 2)

        # Draw nodes
        for node in pos:
            x, y = pos[node]
            x_int, y_int = int(x), int(y)
            if node == current_node:
                color = (255, 0, 0)  # Red for current node
            else:
                if node in visited_nodes:
                    color = (0, 128, 0)  # Green for visited neighbors
                else:
                    color = (0, 0, 255)  # Blue for unvisited neighbors
            pygame.draw.circle(screen, color, (x_int, y_int), node_radius)
            # Display node labels
            node_label = ''.join(map(str, node))
            label_text = font.render(node_label, True, (0, 0, 0))
            screen.blit(label_text, (x_int - node_radius-5, y_int - node_radius-5))

        # Draw texts
        goal_label = ''.join(map(str, goal))
        goal_text = font.render(f"Goal: {goal_label}", True, (0, 0, 0))
        screen.blit(goal_text, (10, 10))

        current_label = ''.join(map(str, current_node))
        current_text = font.render(f"Current: {current_label}", True, (0, 0, 0))
        screen.blit(current_text, (10, 30))

        # Instructions
        instr_text = font.render(
            "Click on a neighboring node to move.", True, (0, 0, 0)
        )
        screen.blit(instr_text, (10, 50))

        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = event.pos
                # Check if user clicked on neighbor
                moved = False
                for neighbor in neighbors:
                    if neighbor in pos:
                        x, y = pos[neighbor]
                        distance = (
                            (mouse_pos[0] - x) ** 2 + (mouse_pos[1] - y) ** 2
                        ) ** 0.5
                        if distance <= node_radius:
                            current_node = neighbor
                            print(f"Moved to {current_node}")
                            moved = True
                            break
                if not moved:
                    print("Clicked outside neighbor nodes.")

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
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    # Dimension of the hypercube (set to a reasonable value for visualization)
    n = 5  # This creates a graph with 16 nodes

    G = generate_hypercube_graph(n)
    maze = generate_maze(G)

    # Start at the node with all zeros
    start = tuple(0 for _ in range(n))
    # Choose a random goal node different from the start
    reachable_nodes = nx.node_connected_component(maze, start)
    goal = tuple(1 for _ in range(n))

    print("Welcome to the Hypercube Maze Game!")

    # Screen size for Pygame window
    screen_size = (2*800, 2*600)

    # Start the game
    play_maze_pygame(maze, start, goal, screen_size)
