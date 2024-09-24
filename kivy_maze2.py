# import networkx as nx
# import random
# import itertools
# import numpy as np

# from kivy.app import App
# from kivy.uix.widget import Widget
# from kivy.graphics import Line, Color, Ellipse, Rectangle
# from kivy.properties import ObjectProperty, DictProperty, ListProperty, NumericProperty
# from kivy.clock import Clock
# from kivy.core.window import Window
# from kivy.uix.label import Label

# def generate_hypercube_graph(n):
#     """
#     Generates an n-dimensional hypercube graph with nodes labeled as n-bit tuples.
#     """
#     G = nx.Graph()
#     nodes = list(itertools.product([0, 1], repeat=n))
#     G.add_nodes_from(nodes)
#     for node in nodes:
#         for i in range(n):
#             neighbor = list(node)
#             neighbor[i] ^= 1  # Flip bit at position i
#             neighbor = tuple(neighbor)
#             G.add_edge(node, neighbor)
#     return G

# def generate_maze(G):
#     """
#     Generates a maze from the given graph using Prim's algorithm for minimum spanning tree.
#     """
#     # Assign random weights to each edge
#     for u, v in G.edges():
#         G.edges[u, v]["weight"] = random.random()

#     # Compute the minimum spanning tree using Prim's algorithm
#     T = nx.minimum_spanning_tree(G, algorithm="prim")
#     # Additionally add the hamming distance as an attribute to each edge
#     for u, v in T.edges():
#         hamming_dist = sum([1 for i in range(len(u)) if u[i] != v[i]])
#         T.edges[u, v]["hamming_dist"] = hamming_dist
#     return T

# def precompute_node_positions(maze, screen_size):
#     """
#     Computes positions for all nodes using a suitable layout algorithm.
#     """
#     # Use spring layout for a better spread of nodes
#     pos = nx.spring_layout(
#         maze,
#         scale=min(screen_size) * 2.5,
#         center=(0, 0),  # Center positions around (0, 0)
#         iterations=50,
#         weight="hamming_dist",
#     )
#     return pos

# class MazeGame(Widget):
#     maze = ObjectProperty(None)
#     current_node = ObjectProperty(None)
#     goal = ObjectProperty(None)
#     pos = DictProperty({})
#     all_pos = DictProperty({})
#     node_radius = NumericProperty(20)  # For touch detection
#     visited_nodes = ListProperty([])
#     path_stack = ListProperty([])
#     camera_position = ListProperty([0, 0])  # Camera position to center the current node

#     def __init__(self, **kwargs):
#         super(MazeGame, self).__init__(**kwargs)
#         # Dimension of the hypercube
#         n = 5  # Adjust dimension as needed
#         G = generate_hypercube_graph(n)
#         self.maze = generate_maze(G)
#         self.screen_size = Window.size
#         self.all_pos = precompute_node_positions(self.maze, self.screen_size)
#         # Start at the node with all zeros
#         self.current_node = tuple(0 for _ in range(n))
#         # Choose a goal node (all ones)
#         self.goal = tuple(1 for _ in range(n))
#         self.visited_nodes.append(self.current_node)
#         # Initialize camera position centered on the current node
#         self.camera_position = self.all_pos[self.current_node]
#         self.register_event_type('on_touch_down')
#         Clock.schedule_interval(self.update, 1.0 / 60.0)

#     def on_touch_down(self, touch):
#         # Adjust touch position relative to camera
#         adjusted_touch_pos = (
#             touch.x - self.center_x + self.camera_position[0],
#             touch.y - self.center_y + self.camera_position[1]
#         )
#         # Check if user clicked on neighbor
#         neighbors = list(self.maze.neighbors(self.current_node))
#         moved = False
#         for neighbor in neighbors:
#             x, y = self.all_pos[neighbor]
#             distance = (
#                 (adjusted_touch_pos[0] - x) ** 2 + (adjusted_touch_pos[1] - y) ** 2
#             ) ** 0.5
#             if distance <= self.node_radius:
#                 self.path_stack.append(self.current_node)
#                 self.current_node = neighbor
#                 self.visited_nodes.append(self.current_node)
#                 # Update camera position to center on the new current node
#                 self.camera_position = self.all_pos[self.current_node]
#                 moved = True
#                 break
#         if not moved:
#             print("Clicked outside neighbor nodes.")

#     def update(self, dt):
#         self.canvas.clear()
#         self.clear_widgets()  # Clear labels to prevent duplicates
#         with self.canvas:
#             # Draw edges
#             Color(0.8, 0.8, 0.8)
#             for u, v in self.maze.edges():
#                 x1, y1 = self.all_pos[u]
#                 x2, y2 = self.all_pos[v]
#                 # Adjust positions relative to camera
#                 screen_x1 = self.center_x + x1 - self.camera_position[0]
#                 screen_y1 = self.center_y + y1 - self.camera_position[1]
#                 screen_x2 = self.center_x + x2 - self.camera_position[0]
#                 screen_y2 = self.center_y + y2 - self.camera_position[1]
#                 Line(points=[screen_x1, screen_y1, screen_x2, screen_y2], width=1)

#             # Draw nodes
#             for node in self.maze.nodes():
#                 x, y = self.all_pos[node]
#                 # Adjust positions relative to camera
#                 screen_x = self.center_x + x - self.camera_position[0]
#                 screen_y = self.center_y + y - self.camera_position[1]
#                 if node == self.current_node:
#                     Color(1, 0, 0)  # Red for current node
#                 elif node == self.goal:
#                     Color(1, 0.65, 0)  # Orange for goal node
#                 else:
#                     if node in self.visited_nodes:
#                         Color(0, 0.5, 0)  # Green for visited nodes
#                     else:
#                         Color(0, 0, 1)  # Blue for unvisited nodes
#                 Ellipse(pos=(screen_x - self.node_radius, screen_y - self.node_radius),
#                         size=(self.node_radius * 2, self.node_radius * 2))
#                 # Draw labels
#                 node_label = "".join(map(str, node))
#                 label = Label(text=node_label, center=(screen_x, screen_y - self.node_radius - 10),
#                               font_size='12sp', color=(0, 0, 0, 1))
#                 # Add the label to the widget tree so it displays on top of the canvas
#                 self.add_widget(label)

#             # Draw texts
#             goal_label = "".join(map(str, self.goal))
#             current_label = "".join(map(str, self.current_node))
#             # Since Kivy labels are widgets, we need to make sure they are added correctly
#             goal_text = Label(text=f"Goal: {goal_label}", pos=(10, self.height - 30), font_size='18sp',
#                               color=(0, 0, 0, 1), halign='left')
#             current_text = Label(text=f"Current: {current_label}", pos=(10, self.height - 60), font_size='18sp',
#                                  color=(0, 0, 0, 1), halign='left')
#             instr_text = Label(text="Click on a neighboring node to move.", pos=(10, self.height - 90),
#                                font_size='14sp', color=(0, 0, 0, 1), halign='left')
#             self.add_widget(goal_text)
#             self.add_widget(current_text)
#             self.add_widget(instr_text)

#             if self.current_node == self.goal:
#                 # Display winning message
#                 win_text = Label(text="Congratulations! You have reached the goal!", center=self.center,
#                                  font_size='24sp', color=(0, 1, 0, 1), halign='center')
#                 self.add_widget(win_text)
#                 Clock.unschedule(self.update)

# class MazeApp(App):
#     def build(self):
#         game = MazeGame()
#         return game

# if __name__ == "__main__":
#     MazeApp().run()


def parse_pos(x, y):
    return (float(x), float(y))


import networkx as nx
import random
import itertools
import numpy as np

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import Line, Color, Ellipse, Rectangle
from kivy.properties import ObjectProperty, DictProperty, ListProperty, NumericProperty
from kivy.clock import Clock
from kivy.core.window import Window
from kivy.uix.label import Label


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
    # Additionally add the hamming distance as an attribute to each edge
    for u, v in T.edges():
        hamming_dist = sum([1 for i in range(len(u)) if u[i] != v[i]])
        T.edges[u, v]["hamming_dist"] = hamming_dist
    return T


def precompute_node_positions(maze, screen_size):
    """
    Computes positions for all nodes using a suitable layout algorithm.
    """
    # Use spring layout for a better spread of nodes
    pos = nx.spring_layout(
        maze,
        scale=min(screen_size) * 2.5,
        center=(0, 0),  # Center positions around (0, 0)
        iterations=50,
        weight="hamming_dist",
    )
    return pos


class MazeGame(Widget):
    maze = ObjectProperty(None)
    current_node = ObjectProperty(None)
    goal = ObjectProperty(None)
    pos = DictProperty({})
    all_pos = DictProperty({})
    node_radius = NumericProperty(20)  # For touch detection
    visited_nodes = ListProperty([])
    path_stack = ListProperty([])
    camera_position = ListProperty([0, 0])  # Camera position to center the current node

    def __init__(self, **kwargs):
        super(MazeGame, self).__init__(**kwargs)
        # Dimension of the hypercube
        n = 5  # Adjust dimension as needed
        G = generate_hypercube_graph(n)
        self.maze = generate_maze(G)
        self.screen_size = Window.size
        self.all_pos = precompute_node_positions(self.maze, self.screen_size)
        # Start at the node with all zeros
        self.current_node = tuple(0 for _ in range(n))
        # Choose a goal node (all ones)
        self.goal = tuple(1 for _ in range(n))
        self.visited_nodes.append(self.current_node)
        # Initialize camera position centered on the current node
        self.camera_position = self.all_pos[self.current_node]
        self.register_event_type("on_touch_down")
        Clock.schedule_interval(self.update, 1.0 / 60.0)

    def on_touch_down(self, touch):
        # Adjust touch position relative to camera
        adjusted_touch_pos = parse_pos(
            touch.x - self.center_x + self.camera_position[0],
            touch.y - self.center_y + self.camera_position[1],
        )
        # Check if user clicked on neighbor
        neighbors = list(self.maze.neighbors(self.current_node))
        moved = False
        for neighbor in neighbors:
            x, y = self.all_pos[neighbor]
            distance = (
                (adjusted_touch_pos[0] - x) ** 2 + (adjusted_touch_pos[1] - y) ** 2
            ) ** 0.5
            if distance <= self.node_radius:
                self.path_stack.append(self.current_node)
                self.current_node = neighbor
                self.visited_nodes.append(self.current_node)
                # Update camera position to center on the new current node
                self.camera_position = self.all_pos[self.current_node]
                moved = True
                break
        if not moved:
            print("Clicked outside neighbor nodes.")

    def update(self, dt):
        self.canvas.clear()
        self.clear_widgets()  # Clear labels to prevent duplicates
        with self.canvas:
            # Draw edges
            Color(0.8, 0.8, 0.8)
            for u, v in self.maze.edges():
                x1, y1 = self.all_pos[u]
                x2, y2 = self.all_pos[v]
                # Adjust positions relative to camera
                screen_x1 = self.center_x + x1 - self.camera_position[0]
                screen_y1 = self.center_y + y1 - self.camera_position[1]
                screen_x2 = self.center_x + x2 - self.camera_position[0]
                screen_y2 = self.center_y + y2 - self.camera_position[1]
                Line(points=[screen_x1, screen_y1, screen_x2, screen_y2], width=1)

            # Draw nodes
            for node in self.maze.nodes():
                x, y = self.all_pos[node]
                # Adjust positions relative to camera
                screen_x = self.center_x + x - self.camera_position[0]
                screen_y = self.center_y + y - self.camera_position[1]
                if node == self.current_node:
                    Color(1, 0, 0)  # Red for current node
                elif node == self.goal:
                    Color(1, 0.65, 0)  # Orange for goal node
                else:
                    if node in self.visited_nodes:
                        Color(0, 0.5, 0)  # Green for visited nodes
                    else:
                        Color(0, 0, 1)  # Blue for unvisited nodes
                Ellipse(
                    pos=parse_pos(
                        screen_x - self.node_radius, screen_y - self.node_radius
                    ),
                    size=(self.node_radius * 2, self.node_radius * 2),
                )
                # Draw labels
                node_label = "".join(map(str, node))
                label = Label(
                    text=node_label,
                    center=parse_pos(screen_x, screen_y - self.node_radius - 10),
                    font_size="12sp",
                    color=(0, 0, 0, 1),
                )
                # Add the label to the widget tree so it displays on top of the canvas
                self.add_widget(label)

            # Draw texts
            goal_label = "".join(map(str, self.goal))
            current_label = "".join(map(str, self.current_node))
            # Since Kivy labels are widgets, we need to make sure they are added correctly
            goal_text = Label(
                text=f"Goal: {goal_label}",
                pos=parse_pos(10, self.height - 30),
                font_size="18sp",
                color=(0, 0, 0, 1),
                halign="left",
            )
            current_text = Label(
                text=f"Current: {current_label}",
                pos=parse_pos(10, self.height - 60),
                font_size="18sp",
                color=(0, 0, 0, 1),
                halign="left",
            )
            instr_text = Label(
                text="Click on a neighboring node to move.",
                pos=parse_pos(10, self.height - 90),
                font_size="14sp",
                color=(0, 0, 0, 1),
                halign="left",
            )
            self.add_widget(goal_text)
            self.add_widget(current_text)
            self.add_widget(instr_text)

            if self.current_node == self.goal:
                # Display winning message
                win_text = Label(
                    text="Congratulations! You have reached the goal!",
                    center=self.center,
                    font_size="24sp",
                    color=(0, 1, 0, 1),
                    halign="center",
                )
                self.add_widget(win_text)
                Clock.unschedule(self.update)


class MazeApp(App):
    def build(self):
        game = MazeGame()
        return game


if __name__ == "__main__":
    MazeApp().run()
