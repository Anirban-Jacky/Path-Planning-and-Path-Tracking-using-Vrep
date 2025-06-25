import pygame
import math
import numpy as np
import random
import argparse

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

# Define display dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Define obstacle radius factor for scaling
OBSTACLE_RADIUS_FACTOR = 0.5

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Bi-RRT Node Generation Animation')
clock = pygame.time.Clock()

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_so_far = []
        self.parent = None

def get_random_point(lc=0, rc=1):
    rand_point = Node(random.uniform(lc,rc), random.uniform(lc, rc))
    return rand_point

def merge_intersection(node1, node2):
    global obslist
    for obs in obslist:
        dr = np.hypot(node2.x-node1.x, node2.y-node1.y)
        D = (node1.x-obs[0])*(node2.y-obs[1]) - (node2.x-obs[0])*(node1.y-obs[1])
        d = (obs[2]*dr)**2 - D**2
        if d >= 0:
            return False
    return True

def get_nearest_point(point, tree):
    distance_list = [np.hypot(node.x - point.x, node.y - point.y)**2 for node in tree]
    return distance_list.index(min(distance_list))

def get_new_point(rnd_point, old_point, delta=0.06):
    theta = math.atan2((old_point.y - rnd_point.y), (old_point.x - rnd_point.x))
    temp_p1 = Node(delta * math.cos(theta) + old_point.x, delta * math.sin(theta) + old_point.y)
    temp_p2 = Node(-delta * math.cos(theta) + old_point.x, -delta * math.sin(theta) + old_point.y)
    temp_tree = [temp_p1, temp_p2]
    new_node = temp_tree[get_nearest_point(rnd_point, temp_tree)]
    val = merge_intersection(new_node, old_point)
    if val and 0 <= new_node.x <= 1 and 0 <= new_node.y <= 1:
        return new_node
    return False

def add_point(tree):
    rand_point = get_random_point()
    min_dist_index = get_nearest_point(rand_point, tree)
    old_point = tree[min_dist_index]
    new_node = get_new_point(rand_point, old_point)
    if new_node:
        new_node.parent = old_point
        tree.append(new_node)
        new_node.path_so_far = old_point.path_so_far + [new_node]
        return new_node

def merge(tree1, tree2):
    nl = []
    for node in tree1[::-1]:
        for n in tree2[::-1]:
            if merge_intersection(node, n):
                nl.append((node, n, len(node.path_so_far) + len(n.path_so_far)))
    if nl:
        nl.sort(key=lambda x: x[2])
        path = nl[0][0].path_so_far + nl[0][1].path_so_far[::-1]
        print("Done!!")
    else:
        print("No path found")
        path = None
    return path

def main(max_iter):
    global obslist
    obslist = [(0.15, 0.1, 0.05), (0.1, 0.4, 0.05), (0.75, 0.75, 0.075),(0.4, 0.5, 0.075),(0.7, 0.5, 0.075)]  #[(x, y, radius)]
    start_node = Node(0.05, 0.05)
    start_node.path_so_far.append(start_node)
    goal_node = Node(0.95, 0.95)
    goal_node.path_so_far.append(goal_node)
    start_tree = [start_node]
    goal_tree = [goal_node]

    # Create a list to store all generated nodes
    all_nodes = []

    # Create a list to store nodes for animation
    animation_nodes = []

    # Main loop for animation
    for i in range(max_iter):
        new_node_start = add_point(start_tree)
        new_node_goal = add_point(goal_tree)
        if new_node_start:
            all_nodes.append((new_node_start, RED))
            animation_nodes.append((new_node_start, RED))
        if new_node_goal:
            all_nodes.append((new_node_goal, BLUE))
            animation_nodes.append((new_node_goal, BLUE))

        # Draw background
        screen.fill(WHITE)

        # Draw obstacles
        for obs in obslist:
            pygame.draw.circle(screen, BLACK, (int(obs[0] * SCREEN_WIDTH), int(obs[1] * SCREEN_HEIGHT)), int(obs[2] * SCREEN_WIDTH * OBSTACLE_RADIUS_FACTOR))

        # Draw start and goal points
        pygame.draw.circle(screen, RED, (int(start_node.x * SCREEN_WIDTH), int(start_node.y * SCREEN_HEIGHT)), 10)
        pygame.draw.circle(screen, GREEN, (int(goal_node.x * SCREEN_WIDTH), int(goal_node.y * SCREEN_HEIGHT)), 10)

        # Draw all nodes
        for node, color in all_nodes:
            pygame.draw.circle(screen, color, (int(node.x * SCREEN_WIDTH), int(node.y * SCREEN_HEIGHT)), 5)
            if node.parent:
                pygame.draw.line(screen, color, (int(node.x * SCREEN_WIDTH), int(node.y * SCREEN_HEIGHT)),
                                 (int(node.parent.x * SCREEN_WIDTH), int(node.parent.y * SCREEN_HEIGHT)), 1)
        # Update display
        pygame.display.flip()
        clock.tick(30)

    # Merge trees after animation
    path = merge(start_tree, goal_tree)

    # Draw final path
    if path:
        screen.fill(WHITE)
        for obs in obslist:
            pygame.draw.circle(screen, BLACK, (int(obs[0] * SCREEN_WIDTH), int(obs[1] * SCREEN_HEIGHT)), int(obs[2] * SCREEN_WIDTH * OBSTACLE_RADIUS_FACTOR))
        pygame.draw.circle(screen, RED, (int(start_node.x * SCREEN_WIDTH), int(start_node.y * SCREEN_HEIGHT)), 10)
        pygame.draw.circle(screen, GREEN, (int(goal_node.x * SCREEN_WIDTH), int(goal_node.y * SCREEN_HEIGHT)), 10)
        for node, color in all_nodes:
            pygame.draw.circle(screen, color, (int(node.x * SCREEN_WIDTH), int(node.y * SCREEN_HEIGHT)), 5)
            if node.parent:
                pygame.draw.line(screen, color, (int(node.x * SCREEN_WIDTH), int(node.y * SCREEN_HEIGHT)),
                                 (int(node.parent.x * SCREEN_WIDTH), int(node.parent.y * SCREEN_HEIGHT)), 1)
        pygame.draw.lines(screen, BLACK, False, [(int(node.x * SCREEN_WIDTH), int(node.y * SCREEN_HEIGHT)) for node in path], 2)
        pygame.display.flip()

    # Wait for user to close window
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == '__main__':
    src=0.1
    parser = argparse.ArgumentParser(description='Bi-Directional RRT')
    parser.add_argument('-i','--iter', type=int, help='Number of iterations (integer); Default=200', default=int(src*1000))
    args = parser.parse_args()
    max_iter = args.iter
    main(max_iter)
