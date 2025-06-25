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
pygame.display.set_caption('RRT Node Generation Animation')
clock = pygame.time.Clock()

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def get_random_point(lc=0, rc=1):
    rand_point = Node(random.uniform(lc,rc), random.uniform(lc, rc))
    return rand_point

def collision_free(node):
    global obslist
    for obs in obslist:
        distance = np.sqrt((node.x - obs[0])**2 + (node.y - obs[1])**2)
        if distance < obs[2]:
            return False
    return True

def nearest_node(new_node, tree):
    distances = [np.sqrt((new_node.x - node.x)**2 + (new_node.y - node.y)**2) for node in tree]
    nearest_index = distances.index(min(distances))
    return nearest_index

def new_point(nearest, rand_point, step_size):
    theta = math.atan2(rand_point.y - nearest.y, rand_point.x - nearest.x)
    new_x = nearest.x + step_size * math.cos(theta)
    new_y = nearest.y + step_size * math.sin(theta)
    return Node(new_x, new_y)

def add_point(tree, rand_point, step_size):
    nearest_index = nearest_node(rand_point, tree)
    nearest = tree[nearest_index]
    new_node = new_point(nearest, rand_point, step_size)
    if collision_free(new_node):
        new_node.parent = nearest
        tree.append(new_node)
        return new_node

def merge(tree, goal_node, step_size):
    global obslist
    for i in range(len(tree)):
        node = tree[i]
        if np.sqrt((node.x - goal_node.x)**2 + (node.y - goal_node.y)**2) <= step_size:
            if collision_free(Node(node.x, node.y)):
                goal_node.parent = node
                tree.append(goal_node)
                return True
    return False

def main(max_iter, step_size):
    global obslist
    obslist = [(0.15, 0.1, 0.05), (0.1, 0.4, 0.05), (0.75, 0.75, 0.075),(0.4, 0.5, 0.075),(0.7, 0.5, 0.075)]  #[(x, y, radius)]
    start_node = Node(0.05, 0.05)
    goal_node = Node(0.95, 0.95)
    tree = [start_node]

    # Create a list to store all generated nodes
    all_nodes = []

    # Main loop for animation
    for i in range(max_iter):
        rand_point = get_random_point()
        new_node = add_point(tree, rand_point, step_size)
        if new_node:
            all_nodes.append((new_node, RED))
        
        # Check for merge with goal node
        if merge(tree, goal_node, step_size):
            break

        # Draw background
        screen.fill(WHITE)

        # Draw obstacles
        for obs in obslist:
            pygame.draw.circle(screen, BLACK, (int(obs[0] * SCREEN_WIDTH), int(obs[1] * SCREEN_HEIGHT)), int(obs[2] * SCREEN_WIDTH * OBSTACLE_RADIUS_FACTOR))

        # Draw start and goal points
        pygame.draw.circle(screen, RED, (int(start_node.x * SCREEN_WIDTH), int(start_node.y * SCREEN_HEIGHT)), 10)
        pygame.draw.circle(screen, GREEN, (int(goal_node.x * SCREEN_WIDTH), int(goal_node.y * SCREEN_HEIGHT)), 10)

        # Draw all nodes and lines between them
        for node, color in all_nodes:
            pygame.draw.circle(screen, color, (int(node.x * SCREEN_WIDTH), int(node.y * SCREEN_HEIGHT)), 5)
            if node.parent:
                pygame.draw.line(screen, color, (int(node.x * SCREEN_WIDTH), int(node.y * SCREEN_HEIGHT)),
                                 (int(node.parent.x * SCREEN_WIDTH), int(node.parent.y * SCREEN_HEIGHT)), 1)

        # Update display
        pygame.display.flip()
        clock.tick(30)

    # Draw final path
    current_node = tree[-1]
    while current_node.parent:
        pygame.draw.line(screen, BLUE, (int(current_node.x * SCREEN_WIDTH), int(current_node.y * SCREEN_HEIGHT)),
                         (int(current_node.parent.x * SCREEN_WIDTH), int(current_node.parent.y * SCREEN_HEIGHT)), 2)
        current_node = current_node.parent
    pygame.display.flip()

    # Wait for user to close window
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='RRT Animation')
    parser.add_argument('-i','--iter', type=int, help='Number of iterations (integer); Default=100', default=100)
    parser.add_argument('-s','--step_size', type=float, help='Step size for node expansion; Default=0.05', default=0.06)
    args = parser.parse_args()
    max_iter = args.iter
    step_size = args.step_size
    main(max_iter, step_size)
