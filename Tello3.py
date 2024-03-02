from djitellopy import tello
import math
import numpy as np
from ultralytics import YOLO
import cv2

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.e = 0 # Add, drone energy valuable
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = start_node.e = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = end_node.e = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(1, 1, 1), (-1, -1, -1),
                             (0, 1, 1), (0, -1, -1), (0, 1, -1), (0, -1, 1), (0, 1, 0), (0, -1, 0), (0, 0, -1), (0, 0, 1)
                             (1, 1, 1), (1, -1, -1), (1, 1, -1), (1, -1, 1), (1, 1, 0), (1, -1, 0), (1, 0, -1), (1, 0, 1)
                             (-1, 1, 1), (-1, -1, -1), (-1, 1, -1), (-1, -1, 1), (-1, 1, 0), (-1, -1, 0), (-1, 0, -1), (-1, 0, 1)
                             ]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1], current_node.position[2] + new_position[2])

            # Make sure within range
            if (node_position[0] > len(maze) - 1) or (node_position[0] < 0) or \
                (node_position[1] > len(maze[0]) - 1) or (node_position[1] < 0) or \
                (node_position[2] > len(maze[0][0]) - 1) or (node_position[2] < 0):
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1][node_position[2]]] == 1:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, h and e values
            # Assuming current_node.position, current_node.parent.position, and child.position are tuples of (x, y) coordinates
            alpha = (current_node.position[0] - current_node.parent.position[0], 
                    current_node.position[1] - current_node.parent.position[1],
                    current_node.position[2] - current_node.parent.position[2])

            beta = (child.position[0] - current_node.position[0], 
                    child.position[1] - current_node.position[1],
                    child.position[2] - current_node.position[2],)

            # Calculate the dot product of alpha and beta
            dot_product = np.dot(alpha, beta)

            # Calculate the magnitudes of alpha and beta
            magnitude_alpha = np.linalg.norm(alpha)
            magnitude_beta = np.linalg.norm(beta)

            # Calculate the cosine of the angle between the vectors
            cos_theta = dot_product / (magnitude_alpha * magnitude_beta)

            # Calculate the angle in radians
            theta = math.acos(cos_theta)

            child.e = theta
            child.g = current_node.g + 1
            child.h = abs(child.position[0] - end_node.position[0]) + \
                    abs(child.position[1] - end_node.position[1]) + \
                    abs(child.position[2] - end_node.position[2])
            child.f = child.g + child.h + child.e

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


# Create a variale to control the Tello Drone
maverick = tello.Tello()
print(maverick.get_battery())

# Connect to the drone 
maverick.connect()
maverick.for_back_velocity = 0
maverick.left_right_velocity = 0
maverick.up_down_velocity = 0
maverick.yaw_velocity = 0
maverick.speed = 0

# Set video stream on
maverick.streamon()

# take off and set speed
maverick.takeoff()
maverick.set_speed(50)

# Definition start and dest position
bcuav_x = 0
bcuav_y = 0
bcuav_z = maverick.get_height()
dest_position = '1230.860.100'
dest_position.split('.')

# Import YOLO model
model = YOLO('best.pt')

# Read camera frame
cap = cv2.VideoCapture(0)

# One rotation N one node when there is no CUAV, 
# One rotation 2N node when there is CUAV
nodes_per_rotate = 1

while cap.isOpened():

    # Read a frame from the video
    success, frame = cap.read()
    if success:
        results = model(frame)

        # If there's a CUAV
        if results:
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cuav = box.xywh
                    cuav.reshape(1, 4)
                    print("Warning: CUAV is here!")
                    print(f"BCAUV yaw: {maverick.get_yaw()}")
                    print(f"Coordinates (x, y): {cuav[0][0]}, {cuav[0][1]}")
                    print(f"Size (w, h): {cuav[0][2]}, {cuav[0][3]}")
                    print("Class:", box.cls.item())

                    # Calculate drone size

                    # point in maze and path planning

        # else
        else:
            maverick.move_forward(20)
            maverick.rotate_clockwise(90)

# Land the drone
maverick.land()