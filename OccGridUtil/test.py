import numpy as np
import random
from collections import deque
import math
import matplotlib.pyplot as plt
import time


global using_angle
using_angle = True
# Path to the file


file_path = 'testout2.txt'
start_time = time.process_time()


# Read the file and preprocess it
with open(file_path, 'r') as file:
    lines = file.readlines()

# Remove brackets and split into numbers
cleaned_data = [line.replace('[', '').replace(']', '').strip() for line in lines]

# Convert to a NumPy array
matrix = np.array([list(map(int, line.split())) for line in cleaned_data])
print(np.sum(matrix))
print(matrix.shape)


np.set_printoptions(threshold=np.inf,linewidth=1000)

# Why do we need to have a seperate start_bfs and robot_pose you may ask?
# if you think about it, the robot_pose is in unknown space from CV's perspective
# So if we bfs from the robot_pose, the algo won't work 
# (you won't add any neighbors to the queue in the first iteration)
# Buuuuuut if we do angle calculations from the start_bfs, we could get skewed results
# So now you need 
# Sincelry,
# Maaz , self documenting code since 2004

start_bfs = (47, 78)
robot_pose = (55, 78)


# Directions: Vertical, Horizontal, Diagonal
# directions = [(-1, 0),   # Up
#               (-1, -1),  # Up-left (diagonal)
#               (-1, 1),   # Up-right (diagonal)
#               (0, -1),   # Left
#               (0, 1),
#               (1,0),
#               (1,1),
#               (1,-1)]  

directions = [(-1, -1),  # Up-left (diagonal)
              (-1, 1),
              (1,1),
              (1,-1)]   # Right

# Function to check if a position is valid
def is_valid_move(position, matrix, visited):
    
    

    y, x = position
    rows, cols = matrix.shape
    return 0 <= y < rows and 0 <= x < cols and matrix[y, x] == 1 and position not in visited

# assumes radians angles
def get_angle_difference(to_angle, from_angle):
    delta = to_angle - from_angle
    delta = (delta + math.pi) % (2 * math.pi) - math.pi
    return delta

def find_desired_heading( cur_gps, goal_gps, orientation):
    # lat long degress to meter ratio for the state of Michigan
    latitude_length=111086.2 
    longitude_length=81978.2 
        
    delta_lat = goal_gps[0] - cur_gps[0]
    delta_lon = cur_gps[1] - goal_gps[1]
    north_m = delta_lat * latitude_length
    west_m = delta_lon * longitude_length
    
    # desired_heading_global = math.atan2(west_m, north_m)
    desired_heading_x = math.cos(orientation) * west_m + math.sin(orientation) * north_m
    desired_heading_y = -math.sin(orientation) * west_m + math.cos(orientation) * north_m
    desired_heading_global = math.atan2(desired_heading_y, desired_heading_x) - math.pi
    return desired_heading_global

def get_angle_to_goal_pentaly(canidate_node, real_robot_pos, orientation, desired_heading_global):
    y,x = canidate_node
    outside_point_y, outside_point_x = real_robot_pos
    dx = x - outside_point_x
    dy = y - outside_point_y
    cell_dir_local = math.atan2(dy, dx)  

    global_cell_dir = orientation + cell_dir_local + math.pi * 0.5

    heading_error = abs(get_angle_difference(desired_heading_global, global_cell_dir))
    heading_error_deg = math.degrees(heading_error)
    return heading_error_deg
 

 #TODO combine first 3 args into a tuple
def calculate_cost(real_rob_pose, orientation ,desire_heading, start, current, rows, cols): 
    edge_penalty_factor=0.2
    distance_weight=0.09
    min_distance=1e-6
    start_penalty_factor=100
    angle_pen_weight = 10
    obs_factor = 10000
    
    angle_pen = 0
    if using_angle:
        angle_pen = get_angle_to_goal_pentaly(current, real_rob_pose, orientation, desire_heading)

    y_start, x_start = start
    y_current, x_current = current
    
    # Euclidean distance
    euclidean_distance = math.sqrt((x_current - x_start)**2 + (y_current - y_start)**2)

    # Ensure the distance is not zero when current == start
    if euclidean_distance == 0:
        euclidean_distance = min_distance  # Avoid zero distance

    # Apply weight to the distance
    weighted_distance = distance_weight * euclidean_distance

    # Pull from inflation layer 
    min_distance_to_obstacle = matrix[y_current][x_current]
    
    # Edge penalty
    edge_penalty = min(x_current, cols - x_current - 1, y_current, rows - y_current - 1)
    edge_penalty = max(0, edge_penalty)  # Ensure non-negative
    
    close_pen = 0
    # Penalize if the current point is too close to the start
    if euclidean_distance <= min_distance:
        close_pen += start_penalty_factor  # Add penalty to move away from the start

    # Final cost
    cost = close_pen + 4*(1/weighted_distance) + obs_factor * min_distance_to_obstacle + edge_penalty_factor * (1 / (edge_penalty + 1)) + angle_pen * angle_pen_weight
    return cost


# BFS Function

def bfs_with_cost(robot_pose, matrix, start_bfs, current_gps=0, goal_gps=0, robot_orientation=0):
     # Calculate cost for this cell
    current_gps = (42.668086, -83.218446) # TODO get this from sensors
    goal_gps = (42.6679277, -83.2193276) # TODO get this from publisher
    robot_orientation = math.radians(270) #TODO get this from sensors


    rows, cols = matrix.shape
    visited = set()
    queue = deque([start_bfs])
    visited.add(start_bfs)

    min_cell_cost = float('inf')
    best_cell = None

    while queue:
        y, x = queue.popleft()

       
        d_heading = 0
        if using_angle:
            d_heading = find_desired_heading(current_gps, goal_gps, robot_orientation)
        
        cost = calculate_cost(robot_pose, robot_orientation, d_heading, start_bfs, (y, x), rows, cols)
        if cost < min_cell_cost:
            min_cell_cost = cost
            best_cell = (y, x)

        # Explore neighbors
        for dy, dx in directions:
            ny, nx = y + dy, x + dx
            if 0 <= ny < rows and 0 <= nx < cols and matrix[ny, nx] == 1 and (ny, nx) not in visited:
                queue.append((ny, nx))
                visited.add((ny, nx))

    return best_cell, min_cell_cost


# Visualize the cost map
def visualize_cost_map(cost_map):
    plt.figure(figsize=(10, 8))
    plt.imshow(cost_map, cmap='viridis', origin='upper')
    plt.colorbar(label='Cost')
    plt.title("Cost Heatmap")
    plt.xlabel("X-axis (Columns)")
    plt.ylabel("Y-axis (Rows)")
    plt.show()

def visualize_matrix_with_goal(matrix, start, goal):
    plt.figure(figsize=(8, 8))
    plt.imshow(matrix, cmap="gray_r")  # Display matrix (1=white, 0=black)
    
    # Mark the start point
    plt.scatter(start[1], start[0], color="blue", label="Start", s=100)
    
    # Mark the optimal goal
    plt.scatter(goal[1], goal[0], color="red", label="Goal", s=100)
    
    # Add grid for clarity
    plt.grid(color="black", linestyle="--", linewidth=0.5)
    plt.xticks(range(matrix.shape[1]))
    plt.yticks(range(matrix.shape[0]))
    #plt.gca().invert_yaxis()  # Invert y-axis to align with matrix indexing
    plt.legend()
    plt.title("Matrix Visualization with Start and Goal")
    plt.show()


# Run BFS


min_cost_cell, min_cost  = bfs_with_cost(robot_pose, matrix, start_bfs)


print(f"\nCell with Minimum Cost: {min_cost_cell}, Minimum Cost: {min_cost:.2f}")

runtime = time.process_time() - start_time
# visualize_cost_map(cost_map)
visualize_matrix_with_goal(matrix, start_bfs, min_cost_cell)



# Print final matrix and path
# print("Final Matrix:")
# print(matrix)
# print("Path Traversed:")
# print(path)

print(matrix[27][102])

print("Time w/o visualization: ", runtime)