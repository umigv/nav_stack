import rclpy
from rclpy.node import Node
from map_interfaces.srv import GoalSelection  # Define a service GoalSelection
from map_interfaces.srv import InflationGrid  # Service to request occupancy grid
import numpy as np

import numpy as np
import random
from collections import deque
import math
import matplotlib.pyplot as plt
import time



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

def calculate_cost(real_rob_pose, orientation ,desire_heading, start, current, rows, cols, matrix): 
    edge_penalty_factor=.2
    distance_weight=.5
    min_distance=2
    start_penalty_factor=100
    angle_pen_weight = 0
    obs_factor = 1.5
    
    angle_pen = 0
    if using_angle:
        angle_pen = get_angle_to_goal_pentaly(current, real_rob_pose, orientation, desire_heading)

    y_start, x_start = start
    y_current, x_current = current
    
    # Euclidean distance
    euclidean_distance = (math.sqrt((x_current - x_start)**2 + (y_current - y_start)**2))
    # euclidean_distance = (math.sqrt((x_current - x_start)**2 + (y_current - y_start)**2))

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
   
    cost = close_pen + 4*(1/(weighted_distance+1)) + obs_factor * min_distance_to_obstacle + edge_penalty_factor * (1 / (edge_penalty + 1)) + angle_pen * angle_pen_weight
   
    return cost


# BFS Function

def bfs_with_cost(robot_pose, matrix, start_bfs, directions, current_gps=0, goal_gps=0, robot_orientation=0):
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

    goal_cost_matrx = np.zeros_like(matrix, dtype=np.float64) + 100.0

    where_visted = np.zeros_like(matrix)
    num_visted = 0
    # visualize_cost_map(goal_cost_matrx)

    while queue:
        num_visted += 1
        y, x = queue.pop() # pop for dfs pop left for bfs

        d_heading = 0
        if using_angle:
            d_heading = find_desired_heading(current_gps, goal_gps, robot_orientation)
        
        cost = calculate_cost(robot_pose, robot_orientation, d_heading, start_bfs, (y, x), rows, cols, matrix)
        goal_cost_matrx[y][x] = cost
        where_visted[y][x] = 1
        if cost < min_cell_cost: 
            min_cell_cost = cost
            best_cell = (y, x)
        # Explore neighbors
        for dy, dx in directions:
            ny, nx = y + dy, x + dx
            if 0 <= ny < rows and 0 <= nx < cols and matrix[ny,nx] >= 0 and matrix[ny, nx] < 100 and (ny, nx) not in visited:
                queue.append((ny, nx))
                visited.add((ny, nx))
    # visualize_cost_map(where_visted)
    # visualize_cost_map(goal_cost_matrx)
    print(best_cell)
    visualize_matrix_with_goal(goal_cost_matrx,robot_pose, best_cell)
    print("Number of cells visited: ", num_visted)
    # max_value = np.max(goal_cost_matrx)
    # min_value = np.min(goal_cost_matrx)

    # print("Maximum value:", max_value)
    # print("Minimum value:", min_value)
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
    print(" GOAL ", goal)
    print(" START ", start)
    plt.figure(figsize=(8, 8))
    plt.imshow(matrix, cmap="viridis") 
    plt.colorbar(label='Cost')

    # Mark start and goal points
    plt.scatter(start[1], start[0], color="blue", label="Start", s=100)
    plt.scatter(goal[1], goal[0], color="red", label="Goal", s=100)

    # Add grid for clarity
    plt.grid(color="black", linestyle="--", linewidth=0.5)

    # Set tick positions to avoid crowding
    plt.xticks(np.arange(0, matrix.shape[1], step=max(1, matrix.shape[1] // 10)))
    plt.yticks(np.arange(0, matrix.shape[0], step=max(1, matrix.shape[0] // 10)))

    plt.legend()
    plt.title("Matrix Visualization with Start and Goal")
    plt.show()

class GoalSelectionService(Node):
    def __init__(self):
        print("Goal Selection Node INIT")
        super().__init__('goal_selection_node')
        # self.srv = self.create_service(GoalSelection, 'goal_selection_service', self.goal_selection_callback)
        self.cli = self.create_client(InflationGrid, 'inflation_grid_service')
        print("Waiting for inflation_grid_service...")
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for inflation_grid_service...')

        self.req = InflationGrid.Request()
        print("Goal Selection Node INIT Completed")
        # time.sleep(1)
        # print("make the request")

    
    def send_request(self):

        return self.cli.call_async(self.req)

    def goal_selection_wrapper(self, grid_msg):
       
        print("Started goal_selection ")

        robot_pose_x, robot_pose_y = grid_msg.robot_pose_x, grid_msg.robot_pose_y
        matrix = np.array(grid_msg.occupancy_grid.data).reshape((grid_msg.occupancy_grid.info.height, grid_msg.occupancy_grid.info.width))
        matrix = np.fliplr(matrix)
        # start_bfs = (47, 78)
        # robot_pose = (55, 78)
        start_bfs_factor = 8 # we don't want to start the search from the robot's position (because its in unknown space) 
        # so shift the starting node this much "up"
        # print("Trying to visualize cost map")
        # visualize_cost_map(matrix)
        # print("Finished  to visualize cost map")

        global using_angle
        using_angle = False # boolean to say whether to use the angle to waypoint in the cost function, set to false for now

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


        start_bfs = (robot_pose_x - start_bfs_factor, robot_pose_y)  # Example offset for BFS start()
        min_cost_cell, min_cost  = bfs_with_cost((robot_pose_x, robot_pose_y), matrix, start_bfs, directions)
        print("Cell with Minimum Cost: ", min_cost_cell, "Minimum Cost: ", min_cost)
        # response.occupancy_grid = grid_msg.occupancy_grid
        # response.robot_pose_x = robot_pose_x
        # response.robot_pose_y = robot_pose_y
        # response.goal_x = min_cost_cell[1]
        # response.goal_y = min_cost_cell[0]
        
        return


def main():
    rclpy.init()
    node = GoalSelectionService()
    
    try:
        while rclpy.ok():
            print("Sending request")
            future = node.send_request()
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            
            if response:
                print("Got a response")
                print(response.robot_pose_x)
                node.goal_selection_wrapper(response)
            else:
                print("No response received")
            
            time.sleep(10)  # Wait for 10 seconds before the next request
    
    except KeyboardInterrupt:
        print("Shutting down...")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
