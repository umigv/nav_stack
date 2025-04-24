import numpy as np
import random
from collections import deque
# Path to the file
file_path = 'testout1.txt'

# Read the file and preprocess it
with open(file_path, 'r') as file:
    lines = file.readlines()

# Remove brackets and split into numbers
cleaned_data = [line.replace('[', '').replace(']', '').strip() for line in lines]

# Convert to a NumPy array
matrix_ones= np.array([list(map(int, line.split())) for line in cleaned_data])
matrix = matrix_ones* 127
print(np.sum(matrix))
print(matrix.shape)


np.set_printoptions(threshold=np.inf,linewidth=1000)
#print(matrix)
print(matrix[50][78])

start = (50, 78)

# Expanded Directions (Up, Diagonal, Horizontal)
directions = [(-1, 0),   # Up
              (-1, -1),  # Up-left (diagonal)
              (-1, 1),   # Up-right (diagonal)
              (0, -1),   # Left
              (0, 1)]    # Right

# Function to check if a position is valid (within bounds and not an obstacle)
def is_valid_move(position, matrix, visited):
    y, x = position
    rows, cols = matrix.shape
    return 0 <= y < rows and 0 <= x < cols and matrix[y, x] == 1 and position not in visited

# BFS Function to find a path or goal
def bfs(matrix, start):
    queue = deque([start])  # Start with the initial position in the queue
    visited = set()  # Set to keep track of visited positions
    visited.add(start)
    
    while queue:
        current_position = queue.popleft()
        y, x = current_position
        
        # Print or store the current position as you're visiting it
        #print(f"Visited: {current_position}")
        
        # Goal condition: if you have a specific goal, you can check here
        # For now, the search will stop when there are no more valid moves.
        
        for direction in directions:
            new_position = (y + direction[0], x + direction[1])
            
            if is_valid_move(new_position, matrix, visited):
                queue.append(new_position)
                visited.add(new_position)
                matrix[new_position[0],new_position[1]] = 8
                # Print the new position after moving
                #print(f"Moving to: {new_position}")

        # You can modify this condition if you want a specific stop or goal logic
        if len(queue) == 0:
            print("No valid moves left.")
            break

        #print(visited)

# Run BFS
bfs(matrix, start)






np.set_printoptions(threshold=np.inf,linewidth=1000)
