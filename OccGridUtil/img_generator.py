#import cv2
from PIL import Image
import numpy as np
import logging
import sys
logging.basicConfig(level=logging.DEBUG)

# Read the image using Pillow
logging.debug("input file name")
file_name = input()
#print(file_name,flush=True)

image = Image.open(file_name)

# Convert the PIL image to a NumPy array
image_array = np.array(image)
new_image = np.array(image_array[:,:,0])
#image_array = 
output_array = new_image
# Convert the NumPy array to RGB (if needed)
#image_rgb = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)

# Print the type and shape of the NumPy array

for i,rows in enumerate(new_image):
    for j,points in enumerate(rows):
        if(points != 255 or i >= 51 or j < 7 or j > 148 or ((((j >= 7) and (j <= 53)) and ((float(i) - float(j)) > -5))) or  ((((j <= 148) and (j >= 102)) and ((float(i) + float(j)) > (150))))):
            #pass
            output_array[i,j] = 0
        else:
            pass
            output_array[i,j] = 1
            #print(points)
            
output_array[50,78] = 9
#print(type(image_array))  # <class 'numpy.ndarray'>
#print(output_array.shape)
np.set_printoptions(threshold=np.inf,linewidth=1000)
print(output_array)
