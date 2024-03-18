import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
data_str = """
-0.480456 -0.169381 0.633550 -0.055375
-0.131922 0.143322 -0.151466 0.123779
0.311075 -0.029316 -0.457655 0.201954
0.112378 0.026059 0.017915 -0.068404
"""
def rotationToVect(transformationMatrix):
    vect = np.array([0.0, 0.0, 0.0])
    vect[0] = transformationMatrix[0][3]
    vect[1] = transformationMatrix[1][3]
    vect[2] = transformationMatrix[2][3]

    rotation = np.array([
        [transformationMatrix[0][0], transformationMatrix[0][1], transformationMatrix[0][2]],
        [transformationMatrix[1][0], transformationMatrix[1][1], transformationMatrix[1][2]],
        [transformationMatrix[2][0], transformationMatrix[2][1], transformationMatrix[2][2]]
    ])

    return np.dot(vect,rotation)
# Split the string into lines and then split each line into values
rows = [[float(val) for val in line.split()] for line in data_str.strip().split('\n')]

# Convert the list of lists into a NumPy array
array = np.array(rows)

# Define the vector
vector = rotationToVect(np.array(array))
print(array)
print(vector)
# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the vector
ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], color='r')
ax.quiver(1, 0, 0, vector[0], vector[1], vector[2], color='g')

# Set the limits for the axes
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([-5, 5])

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()


#this takes a matrix transformation in SE3 and returns the vector





