import matplotlib as mpl
import numpy as np
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import csv

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.add_subplot(projection='3d', autoscale_on=False)


# Read data from CSV
data = []
with open('cmake-build-debug/RigidRandyPlot.csv', 'r') as file:
    reader = csv.reader(file)
    #next(reader)  # Skip header
    for row in reader:
        row_data = []
        for entry in row:
            if entry.strip():  # Skip empty strings
                try:
                    row_data.append(float(entry))
                except ValueError:
                    pass
        if row_data:
            data.append(row_data)

# Read data from CSV
data1 = []
with open('testData/posData.csv', 'r') as file:
    reader = csv.reader(file)
    #next(reader)  # Skip header
    for row in reader:
        row_data = []
        for entry in row:
            if entry.strip():  # Skip empty strings
                try:
                    row_data.append(float(entry))
                except ValueError:
                    pass
        if row_data:
            data1.append(row_data)


print("TOTAL DIFFERENCE")
print(np.sum(np.subtract(data, data1[0:len(data)])))
# Animation function
def animate(i, data, data1):
    # Extract x, y, z for current animation step
    start_index = 3 * i
    end_index = start_index + 3
    x = data[i*3]
    y = data[(i*3)+1]
    z = data[(i*3)+2]

    x1 = data1[i*3]
    y1 = data1[(i*3)+1]
    z1 = data1[(i*3)+2]

    ax.clear()
    ax.plot(x, y, z, label='RigidTest')
    ax.plot(x1, y1, z1, label='RigidTest_Correct', color='green')

    ax.scatter(x, y, z, color='red')  # Plot circles at each point
    ax.scatter(x1, y1, z1, color='green')  # Plot circles at each point
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)




ani = animation.FuncAnimation(fig, animate, fargs=(data, data1), frames=len(data) // 3, interval=100)

ax.autoscale(False)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2)


plt.show()
