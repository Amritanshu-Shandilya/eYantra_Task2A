import matplotlib.pyplot as plt
import sys

if len(sys.argv) != 2:
    print("Please specify points file")
    sys.exit()


# Initialize empty lists to store x and y coordinates
x_coords = []
y_coords = []

# Read the file and extract coordinates
with open(sys.argv[1], 'r') as file:
    for line in file:
        x, y = map(float, line.split())
        x_coords.append(x)
        y_coords.append(y)

# Create the plot
plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='blue')  # Set a default color for the line
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Connected Points')
# Show the plot
plt.show()