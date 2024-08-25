import numpy as np
import csv

# Generate the array
data = []
for i in np.arange(0, 10, 0.5):
    x = 5 * np.sin(i)
    y = 5 * np.cos(i)
    z = i
    data.append([x, y, z])

# Write the array to a CSV file
csv_file = 'trace.csv'
with open(csv_file, 'w', newline='') as file:
    writer = csv.writer(file)
    # Writing the header (optional)
    # writer.writerow(["x", "y", "z"])
    # Writing the data
    writer.writerows(data)

print(f"Data written to {csv_file}")