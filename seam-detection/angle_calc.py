import numpy as np

sensor_width = 22.5
focal_length = 28.75
angle_of_view = 2 * np.arctan(sensor_width/(2 * focal_length)) *(180/np.pi)
print(angle_of_view)