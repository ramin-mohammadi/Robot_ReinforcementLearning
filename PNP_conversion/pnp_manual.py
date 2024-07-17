import cv2
import numpy as np
import math

# need to convert degree to radian by:  (degree / 180) * pi 
def get_radian(degree: float):
    return (degree / 180) * np.pi


# print(np.cos(np.pi))

# Rotational: Camera Reference Frame to match the Robot's Base Frame

# 180 around x axis to get robot's y to align with camera's  
# if worried about aligning z axis for robot and camera (in right direction) since camera 
    # z is down (direction it faces) abd z+ for robot is up, this rotation makes the z point the right way
x_rot = 180
R_X = [[1,0,0], 
       [0, np.cos(get_radian(x_rot)), -np.sin(get_radian(x_rot))], 
       [0, np.sin(get_radian(x_rot)), np.cos(get_radian(x_rot))]]

# around z axis to get
z_rot = 1.6  # 1.6
R_Z = [[np.cos(get_radian(z_rot)), -np.sin(get_radian(z_rot)), 0], 
       [np.sin(get_radian(z_rot)), np.cos(get_radian(z_rot)) , 0], 
       [0,0,1]]

# take dot products of rotational matrices for associated axes
R_Final = np.dot(R_X, R_Z)


# Displacement: (in perspective of robot base frame to get to camera's origin)
# translational_vector = [[-342],[702],[0]] # mm
additional_displacement_x = 0 # 73
additional_displacement_y = 0 # 48
translational_vector = [[-342+additional_displacement_x],[702+additional_displacement_y],[0]] # mm  

homo_matrix = np.hstack((R_Final, translational_vector)) # adds translational vecotr as a column to rotational matrix
homo_matrix = np.vstack((homo_matrix, [[0,0,0,1]])) # add row


# must turn pixel coordinates to mm 
image_x = [188.5, 172.5, 534, 35, 195, 164, 211, 180, 163, 163, 211, 45]
image_y = [210.5, 145, 336, 365, 158, 187, 216, 259, 168, 260, 180, 29]
mm_x = 0.545 # x was better at 0.54
mm_y = 0.538 # y is better now than 0.54
image_x_mm = image_x[11] / mm_x
image_y_mm = image_y[11] / mm_y
image = [[image_x_mm], [image_y_mm], [0], [1]] 

res = np.dot(homo_matrix, image)
print(res)


"""
results:  (where mm_x = 0.545, mm_y=0.538 , x_rot=180, z_rot=1.6, trans vect = [-342, 702]), camera resolution : 640x480) 

pixel: 188.5, 210.5    , generated robot: -7.188, 302.59  , actual: 7.1, 305    Off by: 14mm , 2.5mm
pixel: 172.5, 145  , generated robot: -33.13, 423.75  , actual: -17.4, 435      Off by: 16mm , 12mm
pixel: 534, 336  , generated robot: 619.996, 50.35  , actual: 670.7, 929        Off by: 50mm , 830mm   (bottom right of camera view)
pixel: 35, 365  , generated robot: -296.74, 22.03  , actual: -297.3, 6          Off by: 0.5mm , 16mm

Coordinates in Grid:
pixel: 195, 158 , generated robot: 7.45, 398.44  , actual: 25.8, 397.9       Off by: 18mm , 1.5mm
pixel: 164, 187 , generated robot: -50.9, 346.14  , actual: -35.2, 345.5     Off by: 15mm , 0.6mm
pixel: 211, 216 , generated robot: 33.79, 289.85  , actual: 53.9, 282.1      Off by: 16mm , 7mm
pixel: 163, 168 , generated robot: -51.75, 381.5  , actual: -36.7, 400.6     Off by: 15mm , 19mm
pixel: 163, 260 , generated robot: -56.52, 210.566  , actual: -39.6, 203.1   Off by: 16mm , 7.5mm
pixel: 211, 180 , generated robot: 35.66, 356.74  , actual: 53.6, 355.3      Off by: 18mm , 1.5mm



"""