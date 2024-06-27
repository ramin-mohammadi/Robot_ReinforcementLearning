import cv2
import numpy as np


# Function to transform image coordinates to robot coordinates
def transform_to_robot_coordinates(image_point, depth, mtx, rotation_vector, translation_vector):
    # Convert image point to homogeneous coordinates
    image_coordinates = np.array([[image_point[0]], [image_point[1]], [1]], dtype=np.float32)

    # Calculate camera coordinates (assuming depth in mm)
    camera_coordinates = np.linalg.inv(mtx).dot(image_coordinates) * depth

    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

    # Apply the extrinsic transformation to get robot coordinates
    robot_coordinates = rotation_matrix.dot(camera_coordinates) + translation_vector

    return robot_coordinates.ravel()







camera_points = np.array([[49, 137.5],
                 [35, 365],
                 [506, 200],
                 [534, 336],
                 [280,179],
                 [292,439],
                 [188,210]] , dtype=np.float32)

robot_points = np.array([[-263.4, 448.9, 0], # 150
                 [-297.3, 6, 0], # 156.9
                 [629.9, 312.5, 0],  # 166.9
                 [670.7, 929, 0], # 171.3
                 [195.2, 379.9, 0],#161.9
                 [205.4, -124.2, 0],
                 [6.9, 309.9, 0]] , dtype=np.float32) 


# Load calibration data
with np.load('./multiple_img_calibration_data.npz') as data: # change to "multiple_img_calibration_data.npz" before running or if using different data imgs
    mtx = data['mtx']
    dist = data['dist']
    rvecs = data['rvecs']
    tvecs = data['tvecs']
# camera callibration returns camera matrix (contains camera centers x y and focal length x y), distortion coefficients, rotational vectors, translational vectors
    
# print(mtx, "dddddddddddddddddddddd\n")
# print(dist, "dddddddddddddddddddddd\n")
# print(rvecs, "dddddddddddddddddddddd\n")
# print(tvecs, "dddddddddddddddddddddd\n")

# returns bool value, rotational vector  and translational vector 
# # (success, rotation_vector, translation_vector) = cv2.solvePnP()
success, rvec, tvec = cv2.solvePnP(objectPoints=robot_points, imagePoints=camera_points, cameraMatrix=mtx, distCoeffs=dist) # optionally pass rvec and tvec

print(rvec)
print(tvec)

# turn rotational vector into rotational matrix
# rot_matrix[0] contains 3x3 rotational matrix
rot_matrix = cv2.Rodrigues(rvec) 

print(rot_matrix)

homo_matrix = np.hstack((rot_matrix[0], tvec))
# homo_matrix = np.hstack((real_rotation_matrix, [[0],[0],[0],[1]]))
# homo_matrix = np.vstack((homo_matrix, [[0,0,0,1]]))

print(homo_matrix)


# Example usage of the transformation function
# image_point = [188.5, 210.5]  # Coordinates in the image
# depth = 870  # Depth value in mm, prob not focal length
# robot_coordinates = transform_to_robot_coordinates(image_point, depth, mtx, rvec, tvec)
# X, Y, Z = robot_coordinates
# print(f"Transformed robot coordinates: X={X}, Y={Y}, Z={Z}")

image = [[188.5], [210.5], [160], [1]]

print(np.dot(np.dot(mtx, homo_matrix ) , image))
# print(np.dot(homo_matrix , image))

# print(mtx)










# tvec = np.concatenate(tvec[0], tvec[1], tvec[2])
# HO_C = np.concatenate(np.concatenate((rot_matrix[0], tvec, 1)), [[0,0,0,1]], 0)


# ZYX, jacob = cv2.Rodrigues(rvec)

# totalrotmax=np.array([[ZYX[0,0],ZYX[0,1],ZYX[0,2],tvec[0]],
# [ZYX[1,0],ZYX[1,1],ZYX[1,2],tvec[1]],
# [ZYX[2,0],ZYX[2,1],ZYX[2,2],tvec[2]],
# [0,0,0,1]])

# WtoC=np.mat(totalrotmax)

# inverserotmax=np.linalg.inv(totalrotmax)
# f=inverserotmax
# print(inverserotmax)