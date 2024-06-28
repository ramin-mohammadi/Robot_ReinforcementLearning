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


# https://medium.com/@pacogarcia3/calculate-x-y-z-real-world-coordinates-from-image-coordinates-using-opencv-from-fdxlabs-0adf0ec37cef
def calculate_XYZ(u,v, camera_matrix, tvec, rot_matrix):
                                      
    #Solve: From Image Pixels, find World Points

    uv_1=np.array([[u,v,1]], dtype=np.float32)
    uv_1=np.transpose(uv_1)
    scaling_factor = 1 / 0.56 # divide to convert pixel to mm
    suv_1=uv_1 * scaling_factor
    # print("====\n", np.linalg.inv(camera_matrix), "\n====\n")
    xyz_c=np.dot(np.linalg.inv(camera_matrix), suv_1)
    xyz_c=xyz_c-tvec
    XYZ=np.dot(np.linalg.inv(rot_matrix) , xyz_c)
    return XYZ



# camera_points = np.array([[49, 137.5],
#                  [35, 365],
#                  [506, 200],
#                  [534, 336],
#                  [280,179],
#                  [292,439],
#                  [188,210],
#                  [45,29]] , dtype=np.float32)

# robot_points = np.array([[-263.4, 448.9, 0], # 150
#                  [-297.3, 6, 0], # 156.9
#                  [629.9, 312.5, 0],  # 166.9
#                  [670.7, 929, 0], # 171.3
#                  [195.2, 379.9, 0],#161.9
#                  [205.4, -124.2, 0],
#                  [6.9, 309.9, 0],
#                  [-15.4, 474.7, 0]] , dtype=np.float32) 


# camera_points = np.array([[49, 137.5],
#                  [35, 365],
#                  [506, 200],
#                  [534, 336],
#                  [280,179],
#                  [292,439],
#                  [188,210],
#                  [187.5, 210]]
#               , dtype=np.float32)

# robot_points = np.array([[-263.4, 448.9, 0], # 950
#                  [-297.3, 6, 0], # 156.9
#                  [629.9, 312.5, 0],  # 166.9
#                  [670.7, 929, 0], # 171.3
#                  [195.2, 379.9, 0],#161.9
#                  [205.4, -124.2, 0],
#                  [6.9, 309.9, 0],
#                   [-15.4, 474.7, 0]] , dtype=np.float32) 

# CAMERA CENTER FROM MATRIX (PX) = (576.5, 287.3)
# EQUIVALENT ROBOT LOCATION (MM) = ()

# OTHER COORDINATES TO TEST (in block work area)
# PIXEL: 195, 158 ROBOT: 25.8, 397.9, 148.3
# PIXEL: 164, 187 ROBOT: -35.2, 345.5, 148.3
# PIXEL: 211, 216 ROBOT: 53.9, 282.1, 148.3
# PIXEL: 180, 259 ROBOT: -10.5, 200.3, 148.3
# PIXEL: 163, 168 ROBOT: -36.7, 400.6, 148.3
# PIXEL: 211, 180 ROBOT: 53.6, 355.3, 148.3
# PIXEL: 196, 216 ROBOT: 24.3, 285.9, 148.3
# PIXEL: 163, 260 ROBOT: -39.6, 203.1, 148.3
# (on conveyor belt)
# PIXEL: 308, 207 ROBOT: 251.5, 290, 208
# PIXEL: 270, 261 ROBOT: 177.3, 309.7, 208
# PIXEL: 308, 292 ROBOT: 244.6, 236.4, 208
# PIXEL: 279, 332 ROBOT: 184.7, 61.2, 208
# (on table past block workspace)
# PIXEL: 102, 170 ROBOT: -145.2, 382.9, 148.3
# PIXEL: 1, 223 ROBOT: -265.1, 282.5, 148.3
# PIXEL: 82, 276 ROBOT: -195.2, 178.8, 148.3

camera_points = np.array([[195, 158],
                 [164, 187],
                 [211, 216],
                 [180, 259],
                 [163, 168],
                 [211, 180],
                 [196, 216],
                 [163, 260]]
              , dtype=np.float32)

robot_points = np.array([[25.8, 397.9, 148.3], # 950
                 [-35.2, 345.5, 148.3], # 156.9
                 [53.9, 282.1, 148.3],  # 166.9
                 [-10.5, 200.3, 148.3], # 171.3
                 [-36.7, 400.6, 148.3],#161.9
                 [53.6, 355.3, 148.3],
                 [24.3, 285.9, 148.3],
                  [-39.6, 203.1, 148.3]] , dtype=np.float32) 

# Load calibration data
with np.load('./multiple_img_calibration_data.npz') as data: # change to "multiple_img_calibration_data.npz" before running or if using different data imgs
    mtx = data['mtx']
    dist = data['dist']
    rvecs = data['rvecs']
    tvecs = data['tvecs']
# camera callibration returns camera matrix (contains camera centers x y and focal length x y), distortion coefficients, rotational vectors, translational vectors
    
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 1)
mtx = newcameramtx

# print(mtx, "dddddddddddddddddddddd\n")
# print(dist, "dddddddddddddddddddddd\n")
# print(rvecs, "dddddddddddddddddddddd\n")
# print(tvecs, "dddddddddddddddddddddd\n")

# returns bool value, rotational vector  and translational vector 
# # (success, rotation_vector, translation_vector) = cv2.solvePnP()
success, rvec, tvec = cv2.solvePnP(objectPoints=robot_points, imagePoints=camera_points, cameraMatrix=mtx, distCoeffs=dist) # optionally pass rvec and tvec

# print(rvec)
# print(tvec)

# turn rotational vector into rotational matrix
# NOTE: rot_matrix[0] contains 3x3 rotational matrix, (could just do rot_matrix, _ = cv2.Rodrigues() )
rot_matrix, _ = cv2.Rodrigues(rvec) 

# print("Rotation Matrix:\n", rot_matrix)

homo_matrix = np.hstack((rot_matrix, tvec))
# homo_matrix = np.hstack((real_rotation_matrix, [[0],[0],[0],[1]])) # add column 
# homo_matrix = np.vstack((homo_matrix, [[0,0,0,1]])) # add row

# print("Homogenous Matrix:\n", homo_matrix)


# Example usage of the transformation function
# image_point = [188.5, 210.5]  # Coordinates in the image
# depth = 870  # Depth value in mm, prob not focal length
# robot_coordinates = transform_to_robot_coordinates(image_point, depth, mtx, rvec, tvec)
# X, Y, Z = robot_coordinates
# print(f"Transformed robot coordinates: X={X}, Y={Y}, Z={Z}")

image = [[188.5], [210.5], [0], [1]]   # robot 7.1, 305, 164
robot = [[7.1], [305], [148.3], [1]]

# image = [[308], [207], [0], [1]]
# robot = [[251.5], [290], [208], [1]]

# cam matrix * homo matrix (rotational and translational) * robot coordinate vector
res = (np.dot(np.dot(mtx, homo_matrix) , robot))
print("Formula Output:\n", res)
print("Converted Coordinates: ", res[0][0] / res[2][0], ",", res[1][0] / res[2][0])

# homo matrix (rotational and translational) * pixel coordinate vector
# print(np.dot(homo_matrix , image))

# print("Camera Matrix:\n", mtx)



# print(calculate_XYZ(image[0][0], image[1][0], camera_matrix=mtx, tvec=tvec, rot_matrix=rot_matrix))






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