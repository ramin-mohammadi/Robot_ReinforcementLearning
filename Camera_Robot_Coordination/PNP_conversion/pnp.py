import cv2
import numpy as np


# # Function to transform image coordinates to robot coordinates
# def transform_to_robot_coordinates(image_point, depth, mtx, rotation_vector, translation_vector):
#     # Convert image point to homogeneous coordinates
#     image_coordinates = np.array([[image_point[0]], [image_point[1]], [1]], dtype=np.float32)

#     # Calculate camera coordinates (assuming depth in mm)
#     camera_coordinates = np.linalg.inv(mtx).dot(image_coordinates) * depth

#     # Convert rotation vector to rotation matrix
#     rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

#     # Apply the extrinsic transformation to get robot coordinates
#     robot_coordinates = rotation_matrix.dot(camera_coordinates) + translation_vector

#     return robot_coordinates.ravel()


# https://medium.com/@pacogarcia3/calculate-x-y-z-real-world-coordinates-from-image-coordinates-using-opencv-from-fdxlabs-0adf0ec37cef
def calculate_XYZ(u,v, camera_matrix, tvec, rot_matrix):
                                      
    #Solve: From Image Pixels, find World Points

    uv_1=np.array([[u,v,1]], dtype=np.float32)
    uv_1=np.transpose(uv_1)
    # scaling_factor = 0.56 # divide to convert pixel to mm, 0.56
    px_to_mm = 0.526
    suv_1=uv_1 * px_to_mm # turn pixel coordinates to mm
    xyz_c=np.dot(np.linalg.inv(camera_matrix), suv_1)
    xyz_c=xyz_c-tvec
    XYZ=np.dot(np.linalg.inv(rot_matrix) , xyz_c)
    return XYZ

# def new_calculate_XYZ(image_vector, camera_matrix, homo_matrix):
#     # image_vector = [[u], [v], [0]]
#     homo_matrix = np.vstack((homo_matrix, [[0,0,0,1]])) # add row, turns homo into a 4x4 square matrix allowing inverse operation

#     return np.dot( np.linalg.inv(homo_matrix) ,  image_vector)

# def new_new_calculate_XYZ(image_vector, camera_matrix, homo_matrix):
#     s = 0.526
#     return np.dot(image_vector , np.dot(np.linalg.inv(camera_matrix) , s * np.linalg.inv(homo_matrix)))
    
    
"""
-Have camera and homography matrix be 4x4 and pixel coordinate vector be 4x1
-assuming 
A = camera matrix
B = homography matrix
p = pixel vector
r = robot vector

Normal Formula
p = A * B * r

Inverted (turn A and B into 4x4 sqaure matrices then multiply both sides by A inverse and then B inverse) -> Note A * A^-1 = Identity matrix
r = B^-1 * A^-1 * p
"""
def jin_calculate_XYZ(camera_matrix: np.ndarray, homo_matrix: np.ndarray, image_vector: np.ndarray):
    # turn camera matrix of 3x3 into 4x4
    camera_matrix = np.hstack((camera_matrix, [[0],[0],[0]])) # add column
    camera_matrix = np.vstack((camera_matrix, [[0,0,0,1]])) # add row
    camera_matrix_inv = np.linalg.inv(camera_matrix)
    
    # turn homography matrix from 3x4 into 4x4
    homo_matrix = np.vstack((homo_matrix, [[0,0,0,1]])) # add row
    homo_matrix_inv = np.linalg.inv(homo_matrix) 
    
    XYZ = np.matmul(homo_matrix_inv, camera_matrix_inv)
    XYZ = np.matmul(XYZ, image_vector) # IMPORTANT: image vector is expected to be [[pixel_x],[pixel_y],[1],[1]]
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
             
#                  [188,210],
#                  [45,29]] , dtype=np.float32)

# robot_points = np.array([[-263.4, 448.9, 150], # 150
#                  [-297.3, 6, 156.9], # 156.9
#                  [629.9, 312.5, 166.9],  # 166.9
#                  [670.7, 929, 171.3], # 171.3
              
#                  [6.9, 309.9, 153],
#                  [-15.4, 474.7, 152]] , dtype=np.float32) 


# ALbert
# camera_points = np.array([[195, 158],
#                  [164, 187],
#                  [211, 216],
#                  [180, 259],
#                  [163, 168],
#                  [211, 180],
#                  [196, 216],
#                  [163, 260]]
#               , dtype=np.float32)

# robot_points = np.array([[25.8, 397.9, 148.3], # 950
#                  [-35.2, 345.5, 148.3], # 156.9
#                  [53.9, 282.1, 148.3],  # 166.9
#                  [-10.5, 200.3, 148.3], # 171.3
#                  [-36.7, 400.6, 148.3],#161.9
#                  [53.6, 355.3, 148.3],
#                  [24.3, 285.9, 148.3],
#                   [-39.6, 203.1, 148.3]] , dtype=np.float32) 



# new cam res
camera_points = np.array([[962, 536] , # center from camera matrix cx cy
                [957, 751],
                [308, 372],
                 [139, 826],
                 [1489, 372],
                 [1537, 705],
                 ], dtype=np.float32)


robot_points = np.array([[242.8, 181.5, 224.2], # robot for camera center
                         [242.8, 39.2, 224],
                [-245, 292.9, 148.3], # 950
                 [-367.7, -39, 148.3], # 156.9
                 [626.1, 306.3, 148.3],  # 166.9
                 [659.9, 58.4, 148.3], # 171.3
               ] , dtype=np.float32) 





# print(cv2.findHomography(srcPoints=camera_points, dstPoints=robot_points, method=cv2.CV_RANSAC))
# exit()

# camera_points = np.array([[49, 137.5],
#                  [35, 365],
#                  [506, 200],
#                  [534, 336],
#                  [280,179],
#                  [292,439],
#                  [188,210],
#                  [187.5, 210]]
#               , dtype=np.float32)

# robot_points = np.array([[-263.4, 448.9, 0], # 150
#                  [-297.3, 6, 0], # 156.9
#                  [629.9, 312.5, 0],  # 166.9
#                  [670.7, 929, 0], # 171.3
#                  [195.2, 379.9, 0],#161.9
#                  [205.4, -124.2, 0],
#                  [6.9, 309.9, 0],
#                   [-15.4, 474.7, 0]] , dtype=np.float32) 

# camera_points = np.array([[172.5, 145],
#                  [169, 202],
#                  [205, 191],
#                  [205,237],
#                  [169, 251],
                 
                 
#                 #  [49, 137.5],
#                 #  [35, 365],
#                 #  [506, 200],
#                 #  [534, 336],
#                 #  [280,179],
#                 #  [292,439],
#                 #  [188,210],
#                 #  [45,29]
#                 ]
#               , dtype=np.float32)

# robot_points = np.array([[-17.4, 435, 154], # 150
#                  [-23.3, 322, 154], # 156.9
#                  [44.9, 342, 154],  # 166.9
#                  [41, 248, 154], # 171.3
#                  [-29, 219, 154],#161.9
                 
                 
#                 #  [-263.4, 448.9, 0], # 150
#                 #  [-297.3, 6, 0], # 156.9
#                 #  [629.9, 312.5, 0],  # 166.9
#                 #  [670.7, 929, 0], # 171.3
#                 #  [195.2, 379.9, 0],#161.9
#                 #  [205.4, -124.2, 0],
#                 #  [6.9, 309.9, 0],
#                 #  [-15.4, 474.7, 0]
#     ] , dtype=np.float32) 


"""
# Load calibration data
# camera callibration returns camera matrix (contains camera centers x y and focal length x y), distortion coefficients, rotational vectors, translational vectors
# change to "multiple_img_calibration_data.npz" before running or if using different data imgs
"""
with np.load('C:/RaminMohammadi_REU2024/testingArm/xArm-Python-SDK/two_agent/62cmfromconveyor_calibration_data.npz') as data: 
# with np.load('C:/RaminMohammadi_REU2024/testingArm/xArm-Python-SDK/two_agent/old_camera/multiple_img_calibration_data.npz') as data: 
    mtx = data['mtx']
    dist = data['dist']
    rvecs = data['rvecs']
    tvecs = data['tvecs']
    
# print(mtx, "dddddddddddddddddddddd\n")
# print(dist, "dddddddddddddddddddddd\n")
# print(rvecs, "dddddddddddddddddddddd\n")
# print(tvecs, "dddddddddddddddddddddd\n")


"""
returns bool value, rotational vector  and translational vector 
(success, rotation_vector, translation_vector) = cv2.solvePnP()
"""
success, rvec, tvec = cv2.solvePnP(objectPoints=robot_points, imagePoints=camera_points, cameraMatrix=mtx, distCoeffs=dist) # optionally pass rvec and tvec from camera calibration

print("Rotational Vector:\n", rvec)
print("Translational Vector:\n", tvec)

# turn rotational vector into rotational matrix
# NOTE: rot_matrix[0] contains 3x3 rotational matrix, (could just do rot_matrix, _ = cv2.Rodrigues() )
rot_matrix, _ = cv2.Rodrigues(rvec) 

print("Rotational matrix: \n", rot_matrix)

homo_matrix = np.hstack((rot_matrix, tvec)) # adds translational vecotr as a column to rotational matrix
# homo_matrix = np.hstack((real_rotation_matrix, [[0],[0],[0],[1]])) # add column 
# homo_matrix = np.vstack((homo_matrix, [[0,0,0,1]])) # add row

print("Homogenous matrix: \n", homo_matrix)


# Example usage of the transformation function
# image_point = [188.5, 210.5]  # Coordinates in the image
# depth = 870  # Depth value in mm, prob not focal length
# robot_coordinates = transform_to_robot_coordinates(image_point, depth, mtx, rvec, tvec)
# X, Y, Z = robot_coordinates
# print(f"Transformed robot coordinates: X={X}, Y={Y}, Z={Z}")

# image = [[188.5], [210.5], [0], [1]]   # robot 7.1, 305, 164
# robot = [[7.1], [305], [148.3], [1]] # robot's z cannot be 0 or will lead to bad results, output: (193, 207)

image = [[609.9], [451], [0], [1]]   # formula ouputs: 628, 447, 622
robot = [[-20.9], [244], [148.3], [1]] # robot's z cannot be 0 or will lead to bad results, output: (193, 207)

image = [[609.9], [451], [0], [1]]   # formula ouputs: 628, 447
robot = [[-20.9], [244], [148.3], [1]] # robot's z cannot be 0 or will lead to bad results, output: (193, 207)


# generated camera center is wrong, manually declare center in camera matrix (camera is 640 x 480)
# mtx[0, 2] = 320
# mtx[1, 2] = 240

# cam matrix * homo matrix (rotational and translational) * pixel coordinate vector
print("Formula: ")
print(np.dot(np.dot(mtx, homo_matrix) , robot))

# homo matrix (rotational and translational) * pixel coordinate vector
# print(np.dot(homo_matrix , image))

print("Camera Matrix: ", mtx,"\n")




print("Medium Article Solution:\n", calculate_XYZ(u=image[0][0], v=image[1][0], camera_matrix=mtx, tvec=tvec, rot_matrix=rot_matrix))
print(calculate_XYZ(u=309, v=370, camera_matrix=mtx, tvec=tvec, rot_matrix=rot_matrix))

print("\n\n")
# print(  np.dot( np.linalg.inv(mtx),     new_calculate_XYZ(image_vector=image, camera_matrix=mtx, homo_matrix=homo_matrix)[:3, ]  )   )
# print("\n\n")
# homo_matrix = np.vstack((homo_matrix, [[0,0,0,1]])) # add row
# print("\n\nHomo * image: ")
# print(np.dot(homo_matrix, image))
# print(np.dot([image[0][0], image[1][0], 0], homo_matrix))

image = [[609.9], [451], [1], [1]]   # formula ouputs: 628, 447
robot = [[-20.9], [244], [148.3], [1]] # robot's z cannot be 0 or will lead to bad results, output: (193, 207)

# homo_matrix = np.vstack((homo_matrix, [[0,0,0,1]])) # add row
# print("FINAL:\n", new_new_calculate_XYZ(image_vector=image, camera_matrix=mtx, homo_matrix=homo_matrix))



# IMPORTANT: Either camera matrix and distortion coefficients is bad, or sample points are bad -> 
# MOST LIKELY Camera calibration was done incorectly leading to bad camera matrix and distortion coefficients which are inputs to solvePnp()
print("DR JIN solution:\n", jin_calculate_XYZ(image_vector=image, camera_matrix=mtx, homo_matrix=homo_matrix))