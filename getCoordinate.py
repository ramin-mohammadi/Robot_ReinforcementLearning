
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
# Example usage of the transformation function
image_point = (320, 240)  # Coordinates in the image
depth = 500  # Depth value in mm

robot_coordinates = transform_to_robot_coordinates(image_point, depth, mtx, rvec, tvec)
X, Y, Z = robot_coordinates

print(f"Transformed robot coordinates: X={X}, Y={Y}, Z={Z}")
