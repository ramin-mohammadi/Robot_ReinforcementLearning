import numpy as np
import cv2 as cv
import glob

# Define the chessboard dimensions and square size in millimeters
chessboard_size = (6, 9)  # number of internal corners in the chessboard pattern (9x6 for this example)
square_size = 25  # Size of a square in millimeters (e.g., 25 mm)

# # Prepare object points based on the real-world coordinates of the chessboard corners
# objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
# objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
# objp *= square_size

# # Arrays to store object points and image points from all the images
# objpoints = []  # 3d points in real world space
# imgpoints = []  # 2d points in image plane

# Read all images in the calibration directory
images = glob.glob('REU Robotic Arm\calibration_images\*.jpg')

# if not images:
#     print("No images found in the specified directory.")
#     exit()

# for fname in images:
#     img = cv.imread(fname)
#     if img is None:
#         print(f"Failed to load image {fname}")
#         continue
    
#     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

#     # Find the chessboard corners
#     ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)

#     # If found, add object points, image points (after refining them)
#     if ret:
#         objpoints.append(objp)
#         corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
#                                    (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001))
#         imgpoints.append(corners2)

#         # Draw and display the corners
#         cv.drawChessboardCorners(img, chessboard_size, corners2, ret)
#         cv.imshow('img', img)
#         cv.waitKey(500)
#     else:
#         print(f"Chessboard corners not found in image {fname}")

# cv.destroyAllWindows()

# if not objpoints or not imgpoints:
#     print("No corners were detected in any image. Calibration failed.")
#     exit()

# # Camera calibration
# ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# # Save the calibration results
# np.savez('calibration_data1.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

# # Print out the camera calibration results
# print("Camera matrix:\n", mtx)
# print("Distortion coefficients:\n", dist)

# # Calculate re-projection error
# mean_error = 0
# for i in range(len(objpoints)):
#     imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
#     error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
#     mean_error += error

# print("Total re-projection error: {}".format(mean_error / len(objpoints)))

# Calculate SCALE_FACTOR
# Using one of the calibration images to determine the average size of the squares in pixels
image = cv.imread(images[0])
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)

if ret:
    corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                               (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    square_size_in_pixels = np.mean([cv.norm(corners2[i] - corners2[i + 1]) for i in range(chessboard_size[0] - 1)])
    SCALE_FACTOR = (square_size/ square_size_in_pixels)*2  #multiply by 2 if needed
    print(f"SCALE_FACTOR: {SCALE_FACTOR:.4f} mm per pixel")
else:
    print("Failed to find corners in the sample image for SCALE_FACTOR calculation.")
    exit()

# Object detection and measurement code
def get_color_mask(hsv, lower_color, upper_color):
    mask = cv.inRange(hsv, lower_color, upper_color)
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)
    return mask

def detect_objects_by_color(frame):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Define color ranges
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    lower_blue = np.array([94, 80, 2])
    upper_blue = np.array([126, 255, 255])
    lower_green = np.array([25, 52, 72])
    upper_green = np.array([102, 255, 255])
    
    # Create masks
    mask_red = get_color_mask(hsv, lower_red, upper_red)
    mask_blue = get_color_mask(hsv, lower_blue, upper_blue)
    mask_green = get_color_mask(hsv, lower_green, upper_green)
    
    return mask_red, mask_blue, mask_green

def find_objects(mask, label, color):
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    objects = []
    
    for contour in contours:
        if cv.contourArea(contour) > 500:  # Filter small objects
            rect = cv.minAreaRect(contour)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            center_x, center_y = rect[0]
            width, height = rect[1]
            angle = rect[2]
            
            # Adjust angle to start from the x-axis (0 degrees)
            if width < height:
                angle = 90 + angle
            else:
                angle = angle
            
            # Convert pixel values to millimeters
            center_x_mm = center_x * SCALE_FACTOR
            center_y_mm = center_y * SCALE_FACTOR
            width_mm = width * SCALE_FACTOR
            height_mm = height * SCALE_FACTOR
            
            objects.append((label, (center_x_mm, center_y_mm), angle, box, color))
    
    return objects

def draw_objects(frame, objects):
    for obj in objects:
        label, center, angle, box, color = obj
        center_x, center_y = int(center[0] / SCALE_FACTOR), int(center[1] / SCALE_FACTOR)
        cv.drawContours(frame, [box], 0, color, 2)
        cv.circle(frame, (center_x, center_y), 5, (0, 0, 0), -1)  # Center point in black

        # Calculate the endpoint for the angle line
        length = 50  # Length of the angle line in pixels
        angle_rad = np.deg2rad(angle)
        end_x = int(center_x + length * np.cos(angle_rad))
        end_y = int(center_y + length * np.sin(angle_rad))
        
        # Draw the angle line
        cv.line(frame, (center_x, center_y), (end_x, end_y), (0, 0, 0), 2)

        # Display real-world units
        cv.putText(frame, label, (center_x, center_y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv.putText(frame, f'Angle: {angle:.2f}', (center_x, center_y + 20), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        cv.putText(frame, f'Center: ({center[0]:.2f}, {center[1]:.2f}) mm', (center_x, center_y + 40), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        print(f'{label} center: ({center[0]:.2f} mm, {center[1]:.2f} mm)')

def process_frame(frame, mtx, dist, newcameramtx, roi):
    # Undistort the frame
    dst = cv.undistort(frame, mtx, dist, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]

    # Now process the undistorted frame
    mask_red, mask_blue, mask_green = detect_objects_by_color(dst)
    
    red_objects = find_objects(mask_red, "red_lego", (0, 0, 255))
    blue_objects = find_objects(mask_blue, "blue_lego", (255, 0, 0))
    green_objects = find_objects(mask_green, "green_lego", (0, 255, 0))
    
    all_objects = red_objects + blue_objects + green_objects
    
    draw_objects(dst, all_objects)
    
    return dst

if __name__ == "__main__":
    # Load calibration data
    with np.load('calibration_data1.npz') as data:
        mtx = data['mtx']
        dist = data['dist']
        rvecs = data['rvecs']
        tvecs = data['tvecs']

    # Get optimal new camera matrix
    image = cv.imread(images[0])
    h, w = image.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    
    # Open the video file
    cap = cv.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open video file.")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        frame = process_frame(frame, mtx, dist, newcameramtx, roi)
        
        cv.imshow("Frame", frame)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


# if __name__ == "__main__":
#     cap = cv.VideoCapture(0)  # Open webcam or USB camera

#     if not cap.isOpened():
#         print("Error: Could not open webcam.")
#         exit()

#     # Load the calibration data
#     with np.load('calibration_data.npz') as data:
#         mtx = data['mtx']
#         dist = data['dist']

#     h, w = None, None
#     newcameramtx, roi = None, None

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("Error: Could not read frame.")
#             break

#         if h is None or w is None:
#             h, w = frame.shape[:2]
#             newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

#         frame = process_frame(frame, mtx, dist, newcameramtx, roi)

#         cv.imshow("Frame", frame)

#         if cv.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv.destroyAllWindows()