import numpy as np
import cv2 as cv
import glob

# Define the chessboard dimensions
chessboard_size = (6, 9)  # number of internal corners in the chessboard pattern (9x6 for this example)
square_size = 2.3  # Define size of a square (arbitrary units, e.g., 1.0 for 1x1 unit squares) 23mm 2.3cm

# #Grid Paper
# chessboard_size = (16, 20)  # number of internal corners in the chessboard pattern (9x6 for this example)
# square_size = 1  # Define size of a square (arbitrary units, e.g., 1.0 for 1x1 unit squares) 23mm 2.3cm

# Prepare object points based on the real-world coordinates of the chessboard corners
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all the images
objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane

# Read all images in the calibration directory
images = glob.glob('REU Robotic Arm\calibration_images\\test_calibration_images\*.jpg')


for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                   (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the calibration results
np.savez('multiple_img_calibration_data.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

# Calculate re-projection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
    mean_error += error

print("Total re-projection error: {}".format(mean_error / len(objpoints)))

# Print out the camera calibration results
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)

# Undistort a sample image to verify calibration
img = cv.imread('REU Robotic Arm\calibration_images\\test_calibration_images\WIN_20240626_09_57_26_Pro.jpg')
h, w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# Undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# Crop and display the image
x, y, w, h = roi
dst = dst[y:y + h, x:x + w]
cv.imshow('calibrated_image', dst)
cv.waitKey(0)
cv.destroyAllWindows()