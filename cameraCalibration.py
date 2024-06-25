# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Perform camera calibration using a chessboard.
 
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
import glob # Used to get retrieve files that have a specified pattern
import pickle # For saving calibration info
 
# Path to the image that you want to undistort
distorted_img_filename = 'Original/22.jpg'
 
# Chessboard dimensions
number_of_squares_X = 10 # Number of chessboard squares along the x-axis
number_of_squares_Y = 7  # Number of chessboard squares along the y-axis
nX = number_of_squares_X - 1 # Number of interior corners along x-axis
nY = number_of_squares_Y - 1 # Number of interior corners along y-axis

square_size = 22.85 # ONE SQUARE LENGTH IN CENTIMETERS
 
# Store vectors of 3D points for all chessboard images (world coordinate frame)
object_points = []
 
# Store vectors of 2D points for all chessboard images (camera coordinate frame)
image_points = []
 
# Set termination criteria. We stop either when an accuracy is reached or when
# we have finished a certain number of iterations.
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# Define real world coordinates for points in the 3D coordinate frame
# Object points are (0,0,0), (1,0,0), (2,0,0) ...., (5,8,0)
object_points_3D = np.zeros((nX * nY, 3), np.float32)       
 
# These are the x and y coordinates                                              
object_points_3D[:,:2] = np.mgrid[0:nY, 0:nX].T.reshape(-1, 2) 
 
object_points_3D = object_points_3D * square_size
 
def main():
        
    # Get the file path for images in the current directory
    images = glob.glob('Original/*.jpg')
        
    # Go through each chessboard image, one by one
    count = 1
    for image_file in images:
        # Load the image
        image = cv2.imread(image_file)  

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  

        # Find the corners on the chessboard
        success, corners = cv2.findChessboardCorners(gray, (nY, nX), None)
            
        # If the corners are found by the algorithm, draw them
        if success == True:

            # Append object points
            object_points.append(object_points_3D)

            # Find more exact corner pixels       
            corners_2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)       
            
            # Append image points
            image_points.append(corners_2)

            # Draw the corners
            cv2.drawChessboardCorners(image, (nY, nX), corners_2, success)

            # Display the image. Used for testing.
            #cv2.imshow("Image", image) 
            
            # Display the window for a short period. Used for testing.
            #cv2.waitKey(200) 
                                                                                                                        
        # Now take a distorted image and undistort it 
        distorted_image = cv2.imread(image_file)

        # Perform camera calibration to return the camera matrix, distortion coefficients, rotation and translation vectors etc 
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, 
                                                        image_points, 
                                                        gray.shape[::-1], 
                                                        None, 
                                                        None)

        # Get the dimensions of the image 
        height, width = distorted_image.shape[:2]
            
        # Refine camera matrix
        # Returns optimal camera matrix and a rectangular region of interest
        optimal_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, 
                                                                (width,height), 
                                                                1, 
                                                                (width,height))

        # Undistort the image
        undistorted_image = cv2.undistort(distorted_image, mtx, dist, None, 
                                        optimal_camera_matrix)

        # Create the output file name by removing the '.jpg' part
        size = len(image_file)
        new_filename = image_file[9:size - 4]
        new_filename = f'Undistorted/{new_filename}_undistorted.jpg'
        print(new_filename)
        count = count + 1

        undistorted_image = cv2.putText(undistorted_image, f'{image_file[9:]}',
                                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (255, 0, 0), 2)

        # Save the undistorted image
        cv2.imwrite(new_filename, undistorted_image)

    # Crop the image. Uncomment these two lines to remove black lines
    # on the edge of the undistorted image.
    #x, y, w, h = roi
    #undistorted_image = undistorted_image[y:y+h, x:x+w]
        
    # Display key parameter outputs of the camera calibration process
    # print("Camera matrix:")
    # print(mtx)

    # print("Optimal Camera matrix:") 
    # print(optimal_camera_matrix) 

    # print("\n Distortion coefficient:") 
    # print(dist) 

    # print("\n Rotation Vectors:") 
    # print(rvecs) 

    # print("\n Translation Vectors:") 
    # print(tvecs) 


    # Save the camera calibration results using pickle
    calib_result_pickle = {}
    calib_result_pickle["mtx"] = mtx
    calib_result_pickle["optimal_camera_matrix"] = optimal_camera_matrix
    calib_result_pickle["dist"] = dist
    calib_result_pickle["rvecs"] = rvecs
    calib_result_pickle["tvecs"] = tvecs
    pickle.dump(calib_result_pickle, open("camera_calib_pickle.p", "wb" ))

    # Close all windows
    cv2.destroyAllWindows() 
     
main()

#TO UPLOAD AND USE CALIBRATION IN NEW FILE

# calib_result_pickle = pickle.load(open("camera_calib_pickle.p", "rb" ))
# mtx = calib_result_pickle["mtx"]
# optimal_camera_matrix = calib_result_pickle["optimal_camera_matrix"]
# dist = calib_result_pickle["dist"]

# undistorted_image = cv2.undistort(distorted_image, mtx, dist, None, 
#                                   optimal_camera_matrix)
