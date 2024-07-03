import cv2
import numpy as np

# UPDATED CODE : JULY 3

# red obj detection, coordinates, angle
# camera calibration
# 1920x1080 resolution

# can adjust output to be in pxl or mm
# NO BLUE/GREEN DETECTION


video_cap = cv2.VideoCapture(0)

# SAVING VIDEO TO FILES:
# saves to video to files - overwrites each time

# Webcam max resolution: 1920 x 1080 (1080p)
# Max fps: 60ps
resolution_width = 1920
resolution_height = 1080
frameSize = (resolution_width, resolution_height)
fps = 60

video_cap.set(3, resolution_width)
video_cap.set(4, resolution_height)

fourcc = cv2.VideoWriter_fourcc(*'XVID')

vid_output = cv2.VideoWriter('block_measure.avi', fourcc, fps=fps, frameSize=frameSize)



# Load calibration data
with np.load('C:/Users/karla/Downloads/62cmfromconveyor_calibration_data.npz') as data:
    # Best to worse:
    # multiple_img_calibration_data.npz --> test_calibration folder
    # cropped_chess_calibration_data.npz
    # semi_cropped_calibration_data.npz

    mtx = data['mtx']
    dist = data['dist']
    rvecs = data['rvecs']
    tvecs = data['tvecs']

while True:
    # while ret = true,
    # reads frames from video_cap, frame = real time video
    ret, frame = video_cap.read()

    # # Undistort the frame
    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    # convert to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define color ranges for specific detection of that block
    # mask for red
    # higher end of spectrum - red range
    red_lower = np.array([0, 50, 50], np.uint8)
    red_upper = np.array([10, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsv, red_lower, red_upper)

    # blue mask - og , s val = 80
    blue_lower = np.array([110, 80, 2], np.uint8)
    blue_upper = np.array([115, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    # mask for olive green lego platform
    green_lower = np.array([40, 52, 72], np.uint8)
    green_upper = np.array([70, 200, 255], np.uint8)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)


# USE IF NEED DISPLAY IN MM, else ignore
    # CONVERSION 1 pxl = 0.687692 mm
    # if converting pxl to mm, multiply

    mm = 0.687692





    kernl = np.ones((7, 7), "uint8")

    # increases/smooths white in red mask
    # binarizing the image
    red_mask = cv2.dilate(red_mask, kernl)
    # mask specifies which pixels in the frame image
    # parts of image that were not detected as blue, green, or red, are set to black
    res_red = cv2.bitwise_and(frame, frame, mask=red_mask)

    # for blue
    blue_mask = cv2.dilate(blue_mask, kernl)
    res_blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

    # for green
    green_mask = cv2.dilate(green_mask, kernl)
    res_green = cv2.bitwise_and(frame, frame, mask=green_mask)

    # contours to track red objects
    # detects contours in binary red_mask img
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Cameras center lines & dot
    # horizontal line
    cv2.line(frame,(0,801),(1920,801),(255,255,255),1)
    # verticle line
    cv2.line(frame,(620,0),(620,1080),(255,255,255),1)


    # Marker 1 ( indicated on tape)
    cv2.circle(frame, (220, 270), 4, (0, 0, 255), -1)
    # Marker 2 ( indicated on tape)
    cv2.circle(frame, (220, 810), 4, (0, 0, 255), -1)
    # Marker 3 ( indicated on tape)
    cv2.circle(frame, (1500, 270), 4, (0, 0, 255), -1)
    # Marker 1 ( indicated on tape)
    cv2.circle(frame, (1500, 810), 4, (0, 0, 255), -1)

# Center of
    cv2.circle(frame, (1500, 810), 10, (0, 0, 255), -1)





    # iterate through contours in each color mask
    red = "No red block detected"
    for i, contour in enumerate(contours):
        area_shape = cv2.contourArea(contour)
        # threshold of 400 to eliminate objects smaller than area of 400 pixels
        # static frame
        if (area_shape > 400):
            # pixels
            x, y, w, h = cv2.boundingRect(contour)

# IF NEED DISPLAY IN MM, USE THESE VARIABLES, replace x,y,w,h in center coord with variables ?_mm
            # variable for each point converted to mm
            x_mm = x/mm
            y_mm = y/mm
            w_mm = w/mm
            h_mm = h/mm

            # CENTER COORDINATES
            cv2.putText(frame, f' ({((x + w / 2)):.1f},{((y + h / 2)):.1f})', ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 0, 0))

            # dot in center of camera grid
            cv2.circle(frame, (x + int(w / 2), y + int(h / 2)), 4, (0, 0, 0), -1)


            # start point = top left corner, end point = bottom right corner
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 1)

            # frame to follow with angle
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

            angle = rect[2]

            # determine the angle the robot needs to rotate gripper
            if angle < 45:
                rot_angle = 0 - angle
            else:
                rot_angle = 90 - angle

            # LABEL OF FRAME FOR RED OBJECT
            cv2.putText(frame, "Red block", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))

            # LABEL FOR DEGREES ROBOT NEED TO ROTATE HAND- THIS IS NOT AT WHAT DEGREE THE BLOCK IS AT
            cv2.putText(frame, f" Rotate: {(rot_angle):.0f} degrees  ", (x, (y + h) + 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0))

            # visual add ons #

            # insert green line from corner of outside frame to center (x,y)
            cv2.line(frame, (x, y + h), (x + int(w / 2), y + int(h / 2)), (0, 255, 0), 1, )

            # INFO DISPLAYED IN CONSOLE
            # print(f'Red block angle:{angle}.Robot must rotate {(rot_angle):.0f} degrees')
#            red = f'Red Block center coord:({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})'

    # INFO DISPLAYED IN CONSOLE
#    print(f'Red block angle:{angle}.Robot must rotate {(rot_angle):.0f} degrees')
#    red = f'Red Block center coord:({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})'

    # print(red)


    # output that frame
    # hsv_vid_output.write(hsv)

    vid_output.write(frame)

    # show hsv vid
    # cv2.imshow('frame',hsv)
    cv2.imshow('multiple color detection', frame)

    #     :)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_cap.release()
vid_output.release()




