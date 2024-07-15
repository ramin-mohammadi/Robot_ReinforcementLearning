import cv2
import numpy as np

def get_mm(x_px, y_px):
    # NOTE: camera is rotated 90 degrees, so camera x = robot y etc.
    # There is probably a more efficient way to go about this, sorry.
    if(400 > y_px or y_px > 585):
        return -1, -1 # not within x (robot) bounds
    if y_px <= 585:
        ceiling = 585
        floor = 567
        new_x = 250
    if y_px <= 567:
        ceiling = floor
        floor = 547
        new_x = 240
    if y_px <= 547:
        ceiling = floor
        floor = 529
        new_x = 230
    if y_px <= 529:
        ceiling = floor
        floor = 510
        new_x = 220
    if y_px <= 510:
        ceiling = floor
        floor = 492
        new_x = 210
    if y_px <= 492:
        ceiling = floor
        floor = 474
        new_x = 200
    if y_px <= 474:
        ceiling = floor
        floor = 455
        new_x = 190
    if y_px <= 455:
        ceiling = floor
        floor = 436
        new_x = 180
    if y_px <= 436:
        ceiling = floor
        floor = 419
        new_x = 170
    if y_px <= 419:
        ceiling = floor
        floor = 400
        new_x = 160

    adjust_x = 10 * (y_px - floor) / (ceiling - floor)
    mm_x = new_x + adjust_x # within cm adjustment based on px value
    
    if(790 > x_px or x_px > 1230):
        return -1, -1 # not within y (robot) bounds
    if x_px <= 1230:
        ceiling = 1230
        floor = 1210
        new_y = 250
    if x_px <= 1210:
        ceiling = floor
        floor = 1192
        new_y = 240
    if x_px <= 1192:
        ceiling = floor
        floor = 1174
        new_y = 230
    if x_px <= 1174:
        ceiling = floor
        floor = 1155
        new_y = 220
    if x_px <= 1155:
        ceiling = floor
        floor = 1137
        new_y = 210
    if x_px <= 1137:
        ceiling = floor
        floor = 1119
        new_y = 200
    if x_px <= 1119:
        ceiling = floor
        floor = 1100
        new_y = 190
    if x_px <= 1100:
        ceiling = floor
        floor = 1081
        new_y = 180
    if x_px <= 1081:
        ceiling = floor
        floor = 1063
        new_y = 170
    if x_px <= 1063:
        ceiling = floor
        floor = 1045
        new_y = 160
    if x_px <= 1045:
        ceiling = floor
        floor = 1026
        new_y = 150
    if x_px <= 1026:
        ceiling = floor
        floor = 1008
        new_y = 140
    if x_px <= 1008:
        ceiling = floor
        floor = 989
        new_y = 130
    if x_px <= 989:
        ceiling = floor
        floor = 971
        new_y = 120
    if x_px <= 971:
        ceiling = floor
        floor = 953
        new_y = 110
    if x_px <= 953:
        ceiling = floor
        floor = 934
        new_y = 100
    if x_px <= 934:
        ceiling = floor
        floor = 917
        new_y = 90
    if x_px <= 917:
        ceiling = floor
        floor = 898
        new_y = 80
    if x_px <= 898:
        ceiling = floor
        floor = 880
        new_y = 70
    if x_px <= 880:
        ceiling = floor
        floor = 862
        new_y = 60
    if x_px <= 862:
        ceiling = floor
        floor = 844
        new_y = 50
    if x_px <= 844:
        ceiling = floor
        floor = 826
        new_y = 40
    if x_px <= 826:
        ceiling = floor
        floor = 808
        new_y = 30
    if x_px <= 808:
        ceiling = floor
        floor = 790
        new_y = 20

    adjust_y = 10 * (x_px - floor) / (ceiling - floor)
    mm_y = new_y + adjust_y # within cm adjustment based on px value

    return mm_x, mm_y

def draw_dots(frame):
    # NOTE: Function for drawing all static lines/dots, as to not clutter the other functions

    # Cameras center lines & dot
    # cv2.line(frame,(0,240),(640,240),(255,255,255),1)
    # cv2.line(frame,(320,0),(320,480),(255,255,255),1)
    cv2.circle(frame,(320,240),4,(0,0,255),-1)
    cv2.circle(frame, (210, 120), 4, (0, 0, 255), -1)
    cv2.circle(frame, (430, 360), 4, (0, 0, 255), -1)
    cv2.circle(frame, (210, 360), 4, (0, 0, 255), -1)
    cv2.circle(frame, (430, 120), 4, (0, 0, 255), -1)

    cv2.line(frame, (0, 600), (1920, 600), (0, 0, 255))

    # x-axes dots
    cv2.circle(frame, (1230, 585), 3, (255, 0, 0), -1) # x = 260
    cv2.circle(frame, (1230, 567), 3, (255, 0, 0), -1) # x = 250
    cv2.circle(frame, (1230, 547), 3, (255, 0, 0), -1) # x = 240
    cv2.circle(frame, (1230, 529), 3, (255, 0, 0), -1) # x = 230
    cv2.circle(frame, (1230, 510), 3, (255, 0, 0), -1) # x = 220
    cv2.circle(frame, (1230, 492), 3, (255, 0, 0), -1) # x = 210
    cv2.circle(frame, (1230, 474), 3, (255, 0, 0), -1) # x = 200
    cv2.circle(frame, (1230, 455), 3, (255, 0, 0), -1) # x = 190
    cv2.circle(frame, (1230, 436), 3, (255, 0, 0), -1) # x = 180
    cv2.circle(frame, (1230, 419), 3, (255, 0, 0), -1) # x = 170
    cv2.circle(frame, (1230, 400), 3, (255, 0, 0), -1) # x = 160

    # y-axes dots
    cv2.circle(frame, (1230, 600), 3, (0, 0, 255), -1) # y = 260
    cv2.circle(frame, (1210, 600), 3, (0, 0, 255), -1) # y = 250
    cv2.circle(frame, (1192, 600), 3, (0, 0, 255), -1) # y = 240
    cv2.circle(frame, (1174, 600), 3, (0, 0, 255), -1) # y = 230
    cv2.circle(frame, (1155, 600), 3, (0, 0, 255), -1) # y = 220
    cv2.circle(frame, (1137, 600), 3, (0, 0, 255), -1) # y = 210
    cv2.circle(frame, (1119, 600), 3, (0, 0, 255), -1) # y = 200
    cv2.circle(frame, (1100, 600), 3, (0, 0, 255), -1) # y = 190
    cv2.circle(frame, (1081, 600), 3, (0, 0, 255), -1) # y = 180
    cv2.circle(frame, (1063, 600), 3, (0, 0, 255), -1) # y = 170
    cv2.circle(frame, (1045, 600), 3, (0, 0, 255), -1) # y = 160
    cv2.circle(frame, (1026, 600), 3, (0, 0, 255), -1) # y = 150
    cv2.circle(frame, (1008, 600), 3, (0, 0, 255), -1) # y = 140
    cv2.circle(frame, (989, 600), 3, (0, 0, 255), -1) # y = 130
    cv2.circle(frame, (971, 600), 3, (0, 0, 255), -1) # y = 120
    cv2.circle(frame, (953, 600), 3, (0, 0, 255), -1) # y = 110
    cv2.circle(frame, (934, 600), 3, (0, 0, 255), -1) # y = 100
    cv2.circle(frame, (917, 600), 3, (0, 0, 255), -1) # y = 90
    cv2.circle(frame, (898, 600), 3, (0, 0, 255), -1) # y = 80
    cv2.circle(frame, (880, 600), 3, (0, 0, 255), -1) # y = 70
    cv2.circle(frame, (862, 600), 3, (0, 0, 255), -1) # y = 60
    cv2.circle(frame, (844, 600), 3, (0, 0, 255), -1) # y = 50
    cv2.circle(frame, (826, 600), 3, (0, 0, 255), -1) # y = 40
    cv2.circle(frame, (808, 600), 3, (0, 0, 255), -1) # y = 30
    cv2.circle(frame, (790, 600), 3, (0, 0, 255), -1) # y = 20

def main():
    video_cap = cv2.VideoCapture(0)

    # SAVING VIDEO TO FILES:
    # codec for windows = DIVX
    # saves to video to files - overwrites each time
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # hsv_vid_output = cv2.VideoWriter('output_hsv_vid.avi',fourcc,20.0,(640,480))
    
    
    # Webcam max resolution: 1920 x 1080 (1080p) #################################
    # Max fps: 60ps
    resolution_width = 1920
    resolution_height = 1080
    frameSize = (resolution_width, resolution_height)
    fps = 60
    
    video_cap.set(3, resolution_width)
    video_cap.set(4, resolution_height)
    
    vid_output = cv2.VideoWriter('pos_blocks_mm_t6.avi', fourcc, fps=fps, frameSize=frameSize)
    ###############################################################################

    
    xy_blocks = []   # 2D list of [[x,y], .... ]
    rotation_blocks = [] # 1D list of rotation values for each block [2, 34.2, ...]

    # Load calibration data
    with np.load('62cmfromconveyor_calibration_data.npz') as data: 
    #Best to worse:
    # multiple_img_calibration_data.npz --> test_calibration folder
    # cropped_chess_calibration_data.npz
    # semi_cropped_calibration_data.npz
    
        mtx = data['mtx']
        dist = data['dist']
        rvecs = data['rvecs']
        tvecs = data['tvecs']

    while True:
        ret, frame = video_cap.read()

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

        ''' OTHER COLORS -- CURRENTLY NOT IN USE
        # blue mask - og , s val = 80
        blue_lower = np.array([110, 80, 2], np.uint8)
        blue_upper = np.array([115, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

        # mask for olive green lego platform
        green_lower = np.array([40, 52, 72], np.uint8)
        green_upper = np.array([70, 200, 255], np.uint8)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)


        # for blue
        blue_mask = cv2.dilate(blue_mask, kernl)
        res_blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

        # for green
        green_mask = cv2.dilate(green_mask, kernl)
        res_green = cv2.bitwise_and(frame, frame, mask=green_mask)
        '''

        kernl = np.ones((7, 7), "uint8")

        # increases/smooths white in red mask
        # binarizing the image
        red_mask = cv2.dilate(red_mask, kernl)

        # contours to track red objects
        # detects contours in binary red_mask img
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        draw_dots(frame)

        rot_angle = 0

        # iterate through contours in each color mask
        red = "No red block detected"
        for i, contour in enumerate(contours):
            area_shape = cv2.contourArea(contour)
            # threshold of 400 to eliminate objects smaller than area of 400 pixels
            # static frame
            if (area_shape > 400):
                # pixels
                x, y, w, h = cv2.boundingRect(contour)

                
                # Extend lines of bounding box
                #cv2.line(frame, (0, y), (1920, y), (0, 0, 255), 2, )
                #cv2.line(frame, (0, y + h), (1920, y + h), (0, 0, 255), 2, )
                #cv2.line(frame, (x, 0), (x, 1080), (0, 0, 255), 2, )
                #cv2.line(frame, (x + w, 0), (x + w, 1080), (0, 0, 255), 2, )

                ''' 
                (x-coord for robots 2 and 3) = -1 * (y-coord for robot 1) + 84
                (y-coord for robots 2 and 3) = (x-coord for robot 1) - 270
                '''
                
                x_mm, y_mm = get_mm(x + int(w / 2), y + int(h / 2))
                
                # cv2.putText(frame, f' ({((x + w / 2)):.1f},{((y + h / 2)):.1f})',
                #             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                cv2.putText(frame, f'mm: ({np.round(x_mm, 2)}, {np.round(y_mm, 2)})', 
                            ((x + w), ((y + int(h / 2) + 15))),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                cv2.putText(frame, f'px: ({x + int(w / 2)}, {y + int(h / 2)})',
                            ((x + w), ((y + int(h / 2) - 15))),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

                # dot in center
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
                cv2.putText(frame, "Red block", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))

                # LABEL FOR DEGREES ROBOT NEED TO ROTATE HAND- THIS IS NOT AT WHAT DEGREE THE BLOCK IS AT
                cv2.putText(frame, f" Rotate: {(rot_angle):.0f} degrees  ", (x, (y + h) + 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 0))

                # visual add ons #

                # insert green line from corner of outside frame to center (x,y)
                cv2.line(frame, (x, y + h), (x + int(w / 2), y + int(h / 2)), (0, 255, 0), 1, )

        vid_output.write(frame)

        # show hsv vid
        # cv2.imshow('frame',hsv)
        cv2.imshow('multiple color detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_cap.release()
    vid_output.release()

    return xy_blocks, rotation_blocks

main()
print()