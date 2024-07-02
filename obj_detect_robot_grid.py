import cv2
import numpy as np

# MOST RECENT CODE:

# Calculations are in pixels, output is in mm #
# NO BLUE DETECTION


# Notes
"""
NOTE: for all of these, the y is good, the block is grabbed, but the block doesnt line up with the gripper (at the least, half of the block is grabbed)

zone 1 good but a little too much x+
zone 2 too much x+
zone 3 on paper counted as 4, too much x+
sone 4 good but slightly x+ overshot
zone 5 mistaken for zone 6, slightly too much x+
zone 6 was a little too much in x+
zone 7 mistaken for 8, slightly too much x+
zone 8 slightly too much x+
zone 9 on paper counted as 10, too much x+
sone 10 x+ overshot
zone 11 is being mistaken for zone 12, but x slightly too much x+
zone 12 was good but x was slightly too much in x+
zone 13 mistaken for 14, slightly too much x+
zone 14 overshot x+ 
zone 15 on paper is counted as 16 in code but is PERFECT
zone 16 counted as 16 and was basically perfect 
"""


def get_blocks():
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
    frameSize=(resolution_width, resolution_height)
    fps = 60
    
    video_cap.set(3, resolution_width)
    video_cap.set(4, resolution_height)
    
    vid_output = cv2.VideoWriter('pos_blocks_mm_t6.avi', fourcc, fps=fps, frameSize=frameSize)
    ###############################################################################

    
    xy_blocks = []   # 2D list of [[x,y], .... ]
    rotation_blocks = [] # 1D list of rotation values for each block [2, 34.2, ...]


    while True:
    # for i in range(1):
        # while ret = true,
        # reads frames from video_cap, frame = real time video
        ret, frame = video_cap.read()



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



        # CONVERSION 1MM =  ___ PIXELS AT A HEIGHT OF ___ MM
        # .564
        #.545
        xmm = .545

        ymm = .538



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
    #    cv2.line(frame,(0,240),(640,240),(255,255,255),1)
    #    cv2.line(frame,(320,0),(320,480),(255,255,255),1)
        cv2.circle(frame,(320,240),4,(0,0,255),-1)
        cv2.circle(frame, (210, 120), 4, (0, 0, 255), -1)
        cv2.circle(frame, (430, 360), 4, (0, 0, 255), -1)
        cv2.circle(frame, (210, 360), 4, (0, 0, 255), -1)
        cv2.circle(frame, (430, 120), 4, (0, 0, 255), -1)


        
        rob_x = 0
        rob_y = 0
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


 
                # distance x, y is in mm, to compare distances, both x,y must be mm
                x_mm = x/xmm
                y_mm = y/ymm
                w_mm = w/xmm
                h_mm = h/ymm
                
                cv2.putText(frame, f' ({((x + w / 2)):.1f},{((y + h / 2)):.1f})',
                            ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                # print("BLOCKKK: ", (x + w / 2), ((y + h / 2)))
                
                
    #             # Zone 1:
    #             if x_mm <= 310 and y_mm <= 270:
    #                 distance_x = -354.5 + 35 # + 35
    #                 distance_y = 152.5

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (1:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 1")

    #             # Zone 2:
    #             if x_mm >= 311 and y_mm <= 270:
    #                 distance_x = -349.7 + 35
    #                 distance_y = 152.4

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (2:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 2")
                    
    #             # Zone 3:
    #             if (x_mm <= 310 and (y_mm <= 297 and y_mm > 270)):
    #                 distance_x = -354.92 + 29
    #                 distance_y = 100.6

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (3:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 3")

    #             # Zone 4:
    #             if (x_mm >= 311 and (y_mm <= 297 and y_mm > 270)):
    #                 distance_x = -348.6 + 29
    #                 distance_y = 97.6

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (4:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 4")

    #             # Zone 5:
    #             if (x_mm <= 310 and (y_mm <= 325 and y_mm > 297)):
    #                 distance_x = -354.35 + 27
    #                 distance_y = 44.9

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (5:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 5")

    #             # Zone 6:
    #             if (x_mm >= 311 and (y_mm <= 325 and y_mm > 297)):
    #                 distance_x = -349 + 27
    #                 distance_y = 44.9

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (6:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 6")

    #             # Zone 7:
    #             if (x_mm <= 310 and (y_mm <= 352.5 and y_mm >325)):
    #                 distance_x = -352.77 + 26
    #                 distance_y = -13.4

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (7: {((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 7")

    #             # Zone 8:
    #             if (x_mm >= 311 and (y_mm <= 352.5 and y_mm > 325)):
    #                 distance_x = -348.4 + 26
    #                 distance_y = -13.4

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (8:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 8")

    #             # Zone 9:
    #             if (x_mm <= 310 and (y_mm <= 380 and y_mm > 352.5)):
    #                 distance_x = -353.19 + 25
    #                 distance_y = -84.2

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (9:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 9")

    #             # Zone 10:
    #             if (x_mm >= 311 and (y_mm <= 380 and y_mm > 352.5)):
    #                 distance_x = -346.8 + 25
    #                 distance_y = -84.2

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (10:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 10")

    #             # Zone 11:
    #             if (x_mm <= 310 and (y_mm <= 407 and y_mm > 380)):
    #                 distance_x = -353.62 + 22
    #                 distance_y = -127.2 - 13

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (11:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 11")

    #             # Zone 12:
    #             if (x_mm >= 311 and (y_mm <= 407 and y_mm > 380)):
    #                 distance_x = -347.2 + 22
    #                 distance_y = -127.2 - 13

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (12:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 12")

    #             # Zone 13:
    #             if (x_mm <= 310 and (y_mm <= 435 and y_mm > 407)):
    #                 distance_x = -362.04 + 30
    #                 distance_y = -183.8

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (13:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 13")

    #             # Zone 14:
    #             if (x_mm >= 311 and (y_mm <= 435 and y_mm > 407)):
    #                 distance_x = -346.6 + 30
    #                 distance_y = -184.5

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (14:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 14")

    #             # Zone 15:
    #             if (x_mm <= 310 and (y_mm <= 462 and y_mm > 435)):
    #                 distance_x = -350.46 + 15
    #                 distance_y = -242

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (15: {((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 15")

    #             # Zone 16:
    #             if (x_mm >= 311 and (y_mm <= 462 and y_mm > 435)):
    #                 distance_x = -345 + 15
    #                 distance_y = -242

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (16: {((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 16")

    #             # Zone 17:
    # #            if (x_mm <= 350 and (y_mm <= 600 and y > 462)):
    #             if (x_mm <= 350 and (y_mm <= 700 and y > 450)):
    #                 distance_x = -357.58 + 12
    #                 distance_y = -307.6

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (17:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))
    #                 print("Zone 17")

    #             # Zone 18:
    #             if (x_mm >= 350 and (y_mm <= 490 and y > 462)):
    #                 distance_x = -344.4 + 12
    #                 distance_y = -304.6

    #                 rob_x = x_mm + distance_x
    #                 rob_y = y_mm + distance_y
    #                 # CENTER COORDINATES
    #                 cv2.putText(frame, f' (18:{((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
    #                             ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_COMPLEX, 0.25, (0, 0, 0))

    #                 print("Zone 18")




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
                cv2.putText(frame, "Red block", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))

                # LABEL FOR DEGREES ROBOT NEED TO ROTATE HAND- THIS IS NOT AT WHAT DEGREE THE BLOCK IS AT
                cv2.putText(frame, f" Rotate: {(rot_angle):.0f} degrees  ", (x, (y + h) + 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 0))

                # visual add ons #

                # insert green line from corner of outside frame to center (x,y)
                cv2.line(frame, (x, y + h), (x + int(w / 2), y + int(h / 2)), (0, 255, 0), 1, )

                # INFO DISPLAYED IN CONSOLE
                print(f'Red block angle:{angle}.Robot must rotate {(rot_angle):.0f} degrees')
    #            red = f'Red Block center coord:({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})'
                
                # xy_blocks.append([rob_x, rob_y])
                # rotation_blocks.append(rot_angle)

        # INFO DISPLAYED IN CONSOLE
    #    print(f'Red block angle:{angle}.Robot must rotate {(rot_angle):.0f} degrees')
    #    red = f'Red Block center coord:({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})'

        
        
        print(red)
        
        xy_blocks = np.round(xy_blocks, decimals=4)
        print("X,Y: ", xy_blocks)
        print("Rotation: ", rotation_blocks)
        


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

    return xy_blocks, rotation_blocks



get_blocks()