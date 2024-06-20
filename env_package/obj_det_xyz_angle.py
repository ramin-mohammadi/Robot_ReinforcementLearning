import cv2
import numpy as np

# create instance to store red or blue object info 
class Block(object):
    def __init__(self, x, y, rotation, is_detected):
        self.x = x
        self.y = y
        self.rotation = rotation
        self.is_detected = is_detected
        
    def print(self):
        print(f"x: {(self.x):.1f} y: {(self.y):.1f} rotation: {(self.rotation):.0f} is_detected: {(self.is_detected)}")
        
    

def print_camera_info(red_block: Block, blue_block: Block):
    print("\nCamera Info:")
    print("Red Block")
    red_block.print()
    print("Blue Block")
    blue_block.print()

# Calculations are in pixels, output is in mm #


def get_pos_angle():
    video_cap = cv2.VideoCapture(0) # was originally 1 but was not working

    # SAVING VIDEO TO FILES:
    # codec for windows = DIVX
    # saves to video to files - overwrites each time
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # hsv_vid_output = cv2.VideoWriter('output_hsv_vid.avi',fourcc,20.0,(640,480))
    vid_output = cv2.VideoWriter('object_detection.avi', fourcc, 20.0, (640, 480))
    
    
    # we can measure this by placing red/blue object on top of Base A - camera would display x,y axis of its position
    # or by measuring distance with ruler, or both and compare results

    # hypothetical distance: distance = distance from cameras (0,0) to robots (0,0) in mm !!
    # NOTES: 
        # camera mm in first box: (366.08, 91.81)
        # robot mm in first box: (2.9, 290)
    # robot_x = camera_x + -363.18
    # robot_y = camera_y + 198.19
    
    # x coordinate
    distance_x = -363.18 # want to add this much to camera's generated x to match robot's x , used to be 383.5
    # y coordinate
    distance_y = 198.19 # want to add this much to camera's generated y to match robot's y, used to be 117.3
    
    # Notes (converting pixel to mmm): with red and blue block in drawn boxes on grid paper:
        # x and y's by pixel: (321, 141) (319, 80) 
        # the y axis in real world is 70 mm a part
        # by pixel is 61 mm a part
        # must scale distance up to be 70 mm so:
        # 70 / 61 = 1.14754098 -> multiply pixel values by this or divide by its reciprocal
        # reciprocal: 61 / 70 = .87142857
    

    # assuming every iteration of the loop is a frame
    for _ in range(1):
        # while ret = true,
        # reads frames from video_cap, frame = real time video
        ret, frame = video_cap.read()

        # convert to hsv (hue, saturation, value)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        
        # https://colorspire.com/rgb-color-wheel/

        # define color ranges for specific detection of that block
        # mask for red
        # higher end of spectrum - red range
        red_lower = np.array([0, 50, 50], np.uint8)
        red_upper = np.array([10, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsv, red_lower, red_upper)

        # blue mask - og , s val = 80
        blue_lower = np.array([110, 110, 15], np.uint8) #   (hue, saturation, value)    80 , 2
        blue_upper = np.array([115, 255, 255], np.uint8)  
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper) 

        # Get green color 
        green = np.uint8([[[51, 95, 46]]]) 
        
        # Convert Green color to Green HSV 
        # hsv_green = cv2.cvtColor(green, cv2.COLOR_BGR2HSV) # [63,132,95]
        # print(hsv_green)

        # mask for olive green lego platform (HSV: 114, 52%, 37%) -> 114, 132, 94
        green_lower = np.array([40, 125, 90], np.uint8) # 40, 52, 72
        green_upper = np.array([120, 135, 100], np.uint8) # 100, 200,255
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # CONVERSION 1MM = .79 PIXELS AT A HEIGHT OF 550 MM
        mm = .8714 # .865 

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
        
        
        red_is_detected = False
        red_x = -12.1
        red_y = 181.6
        red_rotation = -91.5 # yaw
        
        blue_is_detected = False
        blue_x = -12.1
        blue_y = 181.6
        blue_rotation = -91.5
        
        green_is_detected = False
        green_x = -12.1
        green_y = 181.6
        green_rotation = -91.5

        # iterate through contours in each color mask
        red = "No red block detected"
        for i, contour in enumerate(contours):
            red_is_detected = True
            area_shape = cv2.contourArea(contour)
            # threshold of 400 to eliminate objects smaller than area of 400 pixels
            # static frame
            if (area_shape > 400):
                # pixels
                x, y, w, h = cv2.boundingRect(contour)

                # distance x, y is in mm, to compare distances, both x,y must be mm
                x_mm = np.round(x/mm, decimals=2)
                y_mm = np.round(y/mm, decimals=2)
                w_mm = w/mm
                h_mm = h/mm
                
                print("Red pixel: ", x, y)
                print("Red mm: ", x_mm, y_mm)

            # distance in x-axis from camera to robot
                # if x_mm > distance_x:
                #     # new_ex is only for display of numbers and for robot
                #     # x is for what will be seen through the camera ( frame,circle,lines)
                #     rob_x = distance_x - x_mm
                # else:
                #     rob_x = x_mm - distance_x
                rob_x = x_mm + distance_x
                # rob_x = distance_x - x_mm
                

            # distance in y-axis from camera(0,0) to robot(0,0)
                #rob_y = y_mm - distance_y'
                rob_y = y_mm + distance_y
                
                print("Red Robot mm: ", rob_x, rob_y)


                # dot above robots (x,y) to check if accurately positioned
    #            cv2.circle(frame, (distance_x, distance_y), 4, (0, 0, 0), -1)

                # CENTER COORDINATES
                cv2.putText(frame, f' ({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
                            ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
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
                
                red_x = np.round(rob_x, decimals=1)
                red_y = np.round(rob_y, decimals=1)
                red_rotation = rot_angle

                # visual add ons #

                # insert green line from corner of outside frame to center (x,y)
                cv2.line(frame, (x, y + h), (x + int(w / 2), y + int(h / 2)), (0, 255, 0), 1, )

                # INFO DISPLAYED IN CONSOLE
                print(f'Red block angle:{angle}.Robot must rotate {(rot_angle):.0f} degrees')
                red = f'Red Block center coord:({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})'

        print(red)
        
        red_block = Block(red_x, red_y, red_rotation, red_is_detected)

        
        
        
        
        
        
        
    

        # contours to track blue
        contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        blue = "No blue block found"
        for i, contour in enumerate(contours):
            blue_is_detected = True
            area_shape = cv2.contourArea(contour)
            if (area_shape > 400):
                x, y, w, h = cv2.boundingRect(contour)

                # distance x, y is in mm, to compare distances, both x,y must be mm
                x_mm = np.round(x / mm, decimals=2)
                y_mm = np.round(y / mm, decimals=2)
                w_mm = w / mm
                h_mm = h / mm
                
                print("Blue pixels: ", x, y)
                print("Blue mm: ", x_mm, y_mm)

                # distance in x-axis from camera to robot
                # if x_mm > distance_x:
                #     # new_ex is only for display of numbers and for robot
                #     # x is for what will be seen through the camera ( frame,circle,lines)
                #     rob_x = distance_x - x_mm
                # else:
                #     rob_x = x_mm - distance_x
                rob_x = x_mm + distance_x
                # rob_x = distance_x - x_mm


                # distance in y-axis from camera(0,0) to robot(0,0)
                # rob_y = y_mm - distance_y
                rob_y = y_mm + distance_y
                
                print("Blue Robot mm: ", rob_x, rob_y)
                
                # DISPLAY CENTER COORDINATES
                cv2.putText(frame, f' ({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
                            (x + int(w / 2), y + int(h / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                # dot in center
                cv2.circle(frame, (x + int(w / 2), y + int(h / 2)), 4, (0, 0, 0), -1)
                # k.l.s

                frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 1)

                # draw box to follow with angle
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)

                angle = rect[2]
                if angle < 45:
                    rot_angle = 0 - angle
                else:
                    rot_angle = 90 - angle

                # Label on Blue block static frame
                cv2.putText(frame, "Blue Block", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
                # label of angle under static frame
                cv2.putText(frame, f"Rotate: {(rot_angle):.0f} degrees", (x, (y + h) + 20), cv2.FONT_HERSHEY_SIMPLEX, .5,
                            (0, 0, 0))
                
                blue_x = np.round(rob_x, decimals=1)
                blue_y = np.round(rob_y, decimals=1)
                blue_rotation = rot_angle
                

                # visual add ons #

                # insert line from corner of outside frame to center (x,y) (AET purposes)
                cv2.line(frame, (x, y + h), (x + int(w / 2), y + int(h / 2)), (0, 255, 0), 1, )

                # INFO DISPLAYED IN CONSOLE
                print(f'Blue block angle:{angle}.Robot arm must rotate {(rot_angle):.0f} degrees')
                blue = f'Blue Block center coord:({((rob_x + w_mm / 2) ):.1f},{((rob_y + h_mm / 2)):.1f})'

        print(blue)
        
        blue_block = Block(blue_x, blue_y, blue_rotation, blue_is_detected)
        
        
        
        
        
        


        # contours to track green
        contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        green = "No green platform found"
        for i, contour in enumerate(contours):
            green_is_detected = True
            area_shape = cv2.contourArea(contour)
            if (area_shape > 2500):
                x, y, w, h = cv2.boundingRect(contour)

                # distance x, y is in mm, to compare distances, both x,y must be mm
                x_mm = np.round(x / mm, decimals=2)
                y_mm = np.round(y / mm, decimals=2)
                w_mm = w / mm
                h_mm = h / mm

                
                rob_x = x_mm + distance_x   

                # distance in y-axis from camera(0,0) to robot(0,0)
                rob_y = y_mm + distance_y

                frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, "Green Platform", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (50, 255, 0))

                # display center coord
                cv2.putText(frame, f'({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})', (x + int(w / 2), y + int(h / 2)),
                            cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0, 0, 0))
                # dot in center
                cv2.circle(frame, (x + int(w / 2), y + int(h / 2)), 4, (0, 0, 0), -1)
                
                green_x = np.round(rob_x, decimals=1)
                green_y = np.round(rob_y, decimals=1)
                green_rotation = rot_angle

            # INFO DISPLAYED IN CONSOLE
            green = f'Green platform center coord:({((rob_x + w_mm / 2)):.1f},{((y + h_mm / 2)):.1f})'
            print(green)
            
            green_block = Block(green_x, green_y, green_rotation, green_is_detected)
        
        

        # output that frame
        # hsv_vid_output.write(hsv)
        vid_output.write(frame)

        # show hsv vid
        #    cv2.imshow('frame',hsv)
        cv2.imshow('multiple color detection', frame)

        #     :)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_cap.release()
    vid_output.release()
    
    return red_block, blue_block





# test camera:
red_block, blue_block = get_pos_angle()
print_camera_info(red_block, blue_block)