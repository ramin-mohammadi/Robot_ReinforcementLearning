import cv2
import numpy as np
import pickle

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


def get_pos_angle(frame, color, mask, mm, distance_x, distance_y):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if color == "Green":
        bgr = (0, 255, 0)
    elif color == "Blue":
        bgr = (255, 0, 0)
    elif color == "Red":
        bgr = (0, 0, 255)
    for i, contour in enumerate(contours):
        is_detected = True
        area_shape = cv2.contourArea(contour)
        if (area_shape > 400):
            # pixels
            x, y, w, h = cv2.boundingRect(contour)

            # distance x, y is in mm, to compare distances, both x,y must be mm
            x_mm = np.round(x/mm, decimals=2)
            y_mm = np.round(y/mm, decimals=2)
            w_mm = w/mm
            h_mm = h/mm
            
            print(f"{color} pixel: ", x, y)
            print(f"{color} mm: ", x_mm, y_mm)

            rob_x = x_mm + distance_x
            rob_y = y_mm + distance_y
            
            print(f"{color} Robot mm: ", rob_x, rob_y)

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
            cv2.drawContours(frame, [box], 0, bgr, 2)

            angle = rect[2]

            # determine the angle the robot needs to rotate gripper
            if angle < 45:
                rot_angle = 0 - angle
            else:
                rot_angle = 90 - angle

            # LABEL OF FRAME FOR RED OBJECT
            cv2.putText(frame, f"{color} block", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, bgr)

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
            print(f'{color} block angle:{angle}.Robot must rotate {(rot_angle):.0f} degrees')
            red = f'{color} Block center coord:({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})'

# mask, name, color

def main():
    video_cap = cv2.VideoCapture(0) # was originally 1 but was not working
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    
    # x coordinate
    distance_x = -363.18 # want to add this much to camera's generated x to match robot's x , used to be 383.5
    # y coordinate
    distance_y = 198.19 # want to add this much to camera's generated y to match robot's y, used to be 117.3
    
    while True:
        # while ret = true,
        # reads frames from video_cap, frame = real time video
        ret, frame = video_cap.read()

        '''
        calib_result_pickle = pickle.load(open("camera_calib_pickle.p", "rb" ))
        mtx = calib_result_pickle["mtx"]
        #optimal_camera_matrix = calib_result_pickle["optimal_camera_matrix"]
        dist = calib_result_pickle["dist"]

        h, w = frame.shape[:2]

        optimal_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, 
                                                                   (w, h), 1, 
                                                                   (w, h))

        frame = cv2.undistort(frame, mtx, dist, None, 
                                         optimal_camera_matrix)
        
        x, y, w, h = roi
        frame = frame[y:y+h, x:x+w]
        '''

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
        
        # mask for olive green lego platform (HSV: 114, 52%, 37%) -> 114, 132, 94
        green_lower = np.array([40, 125, 90], np.uint8) # 40, 52, 72
        green_upper = np.array([120, 135, 100], np.uint8) # 100, 200,255
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # CONVERSION 1MM = .79 PIXELS AT A HEIGHT OF 550 MM
        mm = 0.561*2 # .865 

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

        get_pos_angle(frame, "Red", red_mask, mm, distance_x, distance_y)

        get_pos_angle(frame, "Blue", blue_mask, mm, distance_x, distance_y)

        cv2.imshow('multiple color detection', frame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

main()