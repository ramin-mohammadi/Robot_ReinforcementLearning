import cv2
import numpy as np
import glob

# Create instance to store red or blue object info 
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

# Load calibration data
with np.load('multiple_img_calibration_data.npz') as data: # change to "multiple_img_calibration_data.npz" before running or if using different data imgs
    mtx = data['mtx']
    dist = data['dist']
    rvecs = data['rvecs']
    tvecs = data['tvecs']

def get_pos_angle():
    video_cap = cv2.VideoCapture(0) # was originally 1 but was not working

    # SAVING VIDEO TO FILES:
    # codec for windows = DIVX
    # saves to video to files - overwrites each time
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    vid_output = cv2.VideoWriter('object_detection.avi', fourcc, 20.0, (640, 480))

    # hypothetical distance: distance from camera's (0,0) to robot's (0,0) in mm
    distance_x = -363.18 # offset in mm for x-coordinate
    distance_y = 198.19 # offset in mm for y-coordinate
    mm = .8714 # conversion factor from pixels to mm

    # for _ in range(1):
    while True:
        ret, frame = video_cap.read()

        # Undistort the frame
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges
        # red_lower = np.array([0, 120, 70],  np.uint8)
        # red_upper = np.array([10, 255, 255],  np.uint8)
        # blue_lower = np.array([94, 80, 2], np.uint8)
        # blue_upper = np.array([126, 255, 255], np.uint8)
        # green_lower = np.array([25, 52, 72], np.uint8)
        # green_upper = np.array([102, 255, 255], np.uint8)

        # red_lower = np.array([0, 70, 70],  np.uint8)
        # red_upper = np.array([10, 255, 255],  np.uint8)
        # blue_lower = np.array([112, 112, 15], np.uint8)
        # blue_upper = np.array([115, 255, 255], np.uint8)
        # green_lower = np.array([40, 125, 90], np.uint8)
        # green_upper = np.array([120, 135, 100], np.uint8)

        red_lower = np.array([0, 70, 70], np.uint8)
        red_upper = np.array([10, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsv, red_lower, red_upper)

        blue_lower = np.array([110, 110, 15], np.uint8) #might want to change the values here slightly to reduce noise
        blue_upper = np.array([115, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

        green_lower = np.array([40, 125, 90], np.uint8)
        green_upper = np.array([120, 135, 100], np.uint8)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        kernl = np.ones((7, 7), "uint8")
        red_mask = cv2.dilate(red_mask, kernl)
        res_red = cv2.bitwise_and(frame, frame, mask=red_mask)

        blue_mask = cv2.dilate(blue_mask, kernl)
        res_blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

        green_mask = cv2.dilate(green_mask, kernl)
        res_green = cv2.bitwise_and(frame, frame, mask=green_mask)

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
            if (area_shape > 400):
            # threshold of 400 to eliminate objects smaller than area of 400 pixels
            # static frame
                x, y, w, h = cv2.boundingRect(contour)

                x_mm = np.round(x/mm, decimals=2)
                y_mm = np.round(y/mm, decimals=2)
                w_mm = w/mm
                h_mm = h/mm
                
                print("Red pixel: ", x, y)
                print("Red mm: ", x_mm, y_mm)

                rob_x = x_mm + distance_x
                rob_y = y_mm + distance_y
                
                print("Red Robot mm: ", rob_x, rob_y)

                cv2.putText(frame, f' ({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
                            ((x + int(w / 2)), ((y + (int(h / 2)) + 20))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                cv2.circle(frame, (x + int(w / 2), y + int(h / 2)), 4, (0, 0, 0), -1)
                frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 1)

                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

                angle = rect[2]

                if angle < 45:
                    rot_angle = 0 - angle
                else:
                    rot_angle = 90 - angle

                cv2.putText(frame, "Red block", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
                cv2.putText(frame, f" Rotate: {(rot_angle):.0f} degrees  ", (x, (y + h) + 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 0))
                
                red_x = np.round(rob_x, decimals=1)
                red_y = np.round(rob_y, decimals=1)
                red_rotation = rot_angle

                cv2.line(frame, (x, y + h), (x + int(w / 2), y + int(h / 2)), (0, 255, 0), 1, )

                print(f'Red block angle:{angle}.Robot must rotate {(rot_angle):.0f} degrees')
                red = f'Red Block center coord:({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})'

        print(red)
        
        red_block = Block(red_x, red_y, red_rotation, red_is_detected)

        contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        blue = "No blue block found"
        for i, contour in enumerate(contours):
            blue_is_detected = True
            area_shape = cv2.contourArea(contour)
            if (area_shape > 400):
                x, y, w, h = cv2.boundingRect(contour)
                x_mm = np.round(x / mm, decimals=2)
                y_mm = np.round(y / mm, decimals=2)
                w_mm = w / mm
                h_mm = h / mm
                
                print("Blue pixels: ", x, y)
                print("Blue mm: ", x_mm, y_mm)

                rob_x = x_mm + distance_x
                rob_y = y_mm + distance_y
                
                print("Blue Robot mm: ", rob_x, rob_y)
                
                cv2.putText(frame, f' ({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
                            (x + int(w / 2), y + int(h / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                cv2.circle(frame, (x + int(w / 2), y + int(h / 2)), 4, (0, 0, 0), -1)

                frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 1)

                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)

                angle = rect[2]

                if angle < 45:
                    rot_angle = 0 - angle
                else:
                    rot_angle = 90 - angle

                cv2.putText(frame, "Blue block", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
                cv2.putText(frame, f" Rotate: {(rot_angle):.0f} degrees", (x, (y + h) + 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 0))
                
                blue_x = np.round(rob_x, decimals=1)
                blue_y = np.round(rob_y, decimals=1)
                blue_rotation = rot_angle

                cv2.line(frame, (x, y + h), (x + int(w / 2), y + int(h / 2)), (0, 255, 0), 1)

                print(f'Blue block angle:{angle}.Robot must rotate {(rot_angle):.0f} degrees')
                blue = f'Blue Block center coord:({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})'
        print(blue)
        
        blue_block = Block(blue_x, blue_y, blue_rotation, blue_is_detected)

        contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        green = "No green block found"
        for i, contour in enumerate(contours):
            green_is_detected = True
            area_shape = cv2.contourArea(contour)
            if (area_shape > 400):
                x, y, w, h = cv2.boundingRect(contour)
                x_mm = np.round(x / mm, decimals=2)
                y_mm = np.round(y / mm, decimals=2)
                w_mm = w / mm
                h_mm = h / mm
                
                print("Green pixels: ", x, y)
                print("Green mm: ", x_mm, y_mm)

                rob_x = x_mm + distance_x
                rob_y = y_mm + distance_y
                
                print("Green Robot mm: ", rob_x, rob_y)
                
                cv2.putText(frame, f' ({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})',
                            (x + int(w / 2), y + int(h / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
                cv2.circle(frame, (x + int(w / 2), y + int(h / 2)), 4, (0, 0, 0), -1)

                frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 1)

                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

                angle = rect[2]

                if angle < 45:
                    rot_angle = 0 - angle
                else:
                    rot_angle = 90 - angle

                cv2.putText(frame, "Green block", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
                cv2.putText(frame, f" Rotate: {(rot_angle):.0f} degrees", (x, (y + h) + 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 0))
                
                green_x = np.round(rob_x, decimals=1)
                green_y = np.round(rob_y, decimals=1)
                green_rotation = rot_angle

                cv2.line(frame, (x, y + h), (x + int(w / 2), y + int(h / 2)), (0, 255, 0), 1)

                print(f'Green block angle:{angle}.Robot must rotate {(rot_angle):.0f} degrees')
                green = f'Green Block center coord:({((rob_x + w_mm / 2)):.1f},{((rob_y + h_mm / 2)):.1f})'
        print(green)
        
        green_block = Block(green_x, green_y, green_rotation, green_is_detected)

        print_camera_info(red_block, blue_block)

        vid_output.write(frame)

        cv2.imshow("Red Blocks in Frame", res_red)
        cv2.imshow("Blue Blocks in Frame", res_blue)
        cv2.imshow("Green Blocks in Frame", res_green)
        cv2.imshow("Multiple Color Detection in Real-Time", frame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    video_cap.release()
    vid_output.release()

    cv2.destroyAllWindows()

get_pos_angle()
