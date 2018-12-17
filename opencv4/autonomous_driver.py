import cv2
import numpy as np
import math
from picamera import PiCamera
import RPi.GPIO as GPIO
import time


#**************************Motor Control definitions**************************************
GPIO.setmode(GPIO.BCM)

enA = 4
GPIO.setup(enA,GPIO.OUT)
enB = 18
GPIO.setup(enB,GPIO.OUT)
GPIO.setup(2,GPIO.OUT)
GPIO.setup(3,GPIO.OUT)
GPIO.setup(14,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)

motor1speed = GPIO.PWM(enA,50)
motor2speed = GPIO.PWM(enB,50)

def go():
    GPIO.output(2, True)
    GPIO.output(3, False)
    GPIO.output(14, True)
    GPIO.output(15, False)
def back():
    GPIO.output(2, False)
    GPIO.output(3, True)
    GPIO.output(14, False)
    GPIO.output(15, True)
    
def left():
    GPIO.output(2, True)
    GPIO.output(3, False)
    GPIO.output(14, False)
    GPIO.output(15, True)

def right():
    GPIO.output(2, False)
    GPIO.output(3, True)
    GPIO.output(14, True)
    GPIO.output(15, False)
    
def stop():
    GPIO.output(2, False)
    GPIO.output(3, False)
    GPIO.output(14, False)
    GPIO.output(15, False)
	
#*********************************************************************************



#**************************Camera Definitions*************************************
#load the picture
middle_line = []
fixed_line = []
#setup the width and lenght to reduce the caculation
width = 640
height = 480


#setup the region you think the lane should be
region_of_interest_vertices = [(3, height), (3, height - 200),
                               (width / 2, 0),
                               (width, height - 200),
                               (width, height), ]


#change the pic from RGB to HSV and Gray
def transfer_image(image):
    if image is None:
        print('image is empty for function transfer_image')
        return

    #load picture as HSV and GRAY
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # gray for white color detect
    gray_white = cv2.inRange(gray_image, 100, 200)
	
	# gaussian blur to get smooth edge
    gauss_image = cv2.GaussianBlur(gray_white, (5, 5), 3)
	
    return gauss_image	


#region of interest, just choose the region you think where the lane will be
def region_of_interest(img, shape):
    if img is None:
        print('image is empty for function region_of_interest')
        return

    roi_mask = np.zeros_like(img)

    channel_count = 2
	
    match_maks_color = (255,) * channel_count

    cv2.fillPoly(roi_mask, shape, match_maks_color)

    masked_image = cv2.bitwise_and(img, roi_mask)

    return masked_image
	
	
#canny, edge points detection
def find_edg(img):
    if img is None:
        print('image is empty for function find_edg')
        return

    #set two threshold and detect edge points
    low_threshold = 100
    high_threshold = 200
    cannyed_edges = cv2.Canny(img, low_threshold, high_threshold)
	
    return cannyed_edges
	
	
#hough transform, find the stright lines
def find_lines(img):
    if img is None:
        print('image is empty for function find_lines')
        return

    #hough trasform to find stright line
    lines = cv2.HoughLinesP(
        img,
        rho=6,
        theta=np.pi / 60,
        threshold=160,
        lines=np.array([]),
        minLineLength=40,
        maxLineGap=25
    )

    return lines


#fine position for line
def lines_fine(image, lines_):
    if lines_ is None:
        print('image is empty for lines_fine')
        return
    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []

    for line in lines_:
        for x1, y1, x2, y2 in line:
            if ((x2 - x1) != 0):
                slope = (y2 - y1) / (x2 - x1)
                if math.fabs(slope) < 0.5:
                    continue
                elif slope < 0:
                    left_line_x.extend([x1, x2])
                    left_line_y.extend([y1, y2])
                elif slope > 0:
                    right_line_x.extend([x1, x2])
                    right_line_y.extend([y1, y2])
            else:
                continue

    if (not left_line_y) or (not right_line_y):
        return

    # y position
    min_y = int(image.shape[0] * (65 / 100))
    max_y = int(image.shape[0] * (100 / 100))

    # x position
    poly_left = np.poly1d(np.polyfit(
        left_line_y,
        left_line_x,
        deg=1
    ))

    left_x_start = int(poly_left(max_y))
    left_x_end = int(poly_left(min_y))

    poly_right = np.poly1d(np.polyfit(
        right_line_y,
        right_line_x,
        deg=1
    ))

    right_x_start = int(poly_right(max_y))
    right_x_end = int(poly_right(min_y))

    # mid line
    mid_line_start_x = int((left_x_start + right_x_start) / 2)
    mid_line_start_y = max_y - 20
    mid_line_end_x = int((left_x_end + right_x_end) / 2)
    mid_line_end_y = min_y + 20
    
    middle_line.append(mid_line_start_x)
    middle_line.append(mid_line_start_y)
    middle_line.append(mid_line_end_x)
    middle_line.append(mid_line_end_y)
    fixed_line.append(int(image.shape[1] / 2))
    fixed_line.append(mid_line_end_y + 20)
    fixed_line.append(int(image.shape[1] / 2))
    fixed_line.append(mid_line_start_y - 20)
    
    mid_line = [[mid_line_start_x,
                 mid_line_start_y,
                 mid_line_end_x,
                 mid_line_end_y]]

    fix_line = [[int(image.shape[1] / 2),
                 mid_line_end_y + 20,
                 int(image.shape[1] / 2),
                 mid_line_start_y - 20]]

    edge_line = [
        [left_x_start, max_y, left_x_end, min_y],
        [right_x_start, max_y, right_x_end, min_y],
    ]
    #print("mid line: ", mid_line)
    #print("fix line: ", fix_line)
    #print("edge line: ", edge_line)
    all_lines = [fix_line, mid_line, edge_line]
    #print("all lines: ", all_lines)
    return all_lines


#draw lines on your raw picture
def draw_lines(images, lines_, thickness=3):
    if lines_ is None:
        print('lines is empty for function draw_lines')
        return

    i = 0
    colors = [[0, 0, 255], [0, 255, 0], [255, 0, 0]]

    img = np.copy(images)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8, )

    for line in lines_:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), colors[i], thickness)
        i = i+1

    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

    return img

def get_angle():
    num = (middle_line[0]-fixed_line[0])
    denom = (fixed_line[3]-middle_line[3])
    print(num, "/", denom)
    angle = math.tan(math.radians(num/denom))
    angle = math.fabs(angle) * 100
    print(middle_line)
    print(fixed_line)
    return angle
	
def drive(angle):
	max_speed = 30
	min_speed = 15
    instruction = ""
    if angle < 1:
        instruction = "go"
        motor1speed.start(float(max_speed))
        motor2speed.start(float(max_speed))
        go()	
    if angle >4 and middle_line[0] > 320:
        instruction = "rotleft"
        motor1speed.start(float(max_speed))
        motor2speed.start(float(max_speed))
        left()
    if angle >2 and middle_line[0] > 320:
        instruction = "turnleft"
        motor1speed.start(float(max_speed))
        motor2speed.start(float(min_speed))
        go()
    if angle >4 and middle_line[0] < 320:
        instruction = "rotright"
        motor1speed.start(float(max_speed))
        motor2speed.start(float(max_speed))
        right()	
    if angle > 2 and middle_line[0] < 320:
        instruction = "turnright"
        motor1speed.start(float(min_speed))
        motor2speed.start(float(max_speed))
        go()
    if angle > 5:
        instruction = "stop"
        motor1speed.start(float(max_speed))
        motor2speed.start(float(max_speed))
        stop()
    print(instruction)
	
camera = PiCamera()
#*********************************************************************************


while True:
    camera.capture('live.jpg')
    cap0 = cv2.imread('live.jpg')
    cap = cv2.resize(cap0, (width, height), interpolation=cv2.INTER_CUBIC)
    capflip = cv2.flip(cap, 0)
    middle_line = []
    fixed_line = []
    #HSV and GRAY
    image_color_gray = transfer_image(capflip)
    #Get canny edge points
    img_canny = find_edg(image_color_gray)
    #change the region you want
    image_roied = region_of_interest(img_canny, np.array([region_of_interest_vertices], np.int32))
    #find lines
    lines = find_lines(image_roied)
    if lines is not None:
        # find fine lines
        lines_all = lines_fine(capflip, lines)
        if lines_all is not None:
            angle = get_angle()
            print(angle)
            drive(angle)
            final_image = draw_lines(capflip, lines_all)
            if final_image is not None:
                cv2.imshow("capture", final_image)
            else:
                print('final_image is NONE')
                cv2.imshow("capture",capflip)
				stop()
        else:
            print('lines_all is NONE')
            cv2.imshow("capture",capflip)
    else:
        print('lines is NONE')
        cv2.imshow("capture",capflip)
		stop()

    if cv2.waitKey(1) & 0xFF == ord('`'):
		stop()
        break
	time.sleep(0.1)

cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()

