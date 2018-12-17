import cv2
import numpy as np
import math

# load the picture
cap0 = cv2.imread('pic.jpg')

# setup the width and lenght to reduce the caculation
width = 640
height = 480

cap = cv2.resize(cap0, (width, height), interpolation=cv2.INTER_CUBIC)

# setup the region you think the lane should be
region_of_interest_vertices = [(3, height), (3, height - 20),
                               (width / 2, height / 2 + 30),
                               (width, height - 20),
                               (width, height), ]


# change the pic from RGB to HSV and Gray
def transfer_image(image):
    if image is None:
        print('image is empty for function transfer_image')
        return

    # load picture as HSV and GRAY
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # gray for white color detect
    gray_white = cv2.inRange(gray_image, 100, 200)

    # gaussian blur to get smooth edge
    gauss_image = cv2.GaussianBlur(gray_white, (5, 5), 3)

    return gauss_image


# region of interest, just choose the region you think where the lane will be
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


# canny, edge points detection
def find_edg(img):
    if img is None:
        print('image is empty for function find_edg')
        return

    # set two threshold and detect edge points
    low_threshold = 100
    high_threshold = 200
    cannyed_edges = cv2.Canny(img, low_threshold, high_threshold)

    return cannyed_edges


# hough transform, find the stright lines
def find_lines(img):
    if img is None:
        print('image is empty for function find_lines')
        return

    # hough trasform to find stright line
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


# fine position for line
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

    all_lines = [fix_line, mid_line, edge_line]

    return all_lines


# draw lines on your raw picture
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
        i = i + 1

    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

    return img


while True:
    # HSV and GRAY
    image_color_gray = transfer_image(cap)
    # Get canny edge points
    img_canny = find_edg(image_color_gray)
    # change the region you want
    image_roied = region_of_interest(img_canny, np.array([region_of_interest_vertices], np.int32))
    # find lines
    lines = find_lines(image_roied)
    if lines is not None:
        # find fine lines
        lines_all = lines_fine(cap, lines)
        if lines_all is not None:
            final_image = draw_lines(cap, lines_all)
            # 显示图像
            if final_image is not None:
                cv2.imshow("capture", final_image)
            else:
                print('final_image is NONE')
                break
        else:
            print('lines_all is NONE')
            break
    else:
        print('lines is NONE')
        break

    if cv2.waitKey(1) & 0xFF == ord('`'):
        break

cap.release()
cv2.destroyAllWindows()

