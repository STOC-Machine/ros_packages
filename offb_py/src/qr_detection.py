from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import random as rng
rng.seed(12345)

# Size of final image
HEIGHT = 1200
WIDTH = 1200

area_threshold = 3 # ignore contour with area smaller than that

quadrant_received = [0, 0, 0, 0]
# bottom right, bottom left, top right, top left
# 0: not received yet, 1 received


Position = {"bottom right": 0, "bottom left": 1, "top right": 2, "top left": 3}

def send_message(socket, qr_result):
    ## Connect to an IP with Port, could be a URL
    # sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # sock.connect(('192.168.0.165', 12459))
    ## Send some data, this method can be called multiple times
    socket.send(bytes(qr_result, 'utf-8'))
    ## Receive up to 4096 bytes from a peer
    socket.recv(4096)
    ## Close the socket connection, no more data transmission
    # sock.close()

def distance(point1, point2):
    return (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2

def decode_QR(image, socket):
    from pyzbar.pyzbar import decode
    from PIL import Image
    # print( decode(Image.open(image)))
    try:
        print( decode(Image.open(image))[0].data)
        send_message(socket, qr_result)
        # print( decode(Image.open("qr/Easy/result.jpg"))[0].data)
    except IndexError:
        print("Not enough data")

def extract_and_collect_qr(val, src, position, output, socket):
    # Convert image to gray and blur it
    src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    src_gray = cv.blur(src_gray, (3,3))

    threshold = val
    # Detect edges using Canny
    canny_output = cv.Canny(src_gray, threshold, threshold * 2)
    # Find contours
    contours, _ = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Find the convex hull object for all contours
    hull_list = []

    # Combine all contours into one
    drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
    drawing = src
    for i in range( len(contours)):
        if cv.contourArea(contours[i]) > area_threshold:
            contours[0] = contours[i]
            break
    for i in range(1,len(contours)):
        if cv.contourArea(contours[i]) > area_threshold:
            contours[0] = np.vstack((contours[0], contours[i]))
        
    hull = cv.convexHull(contours[0])
    hull_list.append(hull)
    cv.drawContours(drawing, hull_list, 0, (256,125,10))
    hull_list.append(hull)

    cv.imshow('Hull', drawing)

    width_src, height_src, channels_src = src.shape

    # Extract corner of the hull
    corner_top_left = 0
    for i in range(1, len(hull) ):
        if distance(hull[i][0], [0, 0]) < distance(hull[corner_top_left][0], [0, 0]):
            corner_top_left = i
    corner_top_right = 0
    for i in range(1, len(hull) ):
        if distance(hull[i][0], [width_src, 0]) < distance(hull[corner_top_right][0], [width_src, 0]):
            corner_top_right = i
    corner_bottom_left = 0
    for i in range(1, len(hull) ):
        if distance(hull[i][0], [0, height_src]) < distance(hull[corner_bottom_left][0], [0, height_src]):
            corner_bottom_left = i
    corner_bottom_right = 0
    for i in range(1, len(hull) ):
        if distance(hull[i][0], [width_src, height_src]) < distance(hull[corner_bottom_right][0], [width_src, height_src]):
            corner_bottom_right = i

    if position == 0:
        # Find the point that will map into the middle of result
        pts1 = np.float32([hull[corner_top_left][0], hull[corner_bottom_left][0], hull[corner_top_right][0], hull[corner_bottom_right][0]])
        pts2 = np.float32([[600,600], [600,1200], [1200,600], [1200,1200]])

        M = cv.getPerspectiveTransform(pts1,pts2)

        dst = cv.warpPerspective(src,M,(1200,1200))

        result = cv.imread(output)
        result[600:1200, 600:1200] = dst[600:1200, 600:1200]
        cv.imwrite(output, result)

        quadrant_received[0] = 1

    elif position == 1:
        # Find the point that will map into the middle of result
        pts1 = np.float32([hull[corner_top_right][0], hull[corner_bottom_right][0], hull[corner_top_left][0], hull[corner_bottom_left][0]])
        pts2 = np.float32([[600,600], [600,1200], [0,600], [0,1200]])

        M = cv.getPerspectiveTransform(pts1,pts2)

        dst = cv.warpPerspective(src,M,(1200,1200))

        result = cv.imread(output)
        result[600:1200, 0:600] = dst[600:1200, 0:600 ]
        cv.imwrite(output, result)

        quadrant_received[1] = 1

    elif position == 2:
        # Find the point that will map into the middle of result
        pts1 = np.float32([hull[corner_bottom_left][0], hull[corner_bottom_right][0], hull[corner_top_left][0], hull[corner_top_right][0]])
        pts2 = np.float32([[600,600], [1200,600], [600,0], [1200, 0]])

        M = cv.getPerspectiveTransform(pts1,pts2)

        dst = cv.warpPerspective(src,M,(1200,1200))

        result = cv.imread(output)
        result[0:600, 600:1200] = dst[0:600, 600:1200]
        cv.imwrite(output, result)

        quadrant_received[2] = 1

    else: # 3: top left
        pts1 = np.float32([hull[corner_bottom_right][0], hull[corner_top_right][0], hull[corner_bottom_left][0], hull[corner_top_left][0]])
        pts2 = np.float32([[600,600], [600,0], [0,600], [0,0]])

        M = cv.getPerspectiveTransform(pts1,pts2)

        dst = cv.warpPerspective(src,M,(1200,1200))

        result = cv.imread(output)
        # result = cv.imread("qr/Ken/result_tmp.jpg")
        result[0:600, 0:600] = dst[0:600, 0:600]
        cv.imwrite(output, result)

        quadrant_received[3] = 1

    # if 0 not in quadrant_received:
    decode_QR(output)

def process_image(image, position, output, socket):
    # Load source image
    src = cv.imread(cv.samples.findFile(image))
    src = cv.resize(src, (int(WIDTH),int(HEIGHT) ) )
    cv.imwrite("qr/resize.jpg", src)
    if src is None:
        print('Could not open or find the image:', image)
        exit(0)

    # decode_QR(image)
    max_thresh = 255
    # thresh = 100 # initial threshold
    thresh = 130 # initial threshold
    extract_and_collect_qr(thresh, src, position, socket)


# parser = argparse.ArgumentParser(description='Code for Convex Hull tutorial.')
# parser.add_argument('--input', help='Path to input image.', default='qr/qr1.png')
# parser.add_argument('--position', help='Position of input image. bottom right:0, bottom left: 1, top right: 2, top left: 3', default='0')
# parser.add_argument('--output', help='Output file', default='result.jpg')

# args = parser.parse_args()
# image = args.input
# position = args.position
# output = args.output
process_image(image, int(position), output, socket )


