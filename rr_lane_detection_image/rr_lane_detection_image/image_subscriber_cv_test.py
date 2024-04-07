#!/user/bin/env python3

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy import optimize 
from matplotlib import pyplot as plt, cm, colors

#using standard rodas and pixel width: adjust on the track

ym_per_pix = 30 /720
xm_per_pix = 3.7 / 720

subscriberNodeName='image_raw_lane_detector_subscriber'

topicName='camera/color/image_raw' 

topLeft = [0, 0]
topRight = [0, 0]
bottomLeft = [0, 0]
bottomRight = [0, 0]
left = False
right = False

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("topLeft: ", x, y)
        topLeft = [x,y]
    if event == cv2.EVENT_RBUTTONDOWN:
        print("topRight: ", x, y)
        topRight = [x,y]
    if event == cv2.EVENT_LBUTTONUP:
        print("bottomLeft: ", x, y)
        bottomLeft = [x,y]
        left = True
    if event == cv2.EVENT_RBUTTONUP:
        print("bottomRight: ", x, y)
        bottomRight = [x,y]
        right = True



#region_of_interest_verticies = [ (0, height), (width / 2, height / 2), (width, height)]

def imageToGrayscale(inputImage):
    #convert image grayscale, aplly threshold, blur & extract edges
    gray = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh, (3,3), 0)
    canny = cv2.Cany(blur, 40, 60)
    return gray, thresh, blur, canny

def processsImage(inputImage, show = False):
    #converts image to hls color range
    hls = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HLS)
    #ajust lower white range based on light
    lower_white = np.array([0, 160, 10])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(inputImage, lower_white, upper_white)
    hls_result = cv2.bitwise_and(inputImage, inputImage, mask = mask)


    #convert image grayscale, aplly threshold, blur & extract edges
    gray = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh, (3,3), 0)
    canny = cv2.Canny(blur, 40, 60)
    if (show):
        cv2.imshow("Image", inputImage)
        cv2.imshow("hls filter", hls_result)
        cv2.imshow("grayscale", gray)
        cv2.imshow("threshholed image", thresh)
        cv2.imshow("blured image", blur)
        cv2.imshow("canny edge", canny)
    return inputImage, hls_result, gray, thresh, blur, canny


def prespectiveWarp(inputImage, show = False):
    imgSize = (inputImage.shape[1], inputImage.shape[0])
    width = imgSize[0]
    height = imgSize[1]

    #define point to be warped
    src = np.float32([[280,260],[420, 260], [10, 470], [630, 480]])

    #define window to be shown
    # height should be height of window
    dst = np.float32([[0,0],[640, 0], [0, 480], [640, 480]])

    matrix = cv2.getPerspectiveTransform(src, dst)
    mat_inv = cv2.getPerspectiveTransform(dst, src)
    
    birdseveyView = cv2.warpPerspective(inputImage, matrix, imgSize)
    birdseveyViewLeft = birdseveyView[0 : height, 0 : width // 2]
    birdseveyViewRight = birdseveyView[0 : height, width // 2 : width]
    if (show):
        cv2.imshow('Birdseye', birdseveyView)
        #cv2.imshow('BirdseyeLeft', birdseveyViewLeft)
        #cv2.imshow('BirdseyeRight', birdseveyViewRight)
    return birdseveyView, birdseveyViewLeft, birdseveyViewRight, mat_inv



def plotHistorgram(inputImage):
    #sum the columns (first axis) form the bottom half of the image
    histogram = np.sum(inputImage[inputImage.shape[0] // 2 :, :], axis = 0)


def slide_window_search(inputImage):
    height_y = inputImage.shape[0]
    histogram = np.sum(inputImage[height_y // 2 :, :], axis = 0)

    out_img = np.dstack((inputImage, inputImage,inputImage)) * 255
    midpoint = histogram.shap[0] // 2
    left_x_base = np.argmax(histogram[:midpoint])
    right_x_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 10
    window_height = height_y // nwindows
    nonzero = inputImage.nonzero()
    nonzero_y = np.array(nonzero[0])
    nonzero_x = np.array(nonzero[1])
    left_x_curr = left_x_base
    right_x_curr = right_x_base
    margin = 100
    minmun_pixel = 50
    left_lane_indx = []
    right_lane_indx = []


    for window in range(nwindows):
        window_y_low = height_y - (window + 1) * window_height
        window_y_high = height_y - window * window_height # window_y_low = window_height

        window_xleft_low = left_x_curr - margin
        window_xleft_high = left_x_curr + margin

        window_xright_low = right_x_curr - margin
        window_xright_high = right_x_curr + margin


        #NEED TO UNDERSTAND WHAT THIS DOES
        good_left_indx = ((nonzero_y >= window_y_low) & (nonzero_y < window_y_high) &
        (nonzero_x >= window_xleft_low) &  (nonzero_x < window_xleft_high)).nonzero()[0]
        
        good_right_indx = ((nonzero_y >= window_y_low) & (nonzero_y < window_y_high) &
        (nonzero_x >= window_xright_low) &  (nonzero_x < window_xright_high)).nonzero()[0]


        left_lane_indx.append(good_left_indx)
        right_lane_indx.append(good_right_indx)

        if len(good_left_indx) > minmun_pixel:
            left_x_curr = int(np.mean(nonzero[good_left_indx]))
        if len(good_right_indx) > minmun_pixel:
            right_x_curr = int(np.mean(nonzero[good_right_indx]))

    left_lane_indx = np.concatenate(left_lane_indx)
    right_lane_indx = np.concatenate(right_lane_indx)

    leftx = nonzero_x[left_lane_indx]
    lefty = nonzero_y[left_lane_indx]

    rightx = nonzero_x[right_lane_indx]
    righty = nonzero_y[right_lane_indx]

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    print(left_fit)
    print(right_fit)

    ploty = np.linspace(0, inputImage.shape[0]-1, inputImage.shape[0])
    left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

    ltx = np.trunc(left_fitx)
    rtx = np.trunc(right_fitx)
    plt.plot(right_fitx)
    # plt.show()

    out_img[nonzero_y[left_lane_indx], nonzero_x[left_lane_indx]] = [255, 0, 0]
    out_img[nonzero_y[right_lane_indx], nonzero_x[right_lane_indx]] = [0, 0, 255]

    # plt.imshow(out_img)
    plt.plot(left_fitx,  ploty, color = 'yellow')
    plt.plot(right_fitx, ploty, color = 'yellow')
    plt.xlim(0, 1280)
    plt.ylim(720, 0)

    return ploty, left_fit, right_fit, ltx, rtx




def general_search(inputImage, left_fit, right_fit):
    nonzero = inputImage.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin)))
    
    right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))


    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    ploty = np.linspace(0, inputImage.shape[0] - 1, inputImage.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    ret = {}
    ret['leftx'] = leftx
    ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty
    return ret

def measure_lane_curvature(ploty, leftx, rightx):
    leftx = leftx[::-1]
    rightx = rightx[::-1]
    y_eval = np.max(ploty)
    
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)


    left_curve = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curve = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    mid_curve = (left_curve + right_curve) / 2.0

    if leftx[0] - leftx[-1] > 60:
        curveDir = -1 #left
    elif leftx[0] - leftx[-1] < -60:
        curveDir = 1 #right
    else:
        curveDir = 0
        
    return mid_curve, curveDir

def calcOffCenter(meanPts, inpFrame):
    mpts = meanPts[-1][-1][-2].astype(int)
    pixelDeviation = inpFrame.shape[1]/2 - abs(mpts)
    deviation = pixelDeviation * xm_per_pix
    dir = -1 if deviation < 0 else 0
    return deviation, dir

def addText(img, radius, direction, deviation, devDirection):
    font = cv2.FONT_HERSHEY_COMPLEX
    
    text = 'Radius Of curve: ' + '{:04.0f}'.format(radius)
    text1 = 'Curve Dir ' + (direction)

    cv2.putText(img, text , (50,100), font, 0.8, (0,100, 200), 2, cv2.LINE_AA)
    cv2.putText(img, text1, (50,150), font, 0.8, (0,100, 200), 2, cv2.LINE_AA)
    deviation_text = 'off center: ' + str(round(abs(deviation), 3)) + ' dir: ' + devDirection
    cv2.putText(img, deviation_text, (50, 200), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0,100, 200), 2, cv2.LINE_AA)
    return img

def draw_lane_lines(orginal_image, warped_image, Minv, draw_info):
    leftx = draw_info['leftx']
    rightx = draw_info['rightx']
    left_fitx = draw_info['left_fitx']
    right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']

    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.htsack((pts_left, pts_right))

    meanx = np.mean((left_fitx, right_fitx), axis = 0)
    pts_mean = np.array([np.flipud(np.transpose(np.vstack([meanx, ploty])))])
    
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    cv2.fillPoly(color_warp, np.int_([pts_mean]), (0, 255, 255))

    newwarp = cv2.warpPerspective(color_warp, Minv, (orginal_image[1], orginal_image[0]))
    result = cv2.addWeighted(orginal_image, 1, newwarp, 0.3, 0)
    return pts_mean, result

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        global frameCount
        frameCount = 0
        self.subscription = self.create_subscription(Image, topicName, self.listner_callback, 10)
        self.subscription


    def listner_callback(self, message):
        bridgeObject = CvBridge()
        #self.get_logger().info("recived a video frame")
        converted_CV_frame = bridgeObject.imgmsg_to_cv2(message, desired_encoding='bgr8')
        cv2.imshow("camera", converted_CV_frame)
        cv2.setMouseCallback('camera', click_event)


        #self.get_logger().info("height: %d" % converted_CV_frame.shape[1])
        #self.get_logger().info("width: %d" % converted_CV_frame.shape[0])
        #processsImage(converted_CV_frame)
        prespectiveWarp(converted_CV_frame, show = True)
        
        bird_view, bvl, bvr, minverse= prespectiveWarp(converted_CV_frame)
        img, hls, grayscale, threshold, blur, canny = processsImage(bird_view)

        histogram, left_base, right_base = plotHistorgram(threshold)

        ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(threshold, histogram)


        draw_info = general_search(threshold, left_fit, right_fit)
        curveRadius, curveDirection = measure_lane_curvature(ploty, left_fitx, right_fitx)
        avgPoints, result =  draw_lane_lines(converted_CV_frame, threshold, minverse, draw_info)
        deviation, directionDev = calcOffCenter(avgPoints, converted_CV_frame)
        finalImage = addText(result, curveRadius, curveDirection, deviation, directionDev)
        cv2.imshow("final", finalImage)

        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    imageSub = ImageSubscriber()
    rclpy.spin(imageSub)
    imageSub.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows


if __name__ == '__main__':
    main()