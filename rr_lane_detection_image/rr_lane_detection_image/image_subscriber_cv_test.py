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




#region_of_interest_verticies = [ (0, height), (width / 2, height / 2), (width, height)]

def imageToGrayscale(inputImage):
    #convert image grayscale, aplly threshold, blur & extract edges
    gray = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh, (3,3), 0)
    canny = cv2.Cany(blur, 40, 60)
    return gray, thresh, blur, canny

def processsImage(inputImage):
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

    cv2.imshow("Image", inputImage)
    cv2.imshow("hls filter", hls_result)
    cv2.imshow("grayscale", gray)
    cv2.imshow("threshholed image", thresh)
    cv2.imshow("blured image", blur)
    cv2.imshow("canny edge", canny)
    return inputImage, hls_result, gray, thresh, blur, canny






class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        global frameCount
        frameCount = 0
        self.subscription = self.create_subscription(Image, topicName, self.listner_callback, 10)
        self.subscription


    def listner_callback(self, message):
        bridgeObject = CvBridge()
        #frameCount += 1
        #self.get_logger().info("recived a video frame %d" % frameCount )
        self.get_logger().info("recived a video frame")
        converted_CV_frame = bridgeObject.imgmsg_to_cv2(message)
        processsImage(converted_CV_frame)
        cv2.imshow("camera", converted_CV_frame)

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