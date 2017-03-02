# coding=utf-8
import sys
from Queue import Queue
from collections import deque
import cv2
import numpy as np
import math
import time
import zbar

# ROS libraries
import rospy

# import compressed image message
from sensor_msgs.msg import CompressedImage


class OffboardPos:
    # parameters
    canny_1 = 200  # Canny lower threshold
    canny_2 = 600  # Canny higher threshold
    canny_a = 3  # Canny apeture threshold
    gau_k = 1  # Gaussian blur kernel size
    gau_sigma = 2  # Gaussian blur sigma
    bi_threshold = 150  # binary image threshold
    timing_threshold = 5  # timing pattern variance threshold

    def __init__(self):
        self.imgSub = rospy.Subscriber("/camera/image_mono/compressed", CompressedImage, self.callback, queue_size=1)

        self.scanner = zbar.ImageScanner()

        self.scanner.parse_config("disable")
        # scanner.enable_cache() # do not enable cache here
        self.scanner.parse_config("qrcode.enable")

    def create_line_iterator(self, p1, p2, img):
        """
        Produces and array that consists of the coordinates and intensities of each pixel in a line between two points

        Parameters:
            -p1: a numpy array that consists of the coordinate of the first point (x,y)
            -p2: a numpy array that consists of the coordinate of the second point (x,y)
            -img: the image being processed

        Returns:
            -it: a numpy array that consists of the coordinates and intensities of each pixel in the radii (shape: [numPixels, 3], row = [x,y,intensity])
        """
        # define local variables for readability
        image_h = img.shape[0]
        image_w = img.shape[1]
        p1_x = p1[0]
        p1_y = p1[1]
        p2_x = p2[0]
        p2_y = p2[1]

        # difference and absolute difference between points
        # used to calculate slope and relative location between points
        dX = p2_x - p1_x
        dY = p2_y - p1_y
        dXa = np.abs(dX)
        dYa = np.abs(dY)

        # predefine numpy array for output based on distance between points
        it_buffer = np.empty(shape=(np.maximum(dYa, dXa), 3), dtype=np.float32)
        it_buffer.fill(np.nan)

        # Obtain coordinates along the line using a form of Bresenham's algorithm
        neg_y = p1_y > p2_y
        neg_x = p1_x > p2_x
        if p1_x == p2_x:  # vertical line segment
            it_buffer[:, 0] = p1_x
            if neg_y:
                it_buffer[:, 1] = np.arange(p1_y - 1, p1_y - dYa - 1, -1)
            else:
                it_buffer[:, 1] = np.arange(p1_y + 1, p1_y + dYa + 1)
        elif p1_y == p2_y:  # horizontal line segment
            it_buffer[:, 1] = p1_y
            if neg_x:
                it_buffer[:, 0] = np.arange(p1_x - 1, p1_x - dXa - 1, -1)
            else:
                it_buffer[:, 0] = np.arange(p1_x + 1, p1_x + dXa + 1)
        else:  # diagonal line segment
            steep_slope = dYa > dXa
            if steep_slope:
                slope = dX.astype(np.float32) / dY.astype(np.float32)
                if neg_y:
                    it_buffer[:, 1] = np.arange(p1_y - 1, p1_y - dYa - 1, -1)
                else:
                    it_buffer[:, 1] = np.arange(p1_y + 1, p1_y + dYa + 1)
                it_buffer[:, 0] = (slope * (it_buffer[:, 1] - p1_y)).astype(np.int) + p1_x
            else:
                slope = dY.astype(np.float32) / dX.astype(np.float32)
                if neg_x:
                    it_buffer[:, 0] = np.arange(p1_x - 1, p1_x - dXa - 1, -1)
                else:
                    it_buffer[:, 0] = np.arange(p1_x + 1, p1_x + dXa + 1)
                it_buffer[:, 1] = (slope * (it_buffer[:, 0] - p1_x)).astype(np.int) + p1_y

        # Remove points outside of image
        col_x = it_buffer[:, 0]
        col_y = it_buffer[:, 1]
        it_buffer = it_buffer[(col_x >= 0) & (col_y >= 0) & (col_x < image_w) & (col_y < image_h)]

        # Get intensities from img ndarray
        it_buffer[:, 2] = img[it_buffer[:, 1].astype(np.uint), it_buffer[:, 0].astype(np.uint)]

        return it_buffer

    def cv_distance(self, p, q):
        """
        calculate the distance between two opencv points
        :param p: one 2-dimension numpy array
        :param q: another 2-dimension numpy array
        :return: return the distance between two points
        """
        return int(math.sqrt(pow((p[0] - q[0]), 2) + pow((p[1] - q[1]), 2)))

    def draw_short_connections(self, a, b, draw_img):
        """
        draw the two shortest connection lines between two Position Detection Pattern box corners
        :param draw_img: image input to draw
        :param a: Position Detection Pattern box corner array a
        :param b: Position Detection Pattern box corner array b
        :return: no return
        """
        # 存储 ab 数组里最短的两点的组合
        s1_ab = ()
        s2_ab = ()
        # 存储 ab 数组里最短的两点的距离，用于比较
        s1 = np.iinfo('i').max
        s2 = s1
        for ai in a:
            for bi in b:
                d = self.cv_distance(ai, bi)
                if d < s2:
                    if d < s1:
                        s1_ab, s2_ab = (ai, bi), s1_ab
                        s1, s2 = d, s1
                    else:
                        s2_ab = (ai, bi)
                        s2 = d
        a1, a2 = s1_ab[0], s2_ab[0]
        b1, b2 = s1_ab[1], s2_ab[1]
        a1 = (a1[0] + (a2[0] - a1[0]) * 1 / 14, a1[1] + (a2[1] - a1[1]) * 1 / 14)
        b1 = (b1[0] + (b2[0] - b1[0]) * 1 / 14, b1[1] + (b2[1] - b1[1]) * 1 / 14)
        a2 = (a2[0] + (a1[0] - a2[0]) * 1 / 14, a2[1] + (a1[1] - a2[1]) * 1 / 14)
        b2 = (b2[0] + (b1[0] - b2[0]) * 1 / 14, b2[1] + (b1[1] - b2[1]) * 1 / 14)

        # 将最短的两个线画出来
        cv2.line(draw_img, a1, b1, (0, 0, 255), 1)
        cv2.line(draw_img, a2, b2, (0, 0, 255), 1)

    def check(self, a, b, binary_img, threshold):
        """
        check whether the two input Position Detection Pattern box a and b are in one qr code
        :param threshold: threshold for timing pattern judgement
        :param a: Position Detection Pattern box corner array a
        :param b: Position Detection Pattern box corner array b
        :param binary_img: the binary image containing the qrcode
        :return: boolean judgement, true if the two Position Detection Patterns are in one qr code
        """
        # 存储 ab 数组里最短的两点的组合
        s1_ab = ()
        s2_ab = ()
        # 存储 ab 数组里最短的两点的距离，用于比较
        s1 = np.iinfo('i').max
        s2 = s1
        for ai in a:
            for bi in b:
                d = self.cv_distance(ai, bi)
                if d < s2:
                    if d < s1:
                        s1_ab, s2_ab = (ai, bi), s1_ab
                        s1, s2 = d, s1
                    else:
                        s2_ab = (ai, bi)
                        s2 = d
        a1, a2 = s1_ab[0], s2_ab[0]
        b1, b2 = s1_ab[1], s2_ab[1]
        a1 = (a1[0] + (a2[0] - a1[0]) * 1 / 14, a1[1] + (a2[1] - a1[1]) * 1 / 14)
        b1 = (b1[0] + (b2[0] - b1[0]) * 1 / 14, b1[1] + (b2[1] - b1[1]) * 1 / 14)
        a2 = (a2[0] + (a1[0] - a2[0]) * 1 / 14, a2[1] + (a1[1] - a2[1]) * 1 / 14)
        b2 = (b2[0] + (b1[0] - b2[0]) * 1 / 14, b2[1] + (b1[1] - b2[1]) * 1 / 14)

        return self.is_timing_pattern(self.create_line_iterator(a1, b1, binary_img)[:, 2],
                                      threshold) or self.is_timing_pattern(
            self.create_line_iterator(a2, b2, binary_img)[:, 2], threshold)

    def is_timing_pattern(self, line, threshold):
        """
        to judge wether the input line is a timing pattern
        :param threshold: threshold for timing pattern judgement
        :param line: input line
        :return: true if the line is a timing pattern
        """

        # ignore too short line
        if len(line) < 10:
            return False

        # 除去开头结尾的白色像素点
        # while line[0] != 0:
        #    line = line[1:]
        # while line[-1] != 0:
        #    line = line[:-1]
        # 计数连续的黑白像素点
        c = []
        count = 1
        l = line[0]
        for p in line[1:]:
            if p == l:
                count += 1
            else:
                # 排除一些太小的线段，这些可能是噪声
                if count > 2:
                    c.append(count)
                count = 1
            l = p
        if count > 1:
            c.append(count)
        # 如果黑白间隔太少，直接排除
        if len(c) < 5:
            return False
        # 计算方差，根据离散程度判断是否是 Timing Pattern
        # threshold = 5
        return np.var(c) < threshold

    def find_contour_corner(self, contour):
        """
        find the Position Detection Pattern contour corners
        :param contour: the contour end points in a ndarray
        :return: if the algorithm succeed, it returns the corners of the contour in a ndarray, or it returns the corners of the minimum area rectangle containing the pattern
        """
        i = 0
        # temp_result = np.array([[0,0]])
        corners = np.array([[0, 0]])
        temp_result = Queue()
        while i < len(contour) - 1:
            if contour[i].item(0) == contour[i + 1].item(0):
                # temp_result = np.append(temp_result, contour[i:i+1], axis=0)
                temp_result.put(contour[i], False)
                temp_result.put(contour[i + 1], False)
                i += 2
            else:
                i += 1

        temp_result.put(temp_result.get())

        threshold = 10
        while not temp_result.empty():
            temp_a = temp_result.get()
            temp_b = temp_result.get()
            if self.cv_distance(temp_a[0], temp_b[0]) > threshold:
                corners = np.append(corners, temp_a, axis=0)
                corners = np.append(corners, temp_b, axis=0)

        if corners.size == 10:
            return corners[1:]
        else:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            return box

    def is_valid_symbol(self, symbol_string):
        return True

    def scan(self, boxes, gray_img, zbar_scanner, bi_threshold, timing_pattern_threshold):
        """
        this function scan the image and select one area that contains a valid qr code
        :param bi_threshold:
        :param timing_pattern_threshold:
        :param zbar_scanner:
        :param gray_img:
        :param boxes:
        :return:
        """
        he_img = cv2.equalizeHist(gray_img)

        pixels = np.array([])
        # meter the average intensity of the boxes

        if len(boxes) > 0:

            for box in boxes:
                box_x, box_y, box_w, box_h = cv2.boundingRect(np.array(box))
                temp_img = gray_img[box_y:box_y + box_h, box_x:box_x + box_w]
                pixels = np.append(pixels, temp_img)
            bi_threshold = np.average(pixels)

        th, bi_img = cv2.threshold(he_img, bi_threshold, 255, cv2.THRESH_BINARY)

        current_state = "ZERO"  # state includes "ZERO", "ONE", "TWO", "MORE","FAIL"
        next_state = "ZERO"
        qr_boxes_candidate = deque()
        success = False
        result_symbol = ""
        candidate_i = deque()

        while True:
            current_state = next_state

            if current_state == "ZERO":
                if len(boxes) < 3:
                    next_state = "FAIL"
                    continue
                candidate_i.clear()
                candidate_i.append(0)
                for i in range(1, len(boxes)):
                    if self.check(boxes[0], boxes[i], bi_img, timing_pattern_threshold):
                        candidate_i.append(i)

                valid_count = len(candidate_i)
                if valid_count == 1:
                    boxes = boxes[1:]
                    next_state = "ZERO"
                elif valid_count == 2:
                    qr_boxes_candidate.append(boxes[candidate_i[0]])
                    qr_boxes_candidate.append(boxes[candidate_i[1]])
                    boxes = np.delete(boxes, candidate_i, axis=0)
                    next_state = "ONE"
                elif valid_count == 3:
                    qr_boxes_candidate.append(boxes[candidate_i[0]])
                    qr_boxes_candidate.append(boxes[candidate_i[1]])
                    qr_boxes_candidate.append(boxes[candidate_i[2]])
                    boxes = np.delete(boxes, candidate_i, axis=0)
                    next_state = "TWO"
                else:
                    for x in candidate_i:
                        qr_boxes_candidate.append(boxes[x])
                    boxes = np.delete(boxes, candidate_i, axis=0)
                    next_state = "MORE"
            elif current_state == "ONE":
                if len(boxes) == 0:
                    next_state = "FAIL"
                    continue
                candidate_i.clear()
                for i in range(0, len(boxes)):
                    if self.check(qr_boxes_candidate[1], boxes[i], bi_img, timing_pattern_threshold):
                        candidate_i.append(i)
                valid_count = len(candidate_i)
                if valid_count == 0:
                    qr_boxes_candidate.clear()
                    next_state = "ZERO"
                elif valid_count == 1:
                    qr_boxes_candidate.append(boxes[candidate_i[0]])
                    boxes = np.delete(boxes, candidate_i, axis=0)
                    next_state = "TWO"
                else:
                    for x in candidate_i:
                        qr_boxes_candidate.append(boxes[x])
                    boxes = np.delete(boxes, candidate_i, axis=0)
                    next_state = "MORE"
            elif current_state == "TWO":
                x, y, w, h = cv2.boundingRect(np.array(qr_boxes_candidate).reshape((12, 2)))
                x -= 30
                y -= 30
                w += 30
                h += 30
                if x < 0:
                    x = 0
                if y < 0:
                    y = 0

                qr_zone_img = gray_img[y:y + h, x:x + w]
                zbar_img = zbar.Image(w, h, 'Y800', qr_zone_img.tostring())
                zbar_scanner.scan(zbar_img)
                for symbol in zbar_img:
                    if self.is_valid_symbol(symbol):
                        success = True
                        result_symbol = symbol
                del zbar_img
                if (success):
                    break
                else:
                    qr_boxes_candidate.clear()
                    next_state = "ZERO"
            elif current_state == "MORE":
                success = False
                result_symbol = ""
                break
            elif current_state == "FAIL":
                # handle exceptional situations
                success = False
                result_symbol = ""
                break
            else:
                pass

        return [success, result_symbol]

    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        img_gray = cv2.imdecode(np_arr, 0)

        start = time.clock()

        img_he = cv2.equalizeHist(img_gray)

        img_gb = cv2.GaussianBlur(img_he, (self.gau_k, self.gau_k), self.gau_sigma)

        edges = cv2.Canny(img_gb, self.canny_1, self.canny_2, self.canny_a)

        # find contours in the image

        img_fc, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return

        hierarchy = hierarchy[0]
        found = []

        # find the qr code positioning patterns in the image
        for i in range(len(contours)):
            k = i
            c = 0
            while hierarchy[k][2] != -1:
                k = hierarchy[k][2]
                c += 1
            if c >= 5:
                found.append(i)

        # find the contour corners and draw in the image
        draw_img = img_gray.copy()
        boxes = []
        for i in found:
            box = self.find_contour_corner(contours[i])
            cv2.drawContours(draw_img, [box], 0, (0, 0, 255), 2)
            box = map(tuple, box)
            boxes.append(box)

        cv2.imshow("boxes",draw_img)
        cv2.waitKey(20)

        success, symbol = self.scan(boxes, img_gb, self.scanner, self.bi_threshold, self.timing_threshold)

        if success:
            rospy.loginfo(symbol.data)
            print symbol.data
            end = time.clock()
            print str((end - start) * 1000) + "ms"
        else:
            pass
            # print "scanning failed."


def main(args):
    rospy.init_node('offoard_pos', anonymous=True)
    positioner = OffboardPos()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
