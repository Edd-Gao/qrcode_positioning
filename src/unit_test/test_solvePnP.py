import numpy as np
import tf_conversions
import cv2
import camera_calibration_parsers as parser

center_location = [0, 0]
half_length = 10

object_points = [
    [center_location[0] - half_length, center_location[1] - half_length, 0],
    [center_location[0] + half_length, center_location[1] - half_length, 0],
    [center_location[0] + half_length, center_location[1] + half_length, 0],
    [center_location[0] - half_length, center_location[1] + half_length, 0]
]

image_points = [
    [0, 0],
    [10, 10],
    [30, 40],
    [0, 20]
]

object_points = np.float64(np.array(object_points))
image_points = np.float64(np.array(image_points))
name, info = parser.readCalibration("rpi_cam_v2_640_480.yml")
distortion_coeffs = np.array(info.D)
camera_matrix = np.array(info.K).reshape((3, 3))


retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distortion_coeffs)

if retval:
    rmatrix, jabovians = cv2.Rodrigues(rvec)


exit(0)
