"""
TODO
"""

import cv2 as cv
import numpy as np
import picamera
import picamera.array
import time

# calibrates the white balance of the camera
def calibration():
    with picamera.PiCamera() as camera:
        time.sleep(1)
        camera.awb_mode = 'auto'

        red, blue = [], []
        samples, sample_time = 10, .5
        for i in range(samples):
            r, b = camera.awb_gains
            red.append(r)
            blue.append(b)
            time.sleep(sample_time)

        avg_red = sum(red) / samples
        avg_blue = sum(blue) / samples

        camera.awb_mode = 'off'
        camera.awb_gains = (avg_red, avg_blue)
        print("Calibrated")
        return None


# Captures an image from the camera
def capture_image(need_input=False, display_image=False):
    # Ask the user for a filename if it is needed
    if need_input:
        file_name = input("Enter a file name: ")

    # Capture an image from the camera
    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.capture(stream, format='bgr')
            img = stream.array

            # Write the image to a file
            if need_input is not False:
                cv.imwrite(file_name, img)

            # Display the image or do nothing
            if display_image:
                cv.imshow("Captured Image", img)
                cv.waitKey(0)
            else:
                pass
    return img


# Converts a BRG image to grayscale
def to_grayscale(image=None, display_image=False):
    # Captures an image if one was not provided
    if image is None:
        image = capture_image()

    # Converts to grayscale
    grayscale = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    # Displays the image if it was requested
    if display_image:
        cv.imshow('Grayscale', grayscale)
        cv.waitKey(0)
    return grayscale


# Detect aruco markers in an image
def detect_markers(image=None, verbose=True):
    # Get an image if one was not provided
    if image is None:
        image = capture_image()
    # Convert to grayscale, load aruco dict and parameters, and find tags
    gray = to_grayscale(image)
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
    parameters = cv.aruco.DetectorParameters_create()

    # Sets the min perimeter for a tag to be detected
    parameters.minMarkerPerimeterRate = .03
    # Detects marker corners and IDS
    marker_corners, marker_ids, rejected_candidates = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    # If requested, output the found markers
    if verbose:
        if marker_ids is not None:
            print(f'The ids of the markers in the image are {marker_ids}')
        else:
            print("No markers found")
    # return the markers corners and ids
    return marker_corners, marker_ids


# Find marker's angle and distance from camera
def marker_angle(image=None):
    # Get the angle measurements
    def angle(c, f):
        # If field of view is known the following formula can be used too
        # theta = arctan(px * tan(fov / 2) / (sensor_x / 2))

        # [x, y] pairs for the marker corners
        p1, p2, p3, p4 = c[0], c[1], c[2], c[3]
        tag_center_x = int((p1[0] + p2[0] + p3[0] + p4[0]) / 4)
        tag_center_y = int((p1[1] + p2[1] + p3[1] + p4[1]) / 4)

        # Get the x and y difference from the center of the image
        # Positive is higher in the image
        y_coord = center_y - tag_center_y
        # Positive is farther right in the image
        x_coord = tag_center_x - center_x

        # Find the angle based on the location and focal length
        theta_y = np.arctan(y_coord / f)
        theta_x = np.arctan(x_coord / f)

        # Convert radians to degrees
        deg_y = round(np.rad2deg(theta_y), 2)
        deg_x = round(np.rad2deg(theta_x), 2)

        # print(f'The horizontal angle from the camera is {deg_x} degrees')
        # print(f'The vertical angle from the camera is {deg_y} degrees')

        return deg_x, deg_y

    # Capture and image if one was not supplied
    if image is None:
        image = capture_image()

    # Image parameters
    y, x, z = np.shape(image)
    center_x = int(x / 2)
    center_y = int(y / 2)

    # Camera focal length and sensor size in mm. Camera v2.1 3.04 and 2.760 respectively
    focal = 3.04  # mm
    sensor_size = 2.760  # mm
    pixels_per_mm = y / sensor_size  # Ratio for converting pixels to mm based on the pixel size and sensor size
    focal_pixel = int(pixels_per_mm * focal)  # Focal length in terms of pixels

    # Get the Aruco markers in the image. If there aren't any then return nothing
    corners, ids = detect_markers(image, verbose=False)
    if not corners:
        return 0, 0

    corners = corners[0][0] 

    # get the angles and distance
    angle_x, angle_y = angle(corners, focal_pixel)

    return angle_x, angle_y


def marker_angle_better(image):
    # Side length of the markers in centimeters
    marker_side_length = 4
    half_length = marker_side_length / 2

    # Corners of the markers from the marker's coordinate system
    marker_corners = np.array([[-half_length,    -half_length,    0],     # top left
                                [half_length,    -half_length,    0],     # top right
                                [half_length,    half_length,     0],     # bottom right
                                [-half_length,   half_length,     0]])    # bottom left

    corners, ids = detect_markers(image)
    if not corners:
        print("No markers found")
        return None
    # Get the first tag from the detected tags
    corners = np.asarray(corners[0][0]).astype(float)

    # Find the rotation and translation matrices for the tag
    ret, r_vec, t_vec = cv.solvePnP(marker_corners, corners, k, None)

    x = t_vec[0][0]
    z = t_vec[2][0]

    angle = np.arctan(x / z)
    angle = np.rad2deg(angle)
    angle = -angle  # For some reason to the left of the camera center is supposed to be positive

    return angle


if __name__ == "__main__":
    print("This program does nothing by itself")


