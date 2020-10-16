"""
TODO
"""

import cv2 as cv
import numpy as np
import picamera
import picamera.array
import smbus2, time, board, bus.io
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

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


# Find the angle between a marker and the camera
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

    return round(angle, 2)


# Displays the angle on the LCD
def display_angle(angle):
    lcd.cursor_position(8, 1)
    lcd.message = f"{angle}     "


if __name__ == "__main__":
    # create the I2C bus
    bus = smbus2.SMBus(1)

    # Initialize the LCD
    lcd_columns = 16
    lcd_rows = 2

    i2c = busio.I2C(board.SCL, board.SDA)  # Initialise I2C bus.
    # Initialise the LCD class
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.color = [0, 100, 0]  # Set LCD color to green

    lcd.message("Angle: ")

        while True:
            try:
                old_angle = None
                # Capture an image and get the x and y angles of the ArUco tag
                image = capture_image()
                angle_x = marker_angle_better(image)

                if angle_x != old_angle:
                    display_angle(angle)    
                    old_angle = angle_x
                else:
                    pass            

            # Catch keyboard interrupts to exit cleanly from the program
            except KeyboardInterrupt:
                sys.exit(0)

            # If the I2c connection is lost then try to reconnect every half second
            except OSError as err:
                if err.errno == 121:
                    # input("Press enter to reconnect to the I2C Bus")
                    print("Reconnecting to the I2C bus")
                    time.sleep(.5)
                    bus = smbus2.SMBus(1)
   


