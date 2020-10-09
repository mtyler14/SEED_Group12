"""
This code finds ArUco tags in images captured with a raspberry pi camera and uses the location
of the tag in the image to control an arduino. This program interfaces with an arduino over I2C
and can both send and receive information. This code relies on a separate file to operate correctly. 
The other file, arudo_detection.py is imported as ar. 

To use this code make sure that the following connections are made between the raspberry pi and
the arduino: GND, SCL, SDA. 
"""

import smbus2
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from computer_vision import aruco_detection as ar
import sys


# Shows the quadrant on the LCD
def display_tag(t):
    lcd.cursor_position(11, 0)
    lcd.message = f"{t}     "


# Displays the wheel position on the LCD
def display_position(pos):
    lcd.cursor_position(10, 1)
    lcd.message = f"{pos}     "


# writes one byte to the Arduino
def write_number(value):
    bus.write_byte_data(arduino_address, 0, value)
    return -1


# reads one bytes from the Arduino
def read_number():
    number = bus.read_i2c_block_data(arduino_address, 0, 5)
    value = chr(number[0]) + chr(number[1]) + chr(number[2]) + chr(number[3]) + chr(number[4])


    return value


# create the I2C bus
bus = smbus2.SMBus(1)

# Initialize the LCD
lcd_columns = 16
lcd_rows = 2

i2c = busio.I2C(board.SCL, board.SDA)  # Initialise I2C bus.
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [0, 100, 0]  # Set LCD color to green

arduino_address = 0x04


if __name__ == '__main__':
    lcd.message = f"set point:"
    lcd.message = f"\nPosition:"

    # Code to calibrate the camera
    # selection = input("Would you like to run calibration? y/n")
    # if selection == 'y':
    #     print('Hold a piece of paper in front of the camera and press enter to calibrate.')
    #     input("The calibration will take 5 seconds")
    #     print("Calibrating")
    #     ar.calibration()
    old_quadrant = 1
    # Infinite loop to detect images and control the arduino
    while True:
        try:
            # Capture an image and get the x and y angles of the ArUco tag
            image = ar.capture_image()
            angle_x, angle_y = ar.marker_angle(image)

            # Get the quadrant that the tag is in
            if angle_x > 0 and angle_y > 0:
                quadrant = 1
            elif angle_x < 0 and angle_y > 0:
                quadrant = 2
            elif angle_x < 0 and angle_y < 0:
                quadrant = 3
            elif angle_x > 0 and angle_y < 0:
                quadrant = 4
            else:
                # If no tag is found then the last quadrant used will be reused
                quadrant = old_quadrant

            old_quadrant = quadrant

            # Show the quadrant and send it to the arduino
            display_tag(quadrant)
            write_number(quadrant)

            # read the wheel location from the arduino and display it on the LCD
            returned = read_number()
            display_position(returned)
            # time.sleep(.1)

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





