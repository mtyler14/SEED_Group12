

import smbus2
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from computer_vision import aruco_detection as ar
import sys


def display_tag(t):
    lcd.cursor_position(11, 0)
    lcd.message = f"{t}     "


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


bus = smbus2.SMBus(1)

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

    while True:
        try:
            image = ar.capture_image()
            angle_x, angle_y = ar.marker_angle(image)

            if angle_x > 0 and angle_y > 0:
                tag = 1
            elif angle_x < 0 and angle_y > 0:
                tag = 2
            elif angle_x < 0 and angle_y < 0:
                tag = 3
            elif angle_x > 0 and angle_y < 0:
                tag = 4
            else:
                tag = 1

            display_tag(tag)
            write_number(tag)

            returned = read_number()
            display_position(returned)
            # time.sleep(.1)
        except KeyboardInterrupt:
            sys.exit(0)

        except OSError as err:
            if err.errno == 121:
                # input("Press enter to reconnect to the I2C Bus")
                print("Reconnecting to the I2C bus")
                time.sleep(.5)
                bus = smbus2.SMBus(1)





