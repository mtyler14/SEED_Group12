

import smbus2
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from computer_vision import aruco_detection as ar


# writes one byte to the Arduino
def write_number(value):
    bus.write_byte_data(arduino_address, 0, value)
    return -1


# reads one bytes from the Arduino
def read_number():
    number = bus.read_byte(arduino_address)
    return number


bus = smbus2.SMBus(1)

lcd_columns = 16
lcd_rows = 2

i2c = busio.I2C(board.SCL, board.SDA)  # Initialise I2C bus.
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [0, 100, 0]  # Set LCD color to green

arduino_address = 0x04


def display_tag(tag):
    lcd.clear()
    lcd.message = f"set point: {tag}"

if __name__ == '__main__':
    image = ar.capture_image()
    _, ids = ar.detect_markers(image, verbose=False)
    print(f'The ids are {ids}')

    if ids is not None:
        tag = ids[0][0]
    else:
        tag = 47

    display_tag(tag)
    write_number(tag)
    time.sleep(1)
    

    lcd.message = f"\nPosition:"
    time.sleep(1)
    
    for i in range(20):
        returned = read_number()
        lcd.cursor_position(10,1)
        lcd.message = f"{i}     "
        time.sleep(.25)
