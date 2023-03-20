## References:
##
## pyimagesearch - Ball Tracking with OpenCV:
## https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
##
## Sentdex: OpenCV tutorials
## https://www.youtube.com/watch?v=Z78zbnLlPUA&list=PLQVvvaa0QuDdttJXlLtAJxJetJcqmqlQq
##

import numpy as np
import cv2
import serial
import time
import struct

Arduino = serial.Serial('COM3', baudrate = 115200, timeout = 0, writeTimeout = 0)
Arduino.flushInput()                                # Flush input buffer for good measure
time.sleep(2)                                       # Give time to setup connection
cap = cv2.VideoCapture(0)                           # Start video capture using default camera
x = 0

while True:
    _, frame = cap.read()                           # The frame size is 480x640
    blurred = cv2.GaussianBlur(frame, (11,11), 0)   # Blur image to smooth everything out
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  # Convert to HSV to be able to see only the ball
    lower = np.array([12, 122, 180])                # Lower HSV values of the ball
    upper = np.array([42, 255, 255])                # Upper HSV values of the ball
    mask = cv2.inRange(hsv, lower, upper)           # Show the pixels that are in the HSV value range in white only 
    mask = cv2.erode(mask, None, iterations = 2)    # Remove white noise
    mask = cv2.dilate(mask, None, iterations = 2)   # Enlarge the image after eroding it (Common place to erode then dilate)
    cnt = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]  #Find contours, added [0] at the end to only get the relevant values
    # Each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.
    center = ""
    if len(cnt) > 0:                                # If found a contour
        c = max(cnt, key=cv2.contourArea)           # Getting the largest contour. Largest defined by largest area
        ((x, y), radius) = cv2.minEnclosingCircle(c)# Getting the position and radius of smallest enclosing circle of the found contour
        M = cv2.moments(c)                          # Finding image moment, detailed in documentation exactly what it is and how it's used.
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))   # Formula shown in documentation
        if radius > 15:                             #Make sure not locking in on noise
            cv2.circle(frame, center, 5, (0, 0, 255), -1)   # Draw small circle around center of ball
##    cv2.imshow("mask1", frame)                      # Show the OG frame with a small circle indicating the ball center
##    cv2.imshow("mask2", mask)                       # Show only the ball in white



# # In the future, if I want to read numbers sent as a string in bytes, I can use the following code:
# ser = serial.Serial()
# ser.baudrate = 115200
# ser.port = 'COM11'
# ser.open()
# while(1)
#   if(ser.in_waiting > 0):
#       # Decode to string, then, remove the ending string, \r\n in this case.
#       serialString = ser.readline().decode('UTF-8').replace("\r\n", "")


#### Communication w/ Arduino
##  Wait until Arduino asks for the data    
    while ((Arduino.read()) != b'!'):
        pass
##  Send the ball X position in the correct format (Little endian, unsigned short)
    Arduino.write(struct.pack('<H', int(x)))

####  For debugging, read data sent from Arduino
##  Wait until Arduino Says it's about to send data    
##    while ((Arduino.read()) != b'>'):
##        pass
###  Read 2 bytes and pack them in the correct format (short)
##    raw = Arduino.read(2)
##    val = struct.unpack('h', raw)
###  Print Data sent to Arduino and data received from Arduino
##    print(int(x), val[0], sep=', ')
####
    print(int(x))                                   # Print ball position
    x = 0
    k = cv2.waitKey(5) & 0xFF   # Standard. Pressing ESC key will cause break
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
    

    
