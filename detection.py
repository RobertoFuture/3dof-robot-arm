import cv2 as cv
import numpy as np
import serial

capture = cv.VideoCapture(1)
_, frame = capture.read()
rows, cols, _ = frame.shape

x_middle = int(cols / 2)            # object frame lenght to internal middle point
y_middle = int(rows / 2)            # object frame height to internal middle point

xmid = 640 / 2                      # video capture window length to middle point
ymid = 480 / 2                      # video capture window height to middle point

dx = 40
dy = 40

# boolean = 1

# port configuration

ser = serial.Serial()
ser.port = 'COM6'                  # set port
ser.baudrate = 19200                 # to confirm
ser.bytesize = serial.EIGHTBITS     # set bytesize to eight bits
ser.open()                          # open serial port

receivedData = 0
somechar = 0

is_sweep = 0
is_stab = 0


def sweep():
    global receivedData
    global someChar

    global is_sweep

    if y_middle < ymid - dy:        # base servo instruction: rotate anticlockwise
        is_sweep = 0

        while True:
            ser.write(b'a')
            print("A")

            if ser.in_waiting > 0:
                print(ser.in_waiting)
                receivedData = ser.read(1)
                print(receivedData)
                someChar = receivedData.decode('ascii')
                print(someChar)

                if someChar == 's':
                    break

    elif y_middle > ymid + dy:      # base servo instruction: rotate clockwise
        is_sweep = 0

        while True:
            ser.write(b'b')
            print("B")

            if ser.in_waiting > 0:
                print(ser.in_waiting)
                receivedData = ser.read(1)
                print(receivedData)
                someChar = receivedData.decode('ascii')
                print(someChar)

                if someChar == 's':
                    break

    else:
        is_sweep = 1                 # base servo instruction: stay still


def stab():
    global receivedData
    global someChar

    global is_stab

    if x_middle < xmid - dx:            # shoulder and elbow servos instruction: move 'forwards'
        is_stab = 0

        while True:
            ser.write(b'c')
            print("C")

            if ser.in_waiting > 0:
                print(ser.in_waiting)
                receivedData = ser.read(1)
                print(receivedData)
                someChar = receivedData.decode('ascii')
                print(someChar)

                if someChar == 's':
                    break

    elif x_middle > xmid + dx:           # shoulder and elbow servos instruction: move 'backwards'
        is_stab = 0

        while True:
            ser.write(b'd')
            print("D")

            if ser.in_waiting > 0:
                print(ser.in_waiting)
                receivedData = ser.read(1)
                print(receivedData)
                someChar = receivedData.decode('ascii')
                print(someChar)

                if someChar == 's':
                    break

    else:
        is_stab = 1


def initcont():            # general instruction: go to initial and contraction position
    global receivedData
    global someChar

    while True:
        ser.write(b't')
        print("T")

        if ser.in_waiting > 0:
            print(ser.in_waiting)
            receivedData = ser.read(1)
            print(receivedData)
            someChar = receivedData.decode('ascii')
            print(someChar)

            if someChar == 's':
                break


while True:
    _, frame = capture.read()
    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)     # why?

    # set interval for some colour
    low_colour = np.array([0, 0, 188])
    high_colour = np.array([108, 255, 255])
    colour_mask = cv.inRange(hsv_frame, low_colour, high_colour)

    # create contour
    contours, _ = cv.findContours(colour_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)   # why?
    contours = sorted(contours, key=lambda x: cv.contourArea(x), reverse=True)

    # create contour
    # contoursy, _ = cv.findContours(colour_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)   # why?
    # contoursy = sorted(contoursx, key=lambda y: cv.contourArea(y), reverse=True)

    # frame biggest area of such colour
    for cnt in contours:
        (x, y, w, h) = cv.boundingRect(cnt)

        x_middle = int((x + x + w) / 2)
        y_middle = int((y + y + h) / 2)
        # print(cv.contourArea(cnt))

        break

    # draw line that passes through the middle of the square that frames area
    for cnt in contours:
        if 0 < cv.contourArea(cnt):
            cv.line(frame, (x_middle, 0), (x_middle, 480), (0, 255, 0), 2)
            cv.line(frame, (0, y_middle), (640, y_middle), (0, 255, 0), 2)
        break

    cv.imshow('frame', frame)

    # send data to PIC via serial communication
    # if ser.in_waiting > 0:
    #     boolean = 1

    # print(ser.in_waiting)

    for cnt in contours:

        if 0 < cv.contourArea(cnt):
            sweep()
            stab()

            if is_stab and is_sweep:
                while True:
                    ser.write(b'j')
                    print("J")

                    if ser.in_waiting > 0:
                        print(ser.in_waiting)
                        receivedData = ser.read(1)
                        print(receivedData)
                        someChar = receivedData.decode('ascii')
                        print(someChar)

                        if someChar == 's':
                            break

        else:
            initcont()

        break

    key = cv.waitKey(1)

    if key == 27:
        break

capture.release()
cv.destroyAllWindows()      # closes windows after execution

ser.close()                 # closes port
