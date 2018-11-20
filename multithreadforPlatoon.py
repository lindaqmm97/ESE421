# threadtest.py

import time
import threading
import picamera
import cv2
import numpy as np
import struct
import smbus
import time 
from datetime import datetime
import os

maxX = 0
minX = 1440
maxY = 0
minY = 1080
centerX = 0
centerY = 0
width = 0



# this is something like navigation loop
def imageProcessing():
    while True:
        global maxX
        global minX
        global maxY
        global minY
        global centerX
        global centerY
        global width
        image = cv2.imread("orangeribbon.jpg")
        boundaries = [([0, 10, 80], [70, 56, 255])]
        for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

    # find the colors within the specified boundaries and apply
    # the mask
            mask = cv2.inRange(image, lower, upper)
            output = cv2.bitwise_and(image, image, mask = mask)
    #print(lower)

    # show the images
            cv2.imwrite("images.jpg", output)
            gray = cv2.cvtColor(output,cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray,100,150)

            minLineLength = 150
            maxLineGap = 20
            lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/180,threshold=50,minLineLength=minLineLength,maxLineGap=maxLineGap)


            for x in range(0, len(lines)):
                for x1,y1,x2,y2 in lines[x]:        
                    cv2.line(output,(x1,y1),(x2,y2),(0,128,0),2) # draw the line on the original image
                    if(x1 > maxX):
                        maxX = x1
                    if(x2 > maxX):
                        maxX = x2
                    if(x1 < minX):
                        minX = x1
                    if(x2 < minX):
                        minX = x2
                    if(y1 > maxY):
                        maxY = y1
                    if(y2 > maxY):
                        maxY = y2
                    if(y1 < minY):
                        minY = y1
                    if(y2 < minY):
                        minY = y2

            cv2.imwrite('orangeBoundary.jpg',output)
            centerX = (maxX+minX)/2
            centerY = (maxY+minY)/2
            width = maxX - minX
#    print(centerX)
#   print(centerY)
#    print(width)
            cv2.waitKey(2)




def getFloatData(oldFloats):
    try:
#        ts = time.time()
#       st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        data_received = bus.read_i2c_block_data(address, 1, 12)
        newFloats = [bytes_2_float(data_received, 0)]
        newFloats.append(bytes_2_float(data_received, 1))
        newFloats.append(bytes_2_float(data_received, 2))
    except:
        print("error reading float data")
        newFloats = oldFloats;

    return newFloats

def putByteList(byteList):
    try:
        bus.write_i2c_block_data(address, 255, byteList)
    except:
        print("error writing commands")
    return None

def bytes_2_float(data, index):
    bytes = data[4*index:(index+1)*4]
    return struct.unpack('f', "".join(map(chr, bytes)))[0]

bus = smbus.SMBus(1)
address = 0x04

dummyToPiFloats = [-3.1416, 6.2832, 3]
dummyToPiBytes = [2047, 50, 50]

# this is something like control loop
def communication(a,b,c):
#    writeData1 = [2]
#    writeData2 = [3]
    file = open("/home/pi/data_log.csv", "a")
    if os.stat("/home/pi/data_log.csv").st_size == 0:
        file.write("Time,Heading,Offset,Speed\n")
    while True:
        dummyToPiFloats2 = getFloatData(dummyToPiFloats)
        now = datetime.now()
        file.write(str(now)+","+str(dummyToPiFloats[0])+","+str(dummyToPiFloats[1])+","+str(dummyToPiFloats[2])+"\n")
        file.flush()
        putByteList([a,b,c])
        time.sleep(0.5)

# "Daemon" = runs forever with forced quit when main quits
slowT = threading.Thread(target=imageProcessing)
slowT.setDaemon(True)
slowT.start()
fastT = threading.Thread(target=communication(centerX,centerY,width))
fastT.setDaemon(True)
fastT.start()

    
