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

Xoffset = 0
SteeringAngle = 0


# this is something like navigation loop
def imageProcessing():
    while True:
        global SteeringAngle
        global Xoffset
        camera = picamera.PiCamera()
        photoHeight = 540
        camera.resolution = (16*photoHeight/9, photoHeight)
        camera.capture('blackRoad1.jpg')
        imgColor = cv2.imread('blackRoad1.jpg') 

        img = cv2.cvtColor(imgColor, cv2.COLOR_BGR2GRAY)

        height, width = img.shape
        blur=cv2.blur(img,(3,3))

        img = cv2.GaussianBlur(blur,(5,5),0)

        crop_img=img[2*height/5:height, 0:width]

        edges = cv2.Canny(crop_img,70,80)


#
# show the edges and save the edges image
# note that edges is effectively black-and-white, with white only
# along the edges
#
#cv2.imshow('Edges',edges)
        cv2.imwrite('edges98.jpg',edges)
#
# now look for lines with the Hough algorithm
# theta = resolution for slopes of lines (increments)
# threshold = how many points share the same line?
#
        minLineLength = 100
        maxLineGap = 20
        lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/180,threshold=50,minLineLength=minLineLength,maxLineGap=maxLineGap)
#5
# now parse through the lines that were found
# WE NEED TO IGNORE MOST OF THESE LINES
# AND ADD CODE TO EXTRACT RELATIVE HEADING AND OFFSET

        h=0.135
        f=0.003
        MaxY2right = 0
        MaxX2right = 0
        MaxY1right = 0
        MaxX1right = 0
#find max y2 in the lines in right half
        for x in range(0, len(lines)):
            for x1,y1,x2,y2 in lines[x]:
                if x2 >= width / 2:
                    if y2 >= MaxY2right:
                        MaxY2right = y2
                        MaxX2right = x2
                        MaxY1right = y1
                        MaxX1right = x1

        MaxY2left = 0
        MaxX2left = 0
        MaxY1left = 0
        MaxX1left = 0
#find max y2 in the lines
        for x in range(0, len(lines)):
            for x1,y1,x2,y2 in lines[x]:
                if x2 < width / 2:
                    if y2 >= MaxY2left:
                        MaxY2left = y2
                        MaxX2left = x2
                        MaxY1left = y1
                        MaxX1left = x1

        xright = 0;
        xleft = 0;
        steeringAngleLeft = 0;
        steeringAngleRight = 0;

#for the right half                        
        if MaxY2right!=MaxY1right:
            nright=(MaxX2right-MaxX1right)/(MaxY2right-MaxY1right)
            xright=nright*h
            if(xright<0):
                xright=-xright    
            steeringAngleRight= (((MaxX2right-MaxX1right)/(MaxY2right-MaxY1right)*height/2+(MaxX1right*MaxY2right-MaxX2right*MaxY1right)/(MaxY2right-MaxY1right)-width/2)/3779.52/f-43.2)/2.337
        else:
            xright=100000
            steeringAngleRight=np.pi/2
#        print(xright)
#        print(steeringAngleRight)

#for the left half
        if MaxY2left!=MaxY1left:
            nleft=(MaxX2left-MaxX1left)/(MaxY2left-MaxY1left)
            xleft=nleft*h
            if(xleft<0):
                xleft=-xleft
    
            steeringAngleLeft= ((MaxX1left*MaxY2left-MaxX2left*MaxY1left)/(MaxY2left-MaxY1left)/3779.52/f-43.2)/2.337
        else:
            xleft=10000
            steeringAngleLeft=np.pi/2

        if xright <= xleft:
            Xoffset = xright
            SteeringAngle = steeringAngleRight
        else:            
            Xoffset = xleft         
            SteeringAngle = steeringAngleLeft 

    
#draw lines
        for x in range(0, len(lines)):
            for x1,y1,x2,y2 in lines[x]:        
                cv2.line(imgColor,(x1,y1+2*height/5),(x2,y2+2*height/5),(0,0,255),2) # draw the line on the original image
#
# show the original image with the lines
#
#cv2.imshow('hough',imgColor)
        cv2.imwrite('hough98.jpg',imgColor)
#
# the graphics windows opened by CV2 seem to freak out

        #print("end")
# if you don't have this command at the end
#
        camera.close()
        cv2.waitKey(2)
#        time.sleep(10)



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

def putByteList2(desiredH):
    try:
        bus.write_i2c_block_data(address, 254, desiredH)
    except:
        print("error writing commands")
    return None

def bytes_2_float(data, index):
    bytes = data[4*index:(index+1)*4]
    return struct.unpack('f', "".join(map(chr, bytes)))[0]

def convertASCII(string):
    bytelist = list()
    bytelist.append(len(string))
    for i in range(0,len(string)):
        asciiCode = ord(string[i])
        bytelist.append(asciiCode)

#    print(bytelist)
    return bytelist
      


bus = smbus.SMBus(1)
address = 0x04

dummyToPiFloats = [-3.1416, 6.2832, 3]
dummyToPiBytes = [2047, 50, 50]

# this is something like control loop
def communication(a,b):
#    writeData1 = [2]
#    writeData2 = [3]
    file = open("/home/pi/data_log.csv", "a")
    if os.stat("/home/pi/data_log.csv").st_size == 0:
        file.write("Time,Heading,Offset,Speed\n")
    while True:
#        print(SteeringAngle)
#        print(Xoffset)
        dummyToPiFloats2 = getFloatData(dummyToPiFloats)
        now = datetime.now()
        file.write(str(now)+","+str(dummyToPiFloats[0])+","+str(dummyToPiFloats[1])+","+str(dummyToPiFloats[2])+"\n")
        file.flush()
        putByteList(convertASCII(str(Xoffset)))
        putByteList2(convertASCII(str(SteeringAngle)))
        time.sleep(0.5)

# "Daemon" = runs forever with forced quit when main quits
slowT = threading.Thread(target=imageProcessing)
slowT.setDaemon(True)
slowT.start()
fastT = threading.Thread(target=communication(Xoffset,SteeringAngle))
fastT.setDaemon(True)
fastT.start()

    
