import picamera
import cv2
import numpy as np
#
# set up the camera
# default photo seems to be 1920 x 1080
# half of that keeps things more manageable for on-screen debugging
#
camera = picamera.PiCamera()
photoHeight = 540
camera.resolution = (16*photoHeight/9, photoHeight)
#
# captue an image and read it back in
# (Do this because picamera does not play nice with openCV?)
#
#camera.capture('blackRoad2.jpg')
imgColor = cv2.imread('blackRoad1.jpg') 
#
# convert to grayscale -- this seems to be standard for edge detection
# but we may ultimately want to do something more fancy to help us find
# the border between road and grass
#
img = cv2.cvtColor(imgColor, cv2.COLOR_BGR2GRAY)

#crop image
height, width = img.shape


#
#image blurring
blur=cv2.blur(img,(3,3))


#
# blur is standard to ensure that the edge goes along the entire
# length of the edge--noisy pixels on the edge can be excluded otherwise
# we may also want this to eliminate the texture of the road
#
img = cv2.GaussianBlur(blur,(5,5),0)
#
# "Canny" is the guy who invented the edge detection algorithm
# that is very widely used.
# the two arguments are the thresholds used in the algorithm
#
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
print(xright)
print(steeringAngleRight)

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
print(xleft)
print(steeringAngleLeft)

    
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

print("end")
# if you don't have this command at the end
#
cv2.waitKey(0)
