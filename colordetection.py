import numpy as np
import cv2

image = cv2.imread("orangeribbon.jpg")
boundaries = [([0, 10, 80], [70, 56, 255])]
maxX = 0
minX = 1440
maxY = 0
minY = 1080
centerX = 0
centerY = 0
width = 0


# loop over the boundaries
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

#calculate distance from centroid location
focal_length = .304 #cm
field_of_view = 48.8 #degrees
resolution = 2464 #pixels
self.pix2dist = (focal_length * tan(pi/180*field_of_view/2)) / (resolution / 2)
points = [centerX, centerY]; 
def transform(self,points,planeN=[0,0,1],planeT=[0,0,0]):
if points is None:
return None
planeN = np.reshape(planeN,(3,))
planeT = np.reshape(planeT,(3,))
output = []
if type(points) != list and type(points) != tuple:
points = [points]
for point in points:
point = np.reshape(point,(2,))
point = np.concatenate(([self.camFocal], point * self.pix2dist), axis=0)
point = self.camR @ point # rotate to the world frame
c = (-np.dot(self.camT,planeN) + np.dot(planeT,planeN)) / np.dot(point,planeN)
transformed = c * point + self.camT
output.append(transformed)
return output

    cv2.waitKey(2)
