# adapted from code at https://gist.github.com/gileri/5a9285d6a1cfde142260
#
# check out this information about the "command" parameter (second argument in block read / write)
# https://raspberrypi.stackexchange.com/questions/8469/meaning-of-cmd-param-in-write-i2c-block-data
#
import struct
import smbus
import time 
from datetime import datetime
import os
#import pandas as pd
#
# 1 = command byte to request first data block from Arduino
# 8 = number of bytes (one 4-byte float + one 2-byte word)
#
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

#
# 2 = command byte to request second data block from Arduino
# 4 = number of bytes (one 2-byte word + two bytes)
#
#def getByteData(oldBytes):
#    try:
#        data_received = bus.read_i2c_block_data(address, 2, 4)
#        newBytes = [data_received[0]*255 + data_received[1]]
#        newBytes.append(data_received[2])
#        newBytes.append(data_received[3])
#    except:
#        print("error reading byte data")
#        newBytes = oldBytes;

#    return newBytes

#
# 255 = command byte to initiate writing to Arduino
# (arbitrary--must be different than read)
#
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
#
# crazy conversion of groups of 4 bytes in an array into a float
# simple code assumes floats are at beginning of the array
# "index" = float index, starting at 0
#
def bytes_2_float(data, index):
    bytes = data[4*index:(index+1)*4]
    return struct.unpack('f', "".join(map(chr, bytes)))[0]

def convertASCII(string):
    bytelist = list()
    bytelist.append(len(string))
    for i in range(0,len(string)):
        asciiCode = ord(string[i])
        bytelist.append(asciiCode)

    print(bytelist)
    return bytelist
        

##########
# main part of script starts here
##########

#
# smbus implements i2c on the RPi
#
bus = smbus.SMBus(1)

#
# this is the Slave address of the Arduino
#
address = 0x04

#
# initialize dummy value of output from Pi (bytes only)
#
t = 251.1545367
writeData = str(t)
#print(ord(writeData[1]))
writeData2 = [3]


#
# initialize dummy values of inputs to Pi
#
dummyToPiFloats = [-3.1416, 6.2832, 3.10]
dummyToPiBytes = [2047, 50, 50]

#
# now loop thru reading from and writing to Arduino
#

#df = pd.DataFrame(columns=['Time','Heading','Offset','Speed'])

file = open("/home/pi/data_log.csv", "a")

if os.stat("/home/pi/data_log.csv").st_size == 0:
    file.write("Time,Heading,Offset,Speed\n")

while True:
    time.sleep(5)
    dummyToPiFloats = getFloatData(dummyToPiFloats)
    now = datetime.now()
    file.write(str(now)+","+str(dummyToPiFloats[0])+","+str(dummyToPiFloats[1])+","+str(dummyToPiFloats[2])+"\n")
    file.flush()
#    dummyToPiBytes = getByteData(dummyToPiBytes)
#    print(dummyToPiFloats, dummyToPiBytes)
#
#   send variable to Pi
#
    writeData1 = convertASCII(writeData)
    putByteList2(writeData1)
#    putByteList2(byteListDummyFromPi2)
#    gdf.append(dummyToPiFloats)
    
#df.to_csv('log.csv')

