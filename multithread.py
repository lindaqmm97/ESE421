# threadtest.py

import time
import threading

# this is something like navigation loop
def slowProcess():
    global steadyX
    while True:
        steadyX = (steadyX + 5.0) % 360
        time.sleep(1)

# this is something like control loop
def fastProcess():
    global transientX
    while True:
        error = 0.05*((steadyX - transientX) % 360)
        transientX = (transientX + error) % 360
        time.sleep(0.1)

steadyX = 150.0
transientX = steadyX

# "Daemon" = runs forever with forced quit when main quits
slowT = threading.Thread(target=slowProcess)
slowT.setDaemon(True)
slowT.start()
fastT = threading.Thread(target=fastProcess)
fastT.setDaemon(True)
fastT.start()

while True:
    print steadyX, transientX