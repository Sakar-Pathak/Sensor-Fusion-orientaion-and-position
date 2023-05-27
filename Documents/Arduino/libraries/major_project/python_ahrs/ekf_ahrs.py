import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
#import matplotlib.pyplot as plt

from panda3d.core import loadPrcFile    #to load .prc file
loadPrcFile("config\config.prc")        #it contains configurational settings

from panda3d.core import NodePath, Loader, Quat
from direct.showbase.ShowBase import ShowBase

import serial


print("\n\n\n\n")

class Game(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.model = self.loader.loadModel("models/misc/rgbCube")
        self.model.reparentTo(render)
        self.model.setPos(0, 100, 0)
        self.model.setScale(20,20,5)

        self.taskMgr.add(self.rotate_model_task, "rotate_model_task")

        self.ser = serial.Serial('COM5', 2000000, timeout=1)
        self.ser.flushInput()
        line = self.ser.readline().decode().rstrip()  # decode bytes to string and remove trailing newline
        columns = line.split(",")  # split line into columns using comma as delimiter
        
        # wait until we get a valid line
        while len(columns) != 4:
            self.ser.flushInput()
            line = self.ser.readline().decode().rstrip()
            columns = line.split(",")

        if len(columns) == 4:  # make sure there are three columns
            try:
                w = float(columns[0])
                x = float(columns[1])
                y = float(columns[2])
                z = float(columns[3])
            
            except:
                print("Invalid line: " + line)

        # Set up the initial rotation
        initial_quat = Quat()
        
        initial_quat.setW(w)
        initial_quat.setX(z)
        initial_quat.setY(x)
        initial_quat.setZ(-y)

        self.model.setQuat(initial_quat)


    def rotate_model_task(self, task):
        self.ser.flushInput()
        line = self.ser.readline().decode().rstrip()  # decode bytes to string and remove trailing newline
        columns = line.split(",")  # split line into columns using comma as delimiter
        
        # wait until we get a valid line
        while len(columns) != 4:
            self.ser.flushInput()
            line = self.ser.readline().decode().rstrip()
            columns = line.split(",")


        if len(columns) == 4:  # make sure there are three columns
            try:
                w = float(columns[0])
                x = float(columns[1])
                y = float(columns[2])
                z = float(columns[3])

                quat = Quat()
        
                quat.setW(w)
                quat.setX(z)
                quat.setY(x)
                quat.setZ(-y)

                self.model.setQuat(quat)
            
            except:
                print("Invalid line: " + line)
        
        return task.cont


game = Game()
game.run()

print("\n\n\n\n")