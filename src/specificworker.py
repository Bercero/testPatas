#
# Copyright (C) 2017 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, traceback, time

from PySide import *
from genericworker import *

from random import uniform as rand
from time import sleep
from math import *

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)

    def setParams(self, params):
        try:
            leg = params['leg']
            self.motors = ['arm{0}motor{1}'.format(leg, i+1) for i in range(3)]
            self.maxSpeed = float(params['max_speed'])
            
            self.l0= float(params['l0'])
            self.l1= float(params['l1'])
            self.l2= float(params['l2'])
        
            self.innermodel = self.InnerModel(params["InnerModelPath"])
        except:
            traceback.print_exc()
            print "Error reading config params"
            exit(1)
        return True

    def leer(self):
        mstateMap = self.jointmotor_proxy.getAllMotorState()
        print mstateMap['arm5motor2'].vel

    def pruebaVelocidad(self):
        m = MotorGoalPosition()
        m.name = 'arm5motor1'
        m.position = -0.9
        m.maxSpeed = 50.0
        while True:
            m.position = -m.position
            self.jointmotor_proxy.setSyncPosition([m])
            fin = False
            while not fin:
                mstateMcap = self.jointmotor_proxy.getAllMotorState()
                print "velocidad"
                print mstateMcap[m.name].vel
                print 'posicion'
                print mstateMcap[m.name].pos
                print 'posicion objetivo'
                print m.position
            if m.position == mstateMcap[m.name].pos:
                    fin = True

    def mover(self,  pos):
        goalList = []
        for i in range (3):
            goal = MotorGoalPosition()
            goal.name = self.motors[i]
            goal.position = pos[i]
            goal.maxSpeed = self.maxSpeed
            goalList.append(goal)
       
        self.jointmotor_proxy.setSyncPosition(goalList)


    def pruebaPosiciones(self):
        goalPosList = [rand(-pi/2, pi/2) for x in range(3)]
        self.mover(goalPosList)
        print "................\nmoviendo ..."
        sleep(3)
    
        a0 = self.jointmotor_proxy.getMotorState(self.motors[0]).pos
        a1 = self.jointmotor_proxy.getMotorState(self.motors[1]).pos
        a2 = self.jointmotor_proxy.getMotorState(self.motors[2]).pos
        x, y, z = self.dk(a0, a1,a2)
        print "x = {0}".format(x)
        print "y = {0}".format(y)
        print "z = {0}".format(z)
        print "......................."
        
        #self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
        z = coord3D()
        r = self.innermodel.transform("arm1motor1T", z, "arm1TipT")
        r.printvector("d")
        print r[0], r[1], r[2]
        sleep(2)
        
    
    def dk(self, a0, a1, a2):
        x = (self.l0 + self.l1*cos(a1) + self.l2*cos(a1 + a2))*cos(a0)
        y = (self.l0 + self.l1*cos(a1) + self.l2*cos(a1 + a2))*sin(a0)
        z = self.l1*sin(a1) + self.l2*sin(a1 + a2)

        return x, y, z
        
    @QtCore.Slot()
    def compute(self):
        # print 'SpecificWorker.compute...'
        # computeCODE
        try:
           self.pruebaPosiciones()
        except Ice.Exception, e:
            traceback.print_exc()
            print e
        return True
