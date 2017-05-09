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

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
sys.path.append('/opt/robocomp/lib')
import librobocomp_qmat
import librobocomp_osgviewer
import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 3000
        self.timer.start(self.Period)

    def setParams(self, params):
        try:
            leg = params['leg']
            self.motors = ['arm{0}motor{1}'.format(leg, i+1) for i in range(3)]
            self.maxSpeed = float(params['max_speed'])
            
            self.l0= float(params['l0'])
            self.l1= float(params['l1'])
            self.l2= float(params['l2'])
            
            
            self.innermodel = librobocomp_innermodel.InnerModel(params["InnerModelPath"])           
            
           
        except:
            traceback.print_exc()
            print "Error reading config params"
            exit(1)
        return True


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

        while True in [self.jointmotor_proxy.getMotorState(m).isMoving for m in self.motors]:
            print "moviendo ..."
                        

        a0 = self.jointmotor_proxy.getMotorState(self.motors[0]).pos
        a1 = self.jointmotor_proxy.getMotorState(self.motors[1]).pos
        a2 = self.jointmotor_proxy.getMotorState(self.motors[2]).pos
        x, y, z = self.dk(a0, a1,a2)
        print "direct kinematics coords = {0}, {1}, {2}".format(x, y, z)
        
        self.innermodel.getJoint(self.motors[0]).setAngle(a0)
        self.innermodel.getJoint(self.motors[1]).setAngle(a1)
        self.innermodel.getJoint(self.motors[2]).setAngle(a2)

        
        x1, y1, z1 = self.getPosInnerModel()
        print "innermodel coords        = {0}, {1}, {2}".format(x1, y1, z1)

        print "diferencia = {0}, {1}, {2}".format(x-x1, y-y1, z-z1)     
        print "......................."
        
    
    def dk(self, a0, a1, a2):
        x = (self.l0 + self.l1*cos(a1) + self.l2*cos(a1 + a2))*cos(a0)
        y = (self.l0 + self.l1*cos(a1) + self.l2*cos(a1 + a2))*sin(a0)
        z = self.l1*sin(a1) + self.l2*sin(a1 + a2)

        return x, y, z
        
    
    def getPosInnerModel(self):
        # The API of python-innermodel is not exactly the same as the C++ version
        z = librobocomp_qmat.QVec(3,0)
        r = self.innermodel.transform("arm1motor1T", z, "arm1TipT")
        #r.printvector("d")
        return r[0], r[1], r[2]
    
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
