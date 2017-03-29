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


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 10
        self.timer.start(self.Period)

    def setParams(self, params):
        # try:
        #	par = params["InnerModelPath"]
        #	innermodel_path=par.value
        #	innermodel = InnerModel(innermodel_path)
        # except:
        #	traceback.print_exc()
        #	print "Error reading config params"
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

    def mover(self):
        m = MotorGoalPosition()
        m.name = 'arm5motor1'
        m.position = 0.9
        m.maxSpeed = 0.2
        self.jointmotor_proxy.setSyncPosition([m])

    @QtCore.Slot()
    def compute(self):
        # print 'SpecificWorker.compute...'
        # computeCODE
        try:
            t = time.time()
            # self.leer()
            self.pruebaVelocidad()
            # self.mover()
            # sleep(1.0)
            t2 = time.time()
        # print(t2-t)
        except Ice.Exception, e:
            traceback.print_exc()
            print e
        return True
