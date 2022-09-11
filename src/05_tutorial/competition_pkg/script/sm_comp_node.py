#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2022 Hibikino-Musashi@Home
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Hibikino-Musashi@Home nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import os
import roslib
import smach
import smach_ros

import sm_comp_main as sm

sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *

sys.path.append(roslib.packages.get_pkg_dir("hma_sm_pkg") + "/script/src/lib")
from libsm import *

sys.path.append(roslib.packages.get_pkg_dir("hma_lib_pkg") + "/script/src/lib")
from libutil import *
from libopencv import *
from libtf import *
from libpub import *
from libaction import *

sys.path.append(roslib.packages.get_pkg_dir("hma_yolact_pkg") + "/script/src/lib")
from libyolact import *

sys.path.append(roslib.packages.get_pkg_dir("hma_hsr_nav_pkg") + "/script/src/lib")
from libhsrnav import *

sys.path.append(roslib.packages.get_pkg_dir("hma_hsr_sim_lib_pkg") + "/script/src/lib")
from libhsrutil import *
from libhsrpub import *
from libhsrsub import *
from libhsraction import *
from libhsrmoveit import *


class StateMachine:
    def __init__(self):
        libsm = LibSM()

        libutil = LibUtil()
        libopencv = LibOpenCV()
        libtf = LibTF()
        libpub = LibPub()
        libaction = LibAction(libtf)
        libyolact = LibYolact(True)

        libhsrutil = LibHSRUtil(libutil, libtf)
        libhsrnav = LibHSRNav(libutil, libtf, libhsrutil)
        libhsrpub = LibHSRPub()
        libhsrsub = LibHSRSub(libutil, libopencv)
        libhsraction = LibHSRAction()
        libhsrmoveit = LibHSRMoveit(libtf, libhsrutil, libhsrpub)

        self.lib = {"sm": libsm,
                    "util": libutil,
                    "opencv": libopencv,
                    "tf": libtf,
                    "pub": libpub,
                    "action": libaction,
                    "yolact": libyolact,
                    "hsrutil": libhsrutil,
                    "hsrnav": libhsrnav,
                    "hsrpub": libhsrpub,
                    "hsrsub": libhsrsub,
                    "hsraction": libhsraction,
                    "hsrmoveit": libhsrmoveit}
        self.robot_descriptor = self.lib["util"].get_robot_descriptor()
        rospy.sleep(1.0)

        self.ssm = smach.StateMachine(outcomes=["exit"])

        with self.ssm:
            smach.StateMachine.add(
                "Init",
                sm.Init(self.lib, self.robot_descriptor), 
                transitions={
                    "next": "Wait4Start",
                    "except": "Except"})
            smach.StateMachine.add(
                "Wait4Start",
                sm.Wait4Start(self.lib, self.robot_descriptor),
                transitions={
                    "next": "Start",
                    "except": "Except"})
            smach.StateMachine.add(
                "Start",
                sm.Start(self.lib, self.robot_descriptor),
                transitions={
                    "next": "Task1",
                    "except": "Except"})
            smach.StateMachine.add(
                "Task1",
                sm.Task1(self.lib, self.robot_descriptor),
                transitions={
                    "next": "End",
                    "except": "Except"})
            smach.StateMachine.add(
                "End",
                sm.End(self.lib, self.robot_descriptor),
                transitions={
                    "end": "exit"})
            smach.StateMachine.add(
                "Except",
                sm.Except(self.lib, self.robot_descriptor),
                transitions={
                    "except": "exit"})

        sris = smach_ros.IntrospectionServer("ssm", self.ssm, "/SM_ROOT")
        sris.start()
        return

    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        del self.ssm
        return

    def main(self):
        self.ssm.execute()
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    state_machine = StateMachine()
    rospy.on_shutdown(state_machine.delete)
    state_machine.main()
    rospy.signal_shutdown("")
